// SPDX-License-Identifier: MIT
// Board power transitions and opt-in reset diagnostics. Normal macOS shutdown
// uses the platform callback; BootLog arguments control diagnostic reset paths.
#include <kern/thread.h>
#include <machine/machine_routines.h>
#include <string.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>
#include <kern/sched.h>
#include <pexpert/pexpert.h>
#include "BootResetCore.h"
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#include "BootPower.h"
#endif

// Physical windows from the sheng devicetree (spmi@c400000 reg, reg-names core/chnls/obsrvr/cnfg).
enum {SpmiCore = 0xc400000, SpmiCoreSize = 0x3000, SpmiObsrvr = 0xc440000, SpmiChnls = 0xc500000, SpmiCnfg = 0xc42d000, SpmiCnfgSize = 0x4000};
enum {PsciVersion = 0x84000000u, PsciFeatures = 0x8400000au, PsciSystemReset = 0x84000009u, PsciSystemReset2_32 = 0x84000012u,
      PsciSystemReset2_64 = 0xc4000012u};

// SMCCC call with two arguments; x0 and x2 come back (the Gunyah watchdog's status reports the time since the last ping in x2).
static uint64_t smc2(uint64_t fid, uint64_t a1, uint64_t a2, uint64_t *r2) {
  register uint64_t x0 __asm__("x0") = fid, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = 0;
  __asm__ volatile("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
                   "x14", "x15", "x16", "x17");
  if (r2) *r2 = x2;
  return x0;
}

static uint64_t psci(uint64_t fid, uint64_t a1) {
  register uint64_t x0 __asm__("x0") = fid, x1 __asm__("x1") = a1, x2 __asm__("x2") = 0, x3 __asm__("x3") = 0;
  __asm__ volatile("smc #0" : "+r"(x0), "+r"(x1), "+r"(x2), "+r"(x3) :: "memory", "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13",
                   "x14", "x15", "x16", "x17");
  return x0;
}

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
static int platform_off_available, platform_reset_available;
static int platform_power_invoke(uint32_t fid) {
  const int64_t status = (int64_t)psci(fid, 0);
  // A completed PSCI transition never returns to this kernel.
  IOLog("[power] PSCI %x returned %lld\n", fid, (long long)status);
  return -1;
}
static int platform_halt_restart(unsigned type) {
  IOLog("[power] platform request %u off=%d reset=%d\n", type,
        platform_off_available, platform_reset_available);
  return boot_power_dispatch(type, platform_off_available, platform_reset_available,
                             platform_power_invoke);
}
void boot_reset_install_platform(void) {
  platform_off_available = (int32_t)psci(PsciFeatures, BootPowerOff) >= 0;
  platform_reset_available = (int32_t)psci(PsciFeatures, BootPowerReset) >= 0;
  int (*expected)(unsigned) = NULL;
  const int installed = __atomic_compare_exchange_n(&PE_halt_restart, &expected,
      platform_halt_restart, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
  IOLog("[power] platform handler %s off=%d reset=%d\n",
        installed ? "installed" : "retained", platform_off_available, platform_reset_available);
}
#endif

// ml_io_map hands out permanent mappings (io_submap, KMA_PERMANENT) that ml_io_unmap cannot free: its kmem_free on kernel_map panics with the
// map lock held (int-105 hung launchd that way). Mappings that are unmapped again must be unmappable ones (flags 7 = VM_WIMG_IO).
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, uint32_t flags);
static volatile uint32_t *map(uint64_t phys, uint64_t size) {return (volatile uint32_t *)ml_io_map_unmappable((vm_offset_t)phys, (vm_size_t)size, 7);}

struct reset_maps {
  struct br_arbiter a;
  struct br_found f;
  volatile uint32_t *obs, *chn;
  uint64_t obs_size, chn_size;
  int find;   // br_find's result, -1 before it ran
};
static void unmap_all(struct reset_maps *m) {
  if (m->a.core) ml_io_unmap((vm_offset_t)m->a.core, SpmiCoreSize);
  if (m->a.cnfg) ml_io_unmap((vm_offset_t)m->a.cnfg, SpmiCnfgSize);
  if (m->obs) ml_io_unmap((vm_offset_t)m->obs, (vm_size_t)m->obs_size);
  if (m->chn) ml_io_unmap((vm_offset_t)m->chn, (vm_size_t)m->chn_size);
  memset(m, 0, sizeof *m);
}
// Maps the arbiter, finds the SDAM's APID and maps its observer (and, when asked, read/write) channels. 0 = ready.
static int open_maps(struct reset_maps *m, int want_write) {
  memset(m, 0, sizeof *m);
  m->find = -1;
  m->a.ee = 0;   // qcom,ee = <0>
  m->a.core = map(SpmiCore, SpmiCoreSize);
  m->a.cnfg = map(SpmiCnfg, SpmiCnfgSize);
  if (!m->a.core || !m->a.cnfg) return 1;
  m->find = br_find(&m->a, &m->f);
  if (m->find) return 2;
  m->obs_size = ((uint64_t)BR_OBS_APID_STRIDE * ((uint64_t)m->f.apid + 1) + 0xfff) & ~0xfffull;
  m->obs = map(SpmiObsrvr, m->obs_size);
  if (!m->obs) return 3;
  if (want_write) {
    m->chn_size = (uint64_t)BR_CHNL_APID_STRIDE * ((uint64_t)m->f.apid + 1);
    m->chn = map(SpmiChnls, m->chn_size);
    if (!m->chn) return 4;
  }
  return 0;
}

// The arbiter and the SDAM cell, read only. Fills text; returns 0 when the cell can be written (version known, APID found, owned by this EE).
int boot_reset_probe(char *text, unsigned cap) {
  const uint64_t ver = psci(PsciVersion, 0);
  const int64_t f_reset = (int64_t)psci(PsciFeatures, PsciSystemReset), f2_32 = (int64_t)psci(PsciFeatures, PsciSystemReset2_32),
                f2_64 = (int64_t)psci(PsciFeatures, PsciSystemReset2_64);
  struct reset_maps m;
  uint8_t cell = 0xff;
  int open_rc = open_maps(&m, 0), read_rc = -1;
  if (!open_rc) read_rc = br_read8(m.obs, &m.a, m.f.apid, BR_REBOOT_CELL, &cell);
  const int writable = !open_rc && !read_rc && m.f.owner == m.a.ee;
  snprintf(text, cap, "psci v%x reset=%lld reset2_32=%lld reset2_64=%lld | spmi v%08x apids=%u open=%d find=%d apid=%d owner=%d sdam48=%02x read=%d ok=%d",
           (unsigned)ver, (long long)f_reset, (long long)f2_32, (long long)f2_64, m.a.core ? m.f.version : 0, m.a.core ? m.f.apids : 0, open_rc,
           m.find, m.find ? -1 : m.f.apid, m.find ? -1 : m.f.owner, cell, read_rc, writable);
  unmap_all(&m);
  return writable ? 0 : 1;
}

// The PMK8550 power-on peripherals (SID 0, pmk8550.dtsi pon@1300 reg <0x1300>, <0x800>, "hlos" and "pbs"): the power-on, warm-reset,
// power-off and fault reason registers the PMIC latched for the last transition, raw (bytes 0x04..0x1f of each), for decoding off the
// device. Observer reads only; nothing is written.
int boot_reset_pon(char *text, unsigned cap) {
  static const uint16_t ppids[2] = {0x008, 0x013};
  static const char *const names[2] = {"pbs", "hlos"};
  struct br_arbiter a;
  memset(&a, 0, sizeof a);
  a.core = map(SpmiCore, SpmiCoreSize);
  a.cnfg = map(SpmiCnfg, SpmiCnfgSize);
  int apid[2] = {-1, -1}, n = 0;
  volatile uint32_t *obs = NULL;
  uint64_t obs_size = 0;
  if (!a.core || !a.cnfg) {snprintf(text, cap, "pon: arbiter not mapped"); goto out;}
  {
    const uint32_t version = a.core[0], apids = a.core[BR_FEATURES / 4] & BR_FEATURES_APIDS_MASK;
    if (version < BR_VERSION_V7_MIN || version >= BR_VERSION_V8_MIN || !apids || apids > 1024) {
      snprintf(text, cap, "pon: arbiter %08x apids %u not known", version, apids); goto out;
    }
    int top = -1;
    for (uint32_t i = 0; i < apids; ++i) {
      const uint32_t m = a.core[(BR_APID_MAP + 4 * i) / 4];
      if (!m) continue;
      for (int k = 0; k < 2; ++k)
        if (((m >> 8) & 0xfff) == ppids[k] && apid[k] < 0) {apid[k] = (int)i; if ((int)i > top) top = (int)i;}
    }
    if (top < 0) {snprintf(text, cap, "pon: no apid for 0x008 or 0x013"); goto out;}
    obs_size = ((uint64_t)BR_OBS_APID_STRIDE * ((uint64_t)top + 1) + 0xfff) & ~0xfffull;
    obs = map(SpmiObsrvr, obs_size);
    if (!obs) {snprintf(text, cap, "pon: observer not mapped"); goto out;}
  }
  n = snprintf(text, cap, "pon");
  for (int k = 0; k < 2 && n > 0 && (unsigned)n < cap; ++k) {
    n += snprintf(text + n, cap - (unsigned)n, " %s(apid %d) 04:", names[k], apid[k]);
    if (apid[k] < 0) continue;
    for (uint8_t r = 0x04; r < 0x20 && (unsigned)n < cap; ++r) {
      uint8_t v = 0;
      const int rc = br_read8(obs, &a, apid[k], r, &v);
      n += rc ? snprintf(text + n, cap - (unsigned)n, " --") : snprintf(text + n, cap - (unsigned)n, " %02x", v);
    }
  }
out:
  if (a.core) ml_io_unmap((vm_offset_t)a.core, SpmiCoreSize);
  if (a.cnfg) ml_io_unmap((vm_offset_t)a.cnfg, SpmiCnfgSize);
  if (obs) ml_io_unmap((vm_offset_t)obs, (vm_size_t)obs_size);
  return 0;
}

// The PMIC PON log (Gen3, PMK8350 and later): the PBS firmware appends power-on, power-off, reset and fault events to a FIFO in one of
// the PMK8550 SDAM blocks (downstream pmic-pon-log.c: push pointer at 0x46, 4-byte entries {state, event, data1, data0} in 0x4B..0xBF).
// Which SDAM holds it is not known here, so every SDAM-typed peripheral of SID 0 between 0x7000 and 0x9F00 (peripheral type 0x2E) has
// its 0x46 printed, and the FIFO of each whose pointer lies in 0x4B..0xC0 is dumped raw (at most three), decoded off the device. Reads only.
int boot_reset_ponlog(char *text, unsigned cap) {
  struct br_arbiter a;
  memset(&a, 0, sizeof a);
  a.core = map(SpmiCore, SpmiCoreSize);
  a.cnfg = map(SpmiCnfg, SpmiCnfgSize);
  int apid[0x30], n = 0, dumped = 0;
  for (int k = 0; k < 0x30; ++k) apid[k] = -1;
  volatile uint32_t *obs = NULL;
  uint64_t obs_size = 0;
  if (!a.core || !a.cnfg) {snprintf(text, cap, "ponlog: arbiter not mapped"); goto out;}
  {
    const uint32_t version = a.core[0], apids = a.core[BR_FEATURES / 4] & BR_FEATURES_APIDS_MASK;
    if (version < BR_VERSION_V7_MIN || version >= BR_VERSION_V8_MIN || !apids || apids > 1024) {
      snprintf(text, cap, "ponlog: arbiter %08x apids %u not known", version, apids); goto out;
    }
    int top = -1;
    for (uint32_t i = 0; i < apids; ++i) {
      const uint32_t m = a.core[(BR_APID_MAP + 4 * i) / 4];
      if (!m) continue;
      const uint32_t ppid = (m >> 8) & 0xfff;
      if (ppid >= 0x070 && ppid < 0x0a0 && apid[ppid - 0x070] < 0) {apid[ppid - 0x070] = (int)i; if ((int)i > top) top = (int)i;}
    }
    if (top < 0) {snprintf(text, cap, "ponlog: no apid for 0x070..0x09f"); goto out;}
    obs_size = ((uint64_t)BR_OBS_APID_STRIDE * ((uint64_t)top + 1) + 0xfff) & ~0xfffull;
    obs = map(SpmiObsrvr, obs_size);
    if (!obs) {snprintf(text, cap, "ponlog: observer not mapped"); goto out;}
  }
  n = snprintf(text, cap, "ponlog ptr");
  for (int k = 0; k < 0x30 && n > 0 && (unsigned)n < cap; ++k) {
    if (apid[k] < 0) continue;
    uint8_t type = 0, ptr = 0;
    if (br_read8(obs, &a, apid[k], 0x04, &type) || type != 0x2e) continue;
    if (br_read8(obs, &a, apid[k], 0x46, &ptr)) continue;
    n += snprintf(text + n, cap - (unsigned)n, " %02x:%02x", 0x70 + k, ptr);
  }
  for (int k = 0; k < 0x30 && dumped < 3 && n > 0 && (unsigned)n < cap; ++k) {
    if (apid[k] < 0) continue;
    uint8_t type = 0, ptr = 0;
    if (br_read8(obs, &a, apid[k], 0x04, &type) || type != 0x2e) continue;
    if (br_read8(obs, &a, apid[k], 0x46, &ptr) || ptr < 0x4b || ptr > 0xc0) continue;
    n += snprintf(text + n, cap - (unsigned)n, " | sdam %02x fifo 4b:", 0x70 + k);
    for (uint8_t r = 0x4b; r <= 0xbf && n > 0 && (unsigned)n < cap; ++r) {
      uint8_t v = 0;
      n += br_read8(obs, &a, apid[k], r, &v) ? snprintf(text + n, cap - (unsigned)n, "--") : snprintf(text + n, cap - (unsigned)n, "%02x", v);
    }
    ++dumped;
  }
out:
  if (a.core) ml_io_unmap((vm_offset_t)a.core, SpmiCoreSize);
  if (a.cnfg) ml_io_unmap((vm_offset_t)a.cnfg, SpmiCnfgSize);
  if (obs) ml_io_unmap((vm_offset_t)obs, (vm_size_t)obs_size);
  return 0;
}

// Writes "bootloader" into the reboot-reason cell and verifies it, without restarting. Done once early in an unattended run, it sends every
// later restart to fastboot: the timed one below, a watchdog bite, or a long press of the power key.
int boot_reset_mark_bootloader(char *text, unsigned cap) {
  struct reset_maps m;
  uint8_t before = 0xff, after = 0xff;
  int rc = open_maps(&m, 1);
  if (rc) {snprintf(text, cap, "mark refused: open=%d find=%d", rc, m.find); unmap_all(&m); return rc;}
  if (m.f.owner != m.a.ee) {snprintf(text, cap, "mark refused: apid %d is owned by EE %d", m.f.apid, m.f.owner); unmap_all(&m); return 5;}
  rc = br_set_bootloader(m.obs, m.chn, &m.a, m.f.apid, &before, &after);
  snprintf(text, cap, "mark: cell %02x -> %02x rc=%d", before, after, rc);
  unmap_all(&m);
  return rc ? 10 + rc : 0;
}

// Gunyah accepts watchdog timeouts up to 32000 ms. Firmware hands over an armed 30000 ms timer;
// this kernel pings it every 10 s until the unattended deadline. A zero deadline keeps it running.
_Static_assert(BR_WDT_TIMEOUT_MS <= 32000, "Gunyah watchdog timeout exceeds the hypervisor limit");
volatile int32_t boot_reset_wdt_stats[4];   // [0] status call result, [1] set-time result, [2] enable result, [3] pings sent
static uint32_t wdt_deadline;
static unsigned wdt_stopped;
static unsigned wdt_pinging;   // 1 while wdt_thread pings
void boot_reset_watchdog_stop(void) {
  __atomic_store_n(&wdt_stopped, 1, __ATOMIC_RELEASE);
}
static uint64_t watchdog_ticks(void) {
  uint64_t value;
  __asm__ volatile("isb\n\tmrs %0, cntpct_el0" : "=r"(value));
  return value;
}
static void wdt_thread(void *arg, wait_result_t wr) {
  (void)arg; (void)wr;
  thread_set_thread_name(current_thread(), "boot_watchdog");
  uint64_t f = 0; __asm__ volatile("mrs %0, cntfrq_el0" : "=r"(f)); if (!f) f = 19200000;
  const uint64_t start = watchdog_ticks();
  __atomic_store_n(&wdt_pinging, 1, __ATOMIC_RELEASE);
  while (!__atomic_load_n(&wdt_stopped, __ATOMIC_ACQUIRE) &&
         (!wdt_deadline || (watchdog_ticks() - start) / f < wdt_deadline)) {
    smc2(BR_WDT_PING, 0, 0, NULL);
    boot_reset_wdt_stats[3] = boot_reset_wdt_stats[3] + 1;
    IOSleep(10000);
  }
  __atomic_store_n(&wdt_pinging, 0, __ATOMIC_RELEASE);
}
// Platform sleep (ShengSleep.cpp, through ArmCpu's board tick every 10 s): the ping wdt_thread cannot send while the system sleeps, sent
// from the boot core with interrupts masked. Nothing unless that thread is pinging (the watchdog taken over, not stopped, its deadline
// not passed).
void boot_reset_watchdog_ping(void) {
  if (!__atomic_load_n(&wdt_pinging, __ATOMIC_ACQUIRE) || __atomic_load_n(&wdt_stopped, __ATOMIC_ACQUIRE)) return;
  smc2(BR_WDT_PING, 0, 0, NULL);
  boot_reset_wdt_stats[3] = boot_reset_wdt_stats[3] + 1;
}
int boot_reset_watchdog(uint32_t deadline, char *text, unsigned cap) {
  uint64_t since = 0;
  const int64_t status = (int64_t)smc2(BR_WDT_STATUS, 0, 0, &since);
  boot_reset_wdt_stats[0] = (int32_t)status;
  if (status != 0) {snprintf(text, cap, "watchdog: status %lld, not armed", (long long)status); return 1;}
  uint32_t inherited = 0;
  PE_parse_boot_argn("uefi-watchdog", &inherited, sizeof inherited);
  if (inherited) {
    boot_reset_wdt_stats[1] = (int32_t)smc2(BR_WDT_PING, 0, 0, NULL);
  } else {
    smc2(BR_WDT_CONTROL, BR_WDT_DISABLE, 0, NULL);
    boot_reset_wdt_stats[1] = (int32_t)smc2(BR_WDT_SET_TIME, BR_WDT_TIMEOUT_MS, BR_WDT_TIMEOUT_MS, NULL);
  }
  if (boot_reset_wdt_stats[1]) {snprintf(text, cap, "watchdog: prepare %d", boot_reset_wdt_stats[1]); return 2;}
  boot_reset_wdt_stats[2] = inherited ? 0 : (int32_t)smc2(BR_WDT_CONTROL, BR_WDT_ENABLE, 0, NULL);
  if (boot_reset_wdt_stats[2]) {snprintf(text, cap, "watchdog: enable %d", boot_reset_wdt_stats[2]); return 4;}
  wdt_deadline = deadline;
  thread_t thread;
  if (kernel_thread_start_priority(wdt_thread, NULL, MAXPRI_KERNEL, &thread) != KERN_SUCCESS) {snprintf(text, cap, "watchdog: no thread"); return 3;}
  thread_deallocate(thread);
  snprintf(text, cap, "watchdog: armed=1 inherited=%u timeout=%u ms deadline=%us last-ping=%llu ms", inherited,
           BR_WDT_TIMEOUT_MS, deadline, (unsigned long long)since);
  return 0;
}

// Writes "bootloader" into the reboot-reason cell, verifies it, then PSCI SYSTEM_RESET. Returns only when something refused (text says what);
// the cell is left as it was in every refusal before the write.
int boot_reset_bootloader(char *text, unsigned cap) {
  struct reset_maps m;
  uint8_t before = 0xff, after = 0xff;
  int rc = open_maps(&m, 1);
  if (rc) {
    snprintf(text, cap, "reset refused: open=%d find=%d", rc, m.find);
    unmap_all(&m);
    return rc;
  }
  if (m.f.owner != m.a.ee) {
    snprintf(text, cap, "reset refused: apid %d is owned by EE %d", m.f.apid, m.f.owner);
    unmap_all(&m);
    return 5;
  }
  rc = br_set_bootloader(m.obs, m.chn, &m.a, m.f.apid, &before, &after);
  snprintf(text, cap, "reset: cell %02x -> %02x rc=%d", before, after, rc);
  unmap_all(&m);
  if (rc) return 10 + rc;
  psci(PsciSystemReset, 0);
  return 99;   // SYSTEM_RESET returned
}
