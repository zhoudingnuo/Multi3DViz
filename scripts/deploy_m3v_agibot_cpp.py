#!/usr/bin/env python3
"""Deploy + compile m3v_agibot.cpp on the Agibot D1.

m3v_agibot is the C++ persistent velocity controller for the Agibot (the
counterpart to m3v_move on the Unitree). It reads commands from stdin and
calls the mc_sdk C++ HighLevel API — same protocol as m3v_move:
    "stand" / "lie" / "stop" / "passive" / "vx vy yaw"

Why C++ instead of Python: the Agibot's Python SDK binding has known issues
(关节角返回垃圾值 §8.5), and the control doc explicitly says standUp/lieDown
were only verified in C++ (stand_liedown.cpp). C++ is the verified path.

Key safety behavior (matches m3v_move):
  - After initRobot, wait up to 6s for checkConnect() before accepting cmds.
  - Watchdog: if no command received in 500ms, send move(0,0,0).
  - stdin close → safe shutdown (zero vel → lieDown → passive).
  - NO auto-stand on startup — dog stays passive until "stand" is sent.

Usage:
    python scripts/deploy_m3v_agibot_cpp.py [host] [user]
Defaults: host=10.60.77.154 user=orin-001 (key auth)
"""
import paramiko
import sys

CPP_CODE = r'''// m3v_agibot.cpp — persistent Agibot D1 velocity controller (C++).
// stdin protocol (same as m3v_move on Unitree):
//   "stand"       → standUp()
//   "lie"         → lieDown()
//   "stop"        → move(0,0,0)
//   "passive"     → passive() (emergency: motors release, dog drops)
//   "vx vy yaw"   → move(vx, vy, yaw)
//
// Safety: watchdog sends move(0,0,0) if no command in 500ms. stdin close
// → safe shutdown (zero + lieDown + passive). No auto-stand on startup.
//
// Based on stand_liedown.cpp (verified 2026-07-13) + m3v_move watchdog design.
#include "zsl-1/highlevel.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <vector>

using namespace mc_sdk::zsl_1;

static HighLevel* g_app = nullptr;
static std::atomic<bool> g_running(true);
static std::atomic<long long> g_last_cmd_ms(0);

static long long now_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Watchdog: if no command in 500ms, force move(0,0,0). Ensures the dog
// NEVER keeps moving if the controller disconnects or the network drops.
static void watchdog() {
  while (g_running) {
    sleep_ms(100);
    long long elapsed = now_ms() - g_last_cmd_ms.load();
    if (elapsed > 500 && g_app != nullptr) {
      g_app->move(0.0f, 0.0f, 0.0f);
      // Re-trigger in 100ms unless a new command arrives.
      g_last_cmd_ms.store(now_ms() - 400);
    }
  }
}

int main() {
  HighLevel app;
  g_app = &app;
  app.initRobot("192.168.234.18", 43988, "192.168.234.1");
  fprintf(stderr, "m3v_agibot: init done, waiting for link...\n");

  // Wait for link readiness (mirrors stand_liedown.cpp). Don't accept
  // commands until checkConnect() returns true — sending standUp before
  // the link is up is silently dropped by mc_ctrl.
  bool ok = false;
  for (int i = 0; i < 20; ++i) {
    sleep_ms(300);
    if (app.checkConnect()) { ok = true; break; }
  }
  if (!ok) {
    fprintf(stderr, "m3v_agibot: ERROR link not ready after 6s — aborting "
            "(no motion sent)\n");
    fflush(stderr);
    return 1;
  }
  fprintf(stderr, "m3v_agibot: link OK, SAFE MODE (no auto-stand, "
          "watchdog active)\n");
  fflush(stderr);

  // Start watchdog thread.
  std::thread wd(watchdog);
  wd.detach();

  // Main loop: read commands from stdin.
  // (Use getline for thread-safe line reads. Watchdog runs in parallel.)
  char line[256];
  while (std::cin.getline(line, sizeof(line))) {
    g_last_cmd_ms.store(now_ms());

    if (strcmp(line, "stand") == 0) {
      uint32_t r = app.standUp();
      fprintf(stderr, "m3v_agibot: standUp ret=0x%x\n", r);
      fflush(stderr);
      sleep_ms(500);  // give the state machine time to transition
    } else if (strcmp(line, "lie") == 0) {
      app.move(0.0f, 0.0f, 0.0f);
      sleep_ms(300);
      uint32_t r = app.lieDown();
      fprintf(stderr, "m3v_agibot: lieDown ret=0x%x\n", r);
      fflush(stderr);
      sleep_ms(500);
    } else if (strcmp(line, "stop") == 0) {
      app.move(0.0f, 0.0f, 0.0f);
      fprintf(stderr, "m3v_agibot: stop\n");
      fflush(stderr);
    } else if (strcmp(line, "passive") == 0) {
      uint32_t r = app.passive();
      fprintf(stderr, "m3v_agibot: PASSIVE (emergency) ret=0x%x\n", r);
      fflush(stderr);
    } else {
      float vx, vy, yaw;
      if (sscanf(line, "%f %f %f", &vx, &vy, &yaw) == 3) {
        uint32_t r = app.move(vx, vy, yaw);
        // Don't log every velocity packet (10Hz flood). Only log errors.
        if (r != 0) {
          fprintf(stderr, "m3v_agibot: move(%.2f,%.2f,%.2f) ret=0x%x\n",
                  vx, vy, yaw, r);
          fflush(stderr);
        }
      }
    }
  }

  // stdin closed — controller disconnected. SAFE SHUTDOWN.
  g_running = false;
  app.move(0.0f, 0.0f, 0.0f);
  sleep_ms(300);
  app.lieDown();
  sleep_ms(500);
  app.passive();
  fprintf(stderr, "m3v_agibot: stdin closed, safe shutdown done\n");
  fflush(stderr);
  return 0;
}
'''

COMPILE_CMD = (
    'g++ -std=c++17 -o /home/orin-001/m3v_agibot '
    '/home/orin-001/m3v_agibot.cpp '
    '-I /home/orin-001/ZCodeProject/include '
    '-L /home/orin-001/ZCodeProject/lib/zsl-1/aarch64 '
    '-lmc_sdk_zsl_1_aarch64 '
    "-Wl,-rpath,/home/orin-001/ZCodeProject/lib/zsl-1/aarch64 2>&1"
)


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.60.77.154"
    user = sys.argv[2] if len(sys.argv) > 2 else "orin-001"
    print(f"connecting to {user}@{host} ...")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, username=user, timeout=8)

    # Write the C++ source via SFTP (raw bytes, no shell mangling).
    sftp = c.open_sftp()
    with sftp.file("/home/orin-001/m3v_agibot.cpp", "w") as f:
        f.write(CPP_CODE)
    sftp.close()
    print("m3v_agibot.cpp written")

    # Compile.
    print("compiling...")
    _, o, e = c.exec_command(COMPILE_CMD, timeout=60)
    o.channel.recv_exit_status()
    out = o.read().decode("utf-8", "replace").rstrip()
    err = e.read().decode("utf-8", "replace").rstrip()
    if out:
        print(out)
    if err:
        print("[stderr]", err)

    # Check binary.
    _, o2, _ = c.exec_command("ls -la /home/orin-001/m3v_agibot 2>&1",
                              timeout=5)
    o2.channel.recv_exit_status()
    print(o2.read().decode().rstrip())

    c.close()
    print("done")


if __name__ == "__main__":
    main()
