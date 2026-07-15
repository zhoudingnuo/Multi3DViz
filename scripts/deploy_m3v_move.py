#!/usr/bin/env python3
"""Deploy m3v_move.cpp to Unitree and compile it.
m3v_move is a persistent process that reads 'vx vy yaw' from stdin and
calls sport_client.Move() — no DDS re-init per command (unlike a2_sport_client
which hardcodes Move(0,0,0.5) and re-inits DDS each launch).
"""
import paramiko, sys, time

CPP_CODE = """#include <iostream>
#include <string>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <unitree/robot/a2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
using namespace std;

// Safety: watchdog sends zero velocity if no command received in 500ms.
// This ensures the dog NEVER keeps moving if the controller disconnects
// or the network drops — it auto-stops within half a second.
atomic<bool> g_running(true);
atomic<double> g_vx(0), g_vy(0), g_yaw(0);
atomic<long long> g_last_cmd_ms(0);
mutex g_stdout_mtx;

long long now_ms() {
    auto t = chrono::steady_clock::now();
    return chrono::duration_cast<chrono::milliseconds>(t.time_since_epoch()).count();
}

void watchdog(unitree::robot::a2::SportClient* sc) {
    while (g_running) {
        usleep(100000); // 100ms tick
        long long elapsed = now_ms() - g_last_cmd_ms.load();
        if (elapsed > 500) {
            // No command in 500ms — force stop (safety).
            sc->Move(0.0, 0.0, 0.0);
            g_last_cmd_ms.store(now_ms() - 400); // re-trigger in 100ms unless new cmd
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 2) { cerr << "Usage: " << argv[0] << " networkInterface" << endl; return 1; }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::a2::SportClient sport_client;
    sport_client.SetTimeout(3.0f);
    sport_client.Init();
    cerr << "m3v_move ready (SAFE MODE: no auto-stand, watchdog active)" << endl;

    // Start watchdog thread — ensures dog stops if controller disconnects.
    thread wd(watchdog, &sport_client);
    wd.detach();

    // Main loop: read commands from stdin.
    // Commands:
    //   "stand"         → recovery_stand (dog stands up)
    //   "lie"           → stand_down (dog lies down)
    //   "stop"          → stop_move + zero velocity
    //   "damp"          → damp (emergency, motors release)
    //   "vx vy yaw"     → Move(vx, vy, yaw) with watchdog protection
    string line;
    while (getline(cin, line)) {
        if (line == "stand") {
            sport_client.RecoveryStand();
            cerr << "cmd: stand" << endl;
        } else if (line == "lie") {
            sport_client.Move(0.0, 0.0, 0.0);
            sport_client.StandDown();
            g_vx.store(0); g_vy.store(0); g_yaw.store(0);
            cerr << "cmd: lie" << endl;
        } else if (line == "stop") {
            sport_client.StopMove();
            sport_client.Move(0.0, 0.0, 0.0);
            g_vx.store(0); g_vy.store(0); g_yaw.store(0);
            cerr << "cmd: stop" << endl;
        } else if (line == "damp") {
            sport_client.Damp();
            cerr << "cmd: damp (EMERGENCY)" << endl;
        } else {
            double vx, vy, yaw;
            if (sscanf(line.c_str(), "%lf %lf %lf", &vx, &vy, &yaw) == 3) {
                g_vx.store(vx); g_vy.store(vy); g_yaw.store(yaw);
                g_last_cmd_ms.store(now_ms());
                sport_client.Move(vx, vy, yaw);
            }
        }
    }

    // stdin closed — controller disconnected. SAFE SHUTDOWN: stop + lie down.
    g_running = false;
    sport_client.Move(0.0, 0.0, 0.0);
    usleep(200000);
    sport_client.StandDown();
    usleep(500000);
    sport_client.Damp();
    cerr << "m3v_move: stdin closed, dog damped (safe shutdown)" << endl;
    return 0;
}
"""

COMPILE_CMD = (
    'g++ -std=c++14 -o /home/unitree/m3v_move /home/unitree/m3v_move.cpp '
    '-I /home/unitree/unitree_sdk2-main/include '
    '-I /home/unitree/unitree_sdk2-main/thirdparty/include/ddscxx '
    '/home/unitree/unitree_sdk2-main/lib/aarch64/libunitree_sdk2.a '
    '-L /home/unitree/unitree_sdk2-main/thirdparty/lib/aarch64 -lddscxx -lddsc '
    '-lpthread -lstdc++ '
    '-Wl,-rpath,/home/unitree/unitree_sdk2-main/thirdparty/lib/aarch64 2>&1'
)

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.60.77.187"
    user = sys.argv[2] if len(sys.argv) > 2 else "unitree"
    pw = sys.argv[3] if len(sys.argv) > 3 else "123"
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, username=user, password=pw, timeout=8)

    # Write source
    sftp = c.open_sftp()
    with sftp.file('/home/unitree/m3v_move.cpp', 'w') as f:
        f.write(CPP_CODE)
    sftp.close()
    print("m3v_move.cpp written")

    # Compile
    print("compiling...")
    _, o, e = c.exec_command(COMPILE_CMD, timeout=30)
    o.channel.recv_exit_status()
    out = o.read().decode('utf-8','replace').rstrip()
    err = e.read().decode('utf-8','replace').rstrip()
    if out: print(out)
    if err: print("[stderr]", err)

    # Check binary
    _, o2, _ = c.exec_command('ls -la /home/unitree/m3v_move 2>&1', timeout=5)
    o2.channel.recv_exit_status()
    print(o2.read().decode().rstrip())
    c.close()

if __name__ == "__main__":
    main()
