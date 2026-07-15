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
#include <unitree/robot/a2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
using namespace std;
int main(int argc, char** argv) {
    if (argc < 2) { cerr << "Usage: " << argv[0] << " networkInterface" << endl; return 1; }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::a2::SportClient sport_client;
    sport_client.SetTimeout(3.0f);
    sport_client.Init();
    cerr << "m3v_move ready" << endl;
    // recovery_stand to get dog standing on startup
    sport_client.RecoveryStand();
    usleep(2000000);
    // Loop: read "vx vy yaw" from stdin, send Move
    double vx, vy, yaw;
    string line;
    while (getline(cin, line)) {
        if (sscanf(line.c_str(), "%lf %lf %lf", &vx, &vy, &yaw) == 3) {
            sport_client.Move(vx, vy, yaw);
        }
    }
    // stdin closed: lie down
    sport_client.StandDown();
    return 0;
}
"""

COMPILE_CMD = (
    'cd /home/unitree/unitree_sdk2-main && '
    'g++ -std=c++14 -o /home/unitree/m3v_move /home/unitree/m3v_move.cpp '
    '-I include -L lib -lunitree_robot -lunitree_idl -lunitree_common '
    '-l cyclonedds -Wl,-rpath,lib 2>&1'
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
