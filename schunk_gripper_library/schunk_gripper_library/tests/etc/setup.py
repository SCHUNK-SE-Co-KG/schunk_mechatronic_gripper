#!/usr/bin/env python3
import subprocess
import time
import os
import signal
import sys

# TTY_BUS socket path:
TTY_BUS_SOCKET = "/tmp/ttypts2mux"


def cleanup(tty_bus_proc, pts_dev, fake_devs):
    # terminate tty_bus if it's still running
    if tty_bus_proc and tty_bus_proc.poll() is None:
        os.kill(tty_bus_proc.pid, signal.SIGTERM)

    # remove fake devices
    for fake_dev in fake_devs:
        if os.path.exists(fake_dev):
            try:
                os.unlink(fake_dev)
            except PermissionError:
                subprocess.run(["sudo", "rm", "-f", fake_dev])

    # remove the symlink
    if os.path.islink("/dev/ttyUSB0"):
        os.unlink("/dev/ttyUSB0")

    # remove socket file
    if os.path.exists(TTY_BUS_SOCKET):
        os.unlink(TTY_BUS_SOCKET)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <max_fake_devices>")
        print("Example: {sys.argv[0]} 10")
        sys.exit(1)

    try:
        max_fake_devices = int(sys.argv[1])
    except ValueError:
        print("Error: <max_fake_devices> must be an integer")
        sys.exit(1)

    tty_bus_proc = None
    pts_dev = None
    fake_devs = []
    pty_proc = None

    try:
        # 1) Create a PTY pair first to get pts device
        print("1) Creating PTY pair...")
        pty_proc = subprocess.Popen(
            [
                "socat",
                "pty,rawer,echo=0,link=/tmp/pts_master",
                "pty,rawer,echo=0,link=/tmp/pts_slave",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(1)

        # Find the actual pts device
        if os.path.exists("/tmp/pts_slave"):
            pts_dev = os.readlink("/tmp/pts_slave")
            print(f"   → Using PTS device: {pts_dev}")
        else:
            raise RuntimeError("Failed to create PTS device")

        # 2) Start tty_bus daemon
        print("2) Starting tty_bus daemon...")
        tty_bus_proc = subprocess.Popen(
            ["tty_bus", "-d", "-s", TTY_BUS_SOCKET],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(1)  # Give tty_bus time to start

        # 3) Attach the pts device to tty_bus
        print(f"3) Attaching {pts_dev} to tty_bus...")
        subprocess.run(["tty_attach", "-d", "-s", TTY_BUS_SOCKET, pts_dev], check=True)

        # 4) Create fake virtual serial ports for potential BKS simulations
        print("4) Creating fake virtual serial ports...")
        for i in range(max_fake_devices):
            fake_dev = f"/dev/ttypts2fake{i}"
            fake_devs.append(fake_dev)
            print(f"   → Creating {fake_dev}")
            subprocess.run(
                ["sudo", "tty_fake", "-d", "-s", TTY_BUS_SOCKET, fake_dev], check=True
            )

        # 5) Link /dev/ttyUSB0 → the master pts device
        print("6) Linking /tmp/pts_master → /dev/ttyUSB0")
        if os.path.islink("/dev/ttyUSB0"):
            os.unlink("/dev/ttyUSB0")
        subprocess.run(
            ["sudo", "ln", "-s", "/tmp/pts_master", "/dev/ttyUSB0"], check=True
        )
        subprocess.run(["sudo", "chmod", "666", "/dev/ttyUSB0"], check=True)

        print("\nInfrastructure setup complete.")
        print(
            f"   • Created {max_fake_devices} fake devices: "
            f"    /dev/ttypts2fake0 to /dev/ttypts2fake{max_fake_devices - 1}"
        )
        print("   • /dev/ttyUSB0 is ready for your pymodbus tests")
        print("   • Press Ctrl-C here to tear everything down and remove devices.\n")

        # Keep the script running
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nReceived Ctrl-C, shutting down...")

    except Exception as e:
        print(f"\n✖ Error: {e}")
        raise
    finally:
        print("\nCleaning up…")
        cleanup(tty_bus_proc, pts_dev, fake_devs)
        # Also cleanup the socat process if it exists
        if pty_proc:
            pty_proc.terminate()


if __name__ == "__main__":
    main()
