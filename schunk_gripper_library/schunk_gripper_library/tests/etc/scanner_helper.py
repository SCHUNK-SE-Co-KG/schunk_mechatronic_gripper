from ...utility import Scanner
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian

import subprocess
import tempfile
import time
import os
from typing import Dict

# Path to your BKS_Simulation_Windows directory:
SIM_PATH = os.getenv("BKS_SIMULATION_PATH")


class ScannerTestSetup(Scanner):
    """
    A test setup class that inherits from Scanner.
    This class can be used to set up the scanner for testing purposes.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Additional setup can be done here if needed

    def change_serial_num(self, dev_id: int, serial_number: str) -> bool:
        """
        Change the serial number of the gripper by writing to
        the serial_no_num register.
        Args:
            dev_id: Device ID of the gripper
            serial_number: Serial number as hex string (e.g., "12345678")
        Note: This may not work if the register is read-only.
        """

        if len(serial_number) != 8:
            return False

        if not all(c in "0123456789abcdefABCDEF" for c in serial_number):
            return False

        if not (0 <= dev_id <= 247):
            return False

        try:
            print(f"Changing serial number for device {dev_id} to {serial_number}")
            # Convert hex string to integer

            serial_int = int(serial_number, 16)

            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)

            # Add 32-bit serial number (UINT32 requires 2 registers)
            builder.add_32bit_uint(serial_int)

            payload = builder.to_registers()
            register_address = 0x1020 - 1

            response = self.client.write_registers(
                register_address, payload, slave=dev_id
            )
            print(f"Successfully changed gripper serial number to {serial_number}")
            print(f"Response: {response}")

            return True

        except Exception as e:
            print(f"Unexpected error changing serial number: {e}")

            return False


class BKSLauncher:
    def __init__(self):
        self.simulations: Dict[int, dict] = {}
        self.scanner = ScannerTestSetup()

    def start_bks_simulation(
        self, sim_id: int, serial_num: str, device_index: int = 0
    ) -> bool:
        """
        Start a BKS simulation

        Args:
            sim_id: Unique id for this simulation, will also be used as the gripper ID
            device_index: Index of the fake device to use (0 = /dev/ttypts2fake0, etc.)
        Returns:
            True if started successfully, False otherwise
        """
        if sim_id in self.simulations:
            print(f"Simulation '{sim_id}' is already running")
            return False

        if serial_num is None or len(serial_num) != 8:
            return False

        if serial_num in [sim["serial_num"] for sim in self.simulations.values()]:
            return False

        fake_dev = f"/dev/ttypts2fake{device_index}"
        if not os.path.exists(fake_dev):
            print(
                f"Error: Device {fake_dev} does not exist. "
                f"    Make sure infrastructure is running."
            )
            return False

        if not SIM_PATH:
            return False

        sim_temp_dir = tempfile.mkdtemp(prefix=f"bks_sim_{sim_id}_")
        log_file = os.path.join(sim_temp_dir, f"bks_sim_{sim_id}.log")
        print(f"Starting BKS simulation '{sim_id}' on {fake_dev} in {sim_temp_dir}...")

        try:
            # Copy flash device directory
            flash_device_src = os.path.join(
                SIM_PATH, "flash_devices", "EGK_40_M_B_flash_device"
            )
            flash_device_dest = os.path.join(sim_temp_dir, "EGK_40_M_B_flash_device")

            if not os.path.exists(flash_device_src) or not os.access(
                flash_device_src, os.R_OK
            ):
                subprocess.run(["rm", "-rf", sim_temp_dir], check=False)
                return False

            subprocess.run(
                ["cp", "-r", flash_device_src, flash_device_dest], check=True
            )

            # Copy BKS_SIMULATION executable
            bks_simulation_src = os.path.join(SIM_PATH, "BKS_Simulation_Windows")
            bks_simulation_dest = os.path.join(sim_temp_dir, "BKS_Simulation_Windows")

            if not os.path.exists(bks_simulation_src) or not os.access(
                bks_simulation_src, os.R_OK
            ):
                subprocess.run(["rm", "-rf", sim_temp_dir], check=False)
                return False

            subprocess.run(
                ["cp", "-r", bks_simulation_src, bks_simulation_dest], check=True
            )

        except subprocess.CalledProcessError:
            subprocess.run(["rm", "-rf", sim_temp_dir], check=False)
            return False

        # Use the copied BKS_Simulation_Windows executable
        sim_cmd = f'"{sim_temp_dir}/BKS_Simulation_Windows" \
            EGK_40_M_B -p "{fake_dev}" -c none'

        # Redirect output to log file
        full_cmd = (
            f'cd "{sim_temp_dir}" && {sim_cmd} 2>&1 | tee "{log_file}"; exec bash'
        )
        proc = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", full_cmd])

        self.simulations[sim_id] = {
            "process": proc,
            "device": fake_dev,
            "temp_dir": sim_temp_dir,
            "device_index": device_index,
            "log_file": log_file,
            "serial_num": serial_num,
        }

        # Give it a moment to start
        time.sleep(1)

        if self.scanner.change_gripper_id(old_id=12, new_id=sim_id):
            time.sleep(1)
            if self.scanner.change_serial_num(dev_id=sim_id, serial_number=serial_num):
                time.sleep(1)
                return True

        return False

    def stop_bks_simulation(self, sim_id: int) -> bool:
        """
        Stop a BKS simulation

        Args:
            sim_id: Identifier of the simulation to stop

        Returns:
            True if stopped successfully, False if not found
        """
        if sim_id not in self.simulations:
            print(f"Simulation '{sim_id}' not found")
            return False

        sim_info = self.simulations[sim_id]
        proc = sim_info["process"]
        temp_dir = sim_info["temp_dir"]
        fake_dev = sim_info["device"]

        print(f"Stopping BKS simulation '{sim_id}'...")

        # Kill all BKS_Simulation_Windows processes using the specific device
        try:
            # Find and kill BKS processes
            subprocess.run(
                ["pkill", "-f", f"BKS_Simulation_Windows.*{fake_dev}"],
                capture_output=True,
            )

            # Also kill the terminal if it's still running
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

        except Exception as e:
            print(f"Error stopping simulation: {e}")

        # Clean up temp directory
        subprocess.run(["rm", "-rf", temp_dir], check=False)

        del self.simulations[sim_id]
        return True

    def stop_all(self):
        """Stop all running simulations"""
        print("Stopping all simulations...")

        # Stop all simulations
        for sim_id in list(self.simulations.keys()):
            self.stop_bks_simulation(sim_id)

    def get_last_log(self, sim_id: int) -> str | None:
        if sim_id not in self.simulations:
            print(f"Simulation '{sim_id}' not found")
            return None

        log_file = self.simulations[sim_id].get("log_file")
        if not log_file or not os.path.exists(log_file):
            print(f"Log file not found for simulation '{sim_id}'")
            return None

        try:
            result = subprocess.run(
                ["tail", "-n", str(1), log_file], capture_output=True, text=True
            )
            return result.stdout

        except Exception as e:
            print(f"Error reading log file: {e}")
            return None


# Convenience functions for direct use
_launcher = BKSLauncher()


def start_bks_simulation(sim_id: int, serial_num: str, device_index: int = 0) -> bool:
    """Start a BKS simulation"""
    return _launcher.start_bks_simulation(
        sim_id,
        serial_num,
        device_index,
    )


def stop_bks_simulation(sim_id: int) -> bool:
    """Stop a BKS simulation"""
    return _launcher.stop_bks_simulation(sim_id)


def stop_all():
    """Stop all running simulations"""
    _launcher.stop_all()


def get_last_log(sim_id: int) -> str | None:
    """Read the last log for a simulation"""
    return _launcher.get_last_log(sim_id)
