from schunk_gripper_library.utility import Scanner
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
        self.clear_buffer()
        try:
            print(f"Changing serial number for device {dev_id} to {serial_number}")
            # Convert hex string to integer
            if len(serial_number) != 8:
                raise ValueError(
                    f"Serial number must be 8 hex characters, got: {serial_number}"
                )

            try:
                serial_int = int(serial_number, 16)
            except ValueError:
                raise ValueError(
                    f"Serial number must be valid hex string, got: {serial_number}"
                )

            builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)

            # Add 32-bit serial number (UINT32 requires 2 registers)
            builder.add_32bit_uint(serial_int)

            payload = builder.to_registers()
            register_address = 0x1020 - 1

            response = self.client.write_registers(
                register_address, payload, slave=dev_id, no_response_expected=True
            )
            print(f"Successfully changed gripper serial number to {serial_number}")
            print(response)

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

        fake_dev = f"/dev/ttypts2fake{device_index}"
        if not os.path.exists(fake_dev):
            print(
                f"Error: Device {fake_dev} does not exist. "
                f"    Make sure infrastructure is running."
            )
            return False

        sim_temp_dir = tempfile.mkdtemp(prefix=f"bks_sim_{sim_id}_")
        print(f"Starting BKS simulation '{sim_id}' on {fake_dev} in {sim_temp_dir}...")

        sim_cmd = (
            f'"{SIM_PATH}/BKS_Simulation_Windows" EGK_40_M_B -p "{fake_dev}" -c none'
        )

        full_cmd = f'cd "{sim_temp_dir}" && {sim_cmd}; exec bash'
        proc = subprocess.Popen(["gnome-terminal", "--", "bash", "-c", full_cmd])

        self.simulations[sim_id] = {
            "process": proc,
            "device": fake_dev,
            "temp_dir": sim_temp_dir,
            "device_index": device_index,
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

    def list_running(self):
        """List all running simulations"""
        print("Running BKS Simulations:")
        if not self.simulations:
            print("  None")
        else:
            for sim_id, info in self.simulations.items():
                print(f"  {sim_id}: {info['device']} (temp: {info['temp_dir']})")

    def stop_all(self):
        """Stop all running simulations"""
        print("Stopping all simulations...")

        # Stop all simulations
        for sim_id in list(self.simulations.keys()):
            self.stop_bks_simulation(sim_id)


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


def list_running():
    """List all running simulations"""
    _launcher.list_running()


def stop_all():
    """Stop all running simulations"""
    _launcher.stop_all()
