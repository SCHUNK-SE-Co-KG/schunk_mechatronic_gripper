#!/usr/bin/env python3
import requests  # type: ignore
import json
import argparse
from schunk_gripper_library.driver import Driver

parser = argparse.ArgumentParser()
parser.add_argument(
    "ip", help="The IP address of the gripper on the network.", type=str
)
args = parser.parse_args()

parameter_codes = "./gripper_parameter_codes.json"

driver = Driver()
driver.connect(host=args.ip)
outfile = f"./grippers/{driver.gripper_type}.json"
driver.disconnect()


def read_parameter_codes(filepath: str) -> list[str]:
    with open(filepath, "r") as f:
        data = json.load(f)
    return data.keys()


def main():

    data = read_parameter_codes(parameter_codes)

    values = {}
    for code in data:
        print(f"reading code {code}")
        try:
            response = requests.get(
                f"http://{args.ip}/adi/data.json?inst={int(code, 16)}&count=1"
            )
            values[code] = response.json()
        except Exception as e:
            print(f"error reading code {code}: {e}")

    with open(outfile, "w") as f:
        json.dump(values, f, indent=4)


if __name__ == "__main__":
    main()
