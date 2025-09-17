# Reading system parameters from a gripper
The system parameters are mentioned in the gripper's datasheet [Commissioning Instructions, Firmware 5.2 EGU with EtherNet/IP interface](https://stb.cloud.schunk.com/media/IM0046706.PDF).

## Enums
The easiest approach is reading enums from a web browser.
Enums need to be accessed via the _DEC_ identifier.
For instance, reading available error codes for the enum `HEX=0x0118` (`DEC=280`) would be
```browser
http://<gripper-ip>/adi/enum.json?inst=280
```

## Metadata

```bash
http://<gripper-ip>/adi/metadata.json?offset=0&count=300
```

## Data
Use this script that reads the data directly from the gripper:
```bash
./read_gripper_parameters.py <gripper-ip>
```
