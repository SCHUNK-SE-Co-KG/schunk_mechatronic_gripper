# Using an Ethernet-based gripper with Windows WSL2 + Ubuntu

## Discovering grippers
The grippers normally ship with _DHCP_ enabled and should work out-of-the box.
You can use the driver's _scanning_ functionality to find the IP addresses of connected devices.
Since this scanning works with _broadcasts_ you will probably need to enable port forwarding. We assume that you are working with a ROS2-compatible `Ubuntu` in `WSL2`.
Here are the steps:

1. Inside an `Ubuntu` terminal, get your current IP with
    ```bash
    hostname -I # Pick the one for your desired network
    ```
    We call that IP `<WSL2-IP>` in the steps below
2. Inside a `Windows` terminal, setup port forwarding with
    ```bash
    netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=3250 connectaddress=<WSL-IP> connectport=3250
    ```
    The port `3250` is important here. SCHUNK grippers haven an _HMS chip_ inside that offers a special discovery mechanism over that port using UDP broadcasts.

    Here are some other optional commands:
    - Inspect existing forwarding rules with
      ```bash
      netsh interface portproxy show all
      ```
    - Remove your port forwarding rule with
      ```bash
      netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=3250
      ```
3. In `Windows`, add an _inbound_ rule for the port `3250`
4. Restart `WSL2` for the changes to take effect
