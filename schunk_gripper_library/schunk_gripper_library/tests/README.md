# Run tests locally

We use `socat` to set-up pseudo terminals for a connection between the driver library and a minimalist Modbus server.
Install that with
```bash
sudo apt-get install socat
```

Additionally, inside the dummy's package, install test-related dependencies with

```bash
pip install --user pytest httpx coverage
```

You can run the tests either directly in this folder with `pytest .` or with more output through

```bash
coverage run -m pytest .
coverage report  # for console output
coverage html    # for web-based output
```

## Testing for thread safety
This is an interesting topic on its own, see e.g.
- [py-free-threading](https://py-free-threading.github.io/debugging/)
- [the GIL](https://opensource.com/article/17/4/grok-gil)
- [changes in python 3.13](https://docs.python.org/3/howto/free-threading-python.html)

```bash
pip install pytest-repeat
PYTHON_GIL=0 pytest --count=10 test_threading.py
```

## Modbus unit testing

Special test fixtures will create two pseudo-terminals that are connected with each other,
such as `/dev/pts/14` and `/dev/pts/15`.
The tests then use this pseudo serial connection to setup a Modbus server/client communication like this:

Modbus Client <-> /dev/pts/15 <-> /dev/pts/14 <-> Modbus Server

## Setup for Scanner Tests

Before running scanner tests that use the BKS simulation, you need to:

### 1. Set the BKS Simulation Path
Set the environment variable to point to your BKS simulation directory:
```bash
export BKS_SIMULATION_PATH="/path/to/BKS_2024-03-26_Heartbeat_Kes_Debug_Linux"
```

### 2. Start the Test Infrastructure
Run the setup script with sudo to create virtual serial devices:
```bash
sudo python3 etc/setup.py <max_sims>
```
Where `<max_sims>` is the maximum number of simultaneous BKS simulations you want to support (e.g., `4` for the current tests).

**Note:** Keep this setup script running in the background during your tests. Press `Ctrl-C` to tear down the infrastructure when done.

### 3. Run Your Tests
With the infrastructure running, you can now execute scanner tests that require BKS simulations.
