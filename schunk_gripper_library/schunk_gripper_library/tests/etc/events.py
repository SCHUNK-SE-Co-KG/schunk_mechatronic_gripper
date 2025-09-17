import httpx
from httpx import ConnectError, ConnectTimeout
import subprocess
from threading import Timer


def lose_tcp_connection(
    host: str = "0.0.0.0", port: int = 8000, duration_sec: float = 1.0
) -> bool:
    events = {"lose_connection_sec": duration_sec}
    try:
        response = httpx.post(f"http://{host}:{port}/adi/events.json", json=events)
    except (ConnectError, ConnectTimeout):
        return False
    return response.is_success


def lose_modbus_connection(duration_sec: float = 1.0) -> bool:
    try:
        result = subprocess.run(
            ["pgrep", "-fa", "BKS_Sim"], capture_output=True, text=True, check=True
        )
        line = result.stdout.strip()
        pid = int(line.split()[0])

        subprocess.run(["kill", "-19", str(pid)], check=True)  # stop

        def reset() -> None:
            subprocess.run(["kill", "-18", str(pid)], check=True)  # continue

        Timer(interval=duration_sec, function=reset).start()

    except subprocess.CalledProcessError:
        return False

    return True
