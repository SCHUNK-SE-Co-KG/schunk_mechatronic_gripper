import httpx
from httpx import ConnectError, ConnectTimeout


def lose_connection(
    host: str = "0.0.0.0", port: int = 8000, duration_sec: float = 1.0
) -> bool:
    events = {"lose_connection_sec": duration_sec}
    try:
        response = httpx.post(f"http://{host}:{port}/adi/events.json", json=events)
    except (ConnectError, ConnectTimeout):
        return False
    return response.is_success
