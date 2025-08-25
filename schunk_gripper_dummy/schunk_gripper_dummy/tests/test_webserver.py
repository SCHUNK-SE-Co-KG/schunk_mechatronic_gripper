from schunk_gripper_dummy.main import create_webserver
from schunk_gripper_dummy.dummy import Dummy
from fastapi.testclient import TestClient


def test_data_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    response = client.get("/adi/data.json")
    assert response.is_success

    dummy.stop()


def test_update_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    data = {"inst": "0", "value": "0"}
    response = client.post("/adi/update.json", data=data)
    assert response.is_success

    dummy.stop()


def test_events_route_is_available():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    events = {}
    response = client.post("/adi/events.json", json=events)
    assert response.is_success

    dummy.stop()
