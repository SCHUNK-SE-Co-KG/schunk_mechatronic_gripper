from schunk_gripper_dummy.dummy import Dummy
from schunk_gripper_dummy.main import create_webserver
from fastapi.testclient import TestClient
import time


def test_dummy_responds_correctly_to_data_instance_requests():
    dummy = Dummy()
    inst = "0x0040"
    query = {"inst": inst, "count": 1}
    expected = dummy.data[inst]
    assert dummy.get_data(query) == expected


def test_dummy_survives_invalid_data_requests():
    dummy = Dummy()
    query = {"offset": 1000, "count": "2"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": 100, "count": "90"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": 1000, "count": "-1"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"offset": -1, "count": "1000"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"inst": "0x0040", "count": "0"}
    expected = []
    assert dummy.get_data(query) == expected
    query = {"inst": "0x0040", "count": "2"}
    expected = []
    assert dummy.get_data(query) == expected


def test_dummy_stores_update_requests():
    dummy = Dummy()

    # Using the plc command variable
    msg = "00112233445566778899AABBCCDDEEFF"
    data = {"inst": dummy.plc_output, "value": msg}
    dummy.update(data)
    assert dummy.data[dummy.plc_output] == [msg]

    # Using general variables
    msg = "AABBCCDD"
    inst = "0x0238"
    data = {"inst": inst, "value": msg}
    dummy.update(data)
    assert dummy.data[inst] == [msg]


def test_dummy_rejects_invalid_update_requests():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    valid_data = "AABBCCDD"
    valid_inst = "0x0238"
    data = {"inst": valid_inst, "value": valid_data}
    assert client.post("/adi/update.json", data=data).json() == {"result": 0}

    invalid_data = "hello:)"
    valid_inst = "0x0238"
    data = {"inst": valid_inst, "value": invalid_data}
    assert client.post("/adi/update.json", data=data).json() == {"result": 1}

    valid_data = "AABBCCDD"
    invalid_inst = "0x9999"
    data = {"inst": invalid_inst, "value": valid_data}
    assert client.post("/adi/update.json", data=data).json() == {"result": 1}

    dummy.stop()


def test_dummy_resets_success_status_bits_with_new_update_requests():
    dummy = Dummy()
    dummy.set_status_bit(bit=13, value=True)  # position reached
    dummy.set_status_bit(bit=4, value=True)  # command successful
    dummy.set_status_bit(bit=12, value=True)  # workpiece gripped
    empty_command = "01" + "".zfill(30)  # only fast stop active
    data = {"inst": dummy.plc_output, "value": empty_command}
    dummy.update(data)
    assert dummy.get_status_bit(bit=13) == 0
    assert dummy.get_status_bit(bit=4) == 0
    assert dummy.get_status_bit(bit=12) == 0


def test_dummy_can_mimic_connection_loss():
    dummy = Dummy()
    dummy.start()
    server = create_webserver(dummy)
    client = TestClient(server)

    # Temporarily shutdown the data route
    for _ in range(3):
        until_back = 0.5
        events = {"lose_connection_sec": until_back}
        assert client.post("/adi/events.json", json=events).is_success
        assert not client.get("/adi/data.json").is_success

        time.sleep(until_back)
        assert client.get("/adi/data.json").is_success

    dummy.stop()
