import asyncio, math, socket, uuid, random
import numpy as np
import quaternion
from bitstring import BitArray, pack
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from functools import partial

MAX_DEGREES = 180

application_name = "Python3 SlimeVR Example"
server = ("127.0.0.1", 6969)
connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

home_pressed = False
app_pressed = False
vol_up_pressed = False
vol_down_pressed = False

devices = [
    "54:AB:3A:F3:0C:DA",
    "A8:1E:84:ED:6D:07",
    "A8:1E:84:91:D6:44"
]

calibration = {}

heartbeat_packet = {
    "packet_type": 0
}

handshake_packet = {
    "packet_type": 3,
    "board_type": ("uint:32", 0),
    "imu_type": ("uint:32", 0),
    "mcu_type": ("uint:32", 0),
    "imu_info": ("3*uint:32", 0, 0, 0),
    "firmware_build": ("uint:32", 8),
    "firmware_name_length": len(application_name),
    "firmware_name": application_name.encode(),
    "mac": hex(uuid.getnode())
}

sensor_packet = {
    "packet_type": 15,
    "sensor_id": 0,
    "sensor_state": 1,
    "sensor_type": 0
}

rotation_packet = {
    "packet_type": 17,
    "sensor_id": 0,
    "data_type": 1,
    "quaternion": ("4*float:32",),
    "accuracy_info": 0
}

# Logic

def set_id(packet, id):
    packet["sensor_id"] = id
    return packet

def get_random_rotation():
    return random.uniform(-MAX_DEGREES, MAX_DEGREES)

def randomize_rotation(rotation_dictionary):
    rotation_dictionary["quaternion"] = (rotation_dictionary["quaternion"][0], get_random_rotation(), get_random_rotation(), get_random_rotation(), random.random())
    return rotation_dictionary

def set_quaternion(rotation_dictionary, quaternion):
    rotation_dictionary["quaternion"] = (rotation_dictionary["quaternion"][0], *quaternion)
    return rotation_dictionary

def packet_to_bitarray(packet):
    bitarray = BitArray()
    for item in enumerate(packet):
        # Index Checks
        if item[0] == 0:
            bitarray.append(f"uint:32={packet[item[1]]}, uint:64=0")
            continue
        # Type Checks
        if isinstance(packet[item[1]], int):
            bitarray.append(f"uint:8={packet[item[1]]}")
        elif isinstance(packet[item[1]], tuple):
            bitarray.append(pack(*packet[item[1]]))
        else:
            bitarray.append(packet[item[1]])
    return bitarray

# Bluetooth

CHARACTERISTIC_UUID = "00000001-1000-1000-8000-00805f9b34fb"

def to_signed(val: int, bit_width=11) -> int:
    val_12_bit = val & (2**(bit_width+1)-1)
    val_sign = val_12_bit & (1 << bit_width)
    val_abs = val_12_bit & ~(1 << bit_width)
    val_signed = (-(2**bit_width))+val_abs if val_sign else val_abs
    return val_signed

# === MAIN ===

def notification_handler(sensor_id, sender, data):
    global home_pressed, app_pressed, vol_up_pressed, vol_down_pressed, calibration

    ddX = (data[1] & 0x03) << 11 | (data[2] & 0xFF) << 3 | (data[3] & 0x80) >> 5
    ddX = to_signed(ddX)

    ddY = (data[3] & 0x1F) << 8 | (data[4] & 0xFF)
    ddY = to_signed(ddY)

    ddZ = (data[5] & 0xFF) << 5 | (data[6] & 0xF8) >> 3
    ddZ = to_signed(ddZ)

    axis = [-ddX, ddY, ddZ]
    vector = np.array([(2 * math.pi / 4095.0) * x for x in axis])
    # rotation_convert = list(Rotation.from_quat([*vector, w]).as_quat())
    # rotation_convert.insert(0, rotation_convert.pop())
    # rotation = Quaternion(rotation_convert)
    rotation = quaternion.from_rotation_vector(vector)

    # vol_up = data[18] & 0x10
    # vol_down = data[18] & 0x8

    # if vol_up != 0 and not vol_up_pressed:
    #     vol_up_pressed = True
    #     calibration[devices[sensor_id]] = rotation
    #     print("Calibration applied: ", calibration[devices[sensor_id]])
    # elif vol_up == 0 and vol_up_pressed:
    #     vol_up_pressed = False

    # if vol_down != 0 and not vol_down_pressed:
    #     vol_down_pressed = True
    #     q = quaternion.as_float_array(rotation)
    #     calibration[devices[sensor_id]] = np.quaternion(q[0], -q[1], -q[2], -q[3])
    #     print("Calibration applied: ", calibration[devices[sensor_id]])
    # elif vol_down == 0 and vol_down_pressed:
    #     vol_down_pressed = False

    # if (data[18] & 0x4) != 0 and not app_pressed and len(calibration) > 0:
    #     app_pressed = True
    #     calibration.pop(devices[sensor_id])
    #     print("Calibration reset")
    # elif (data[18] & 0x4) == 0 and app_pressed:
    #     app_pressed = False

    # if (len(calibration) > 0):
    #     rotation = rotation * calibration[devices[sensor_id]]

    r = quaternion.as_float_array(rotation) # w x y z
    quat = (r[1], r[3], r[2], r[0]) # x z y w

    # print("X: " + format(quat[0], ' .5f'), "Y: " + format(quat[1], ' .5f'), "Z: " + format(quat[2], ' .5f'), "W: " + format(quat[3], ' .5f'), " | ","X: " + format(vector[0], ' .5f'), "Y: " + format(vector[1], ' .5f'), "Z: " + format(vector[2], ' .5f'), " | ", "X: " + format(axis[0], ' 5'), "Y: " + format(axis[1], ' 5'), "Z: " + format(axis[2], ' 5'), " | ", "Calibrations: " + str(len(calibration)))

    local_rotation_packet = rotation_packet
    local_rotation_packet = set_id(local_rotation_packet, sensor_id)
    local_rotation_packet = set_quaternion(local_rotation_packet, quat)
    connection.sendto(packet_to_bitarray(heartbeat_packet).bytes, server)
    connection.sendto(packet_to_bitarray(local_rotation_packet).bytes, server)

# ============

async def connect_to_device(address):
    print("Connecting to", address)
    try:
        async with BleakClient(address, timeout=15.0) as client:
            print(f"Connected: {client.address}")
            connection.sendto(packet_to_bitarray(handshake_packet).bytes, server)
            sensor_id = devices.index(client.address)
            local_sensor_packet = sensor_packet
            local_sensor_packet = set_id(local_sensor_packet, sensor_id)
            connection.sendto(packet_to_bitarray(local_sensor_packet).bytes, server)
            try:
                while True:
                    await client.start_notify(CHARACTERISTIC_UUID, partial(notification_handler, sensor_id))
            except:
                print("Connection", address, "terminated, reconnecting...")
                await client.stop_notify(CHARACTERISTIC_UUID)
                await client.disconnect()
                await connect_to_device(address)
    except:
        print("Couldn't connect to", address, "reconnecting...")
        await(connect_to_device(address))

async def main(addresses):
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "Daydream controller":
            print("[BLE Discovery] {}".format(d.address))
            asyncio.create_task(connect_to_device(d))
            asyncio.sleep(1)
    # return await asyncio.gather(*(connect_to_device(device) for device in addresses))

# async def main(address, char_uuid):
#     async with BleakClient(address) as client:
#         print(f"Connected: {client.is_connected}")
#         connection.sendto(packet_to_bitarray(handshake_packet).bytes, server)
#         connection.sendto(packet_to_bitarray(sensor_packet).bytes, server)
#         while client.is_connected:
#             await client.start_notify(char_uuid, notification_handler)
#         print(f"Connected: {client.is_connected}")


# if __name__ == "__main__":
#     asyncio.run(
#         main(
#             sys.argv[1] if len(sys.argv) > 1 else ADDRESS,
#             sys.argv[2] if len(sys.argv) > 2 else CHARACTERISTIC_UUID,
#         )
#     )

if __name__ == "__main__":
    asyncio.run(main(devices))