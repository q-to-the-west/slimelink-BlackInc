import asyncio, math, socket, uuid, random, sys
import numpy as np
import quaternion
from bitstring import BitArray, pack
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService
from bleak.backends.characteristic import BleakGATTCharacteristic
from functools import partial

# Globals

application_name = "Python3 SlimeVR Example"
slimevr_server_address = ('127.0.0.1', 6969)
connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
xTouch:float = 0.0
yTouch:float = 0.0
devices = []

# Constants

# Bluetooth Constants
DAYDREAM_NAME = "Daydream controller"
SERVICE_UUID = uuid.UUID('0000fe55-0000-1000-8000-00805f9b34fb')
CHARACTERISTIC_UUID = uuid.UUID('00000001-1000-1000-8000-00805f9b34fb')
NOTIFICATION_UUID = uuid.UUID('00002902-0000-1000-8000-00805f9b34fb')

# Data Constants
CLICK_BUTTON = 0x1
HOME_BUTTON = 0x2
APP_BUTTON = 0x4
VOLUME_DOWN_BUTTON = 0x8
VOLUME_UP_BUTTON = 0x10
MAX_DEGREES = 180

# Packets

# rotation_packet = {
#     "packet_type": 1,
#     "quaternion": ("4*float:32",)
# }

heartbeat_packet = {
    "packet_type": 0
}

handshake_packet = {
    "packet_type": 3,
    "board_type": ("int:32", 0),
    "imu_type": ("int:32", 0),
    "mcu_type": ("int:32", 0),
    "imu_info": ("3*int:32", 0, 0, 0),
    "firmware_build": ("int:32", 8),
    # "firmware_name_length": len(application_name),
    "firmware_name": application_name.encode(),
    "mac": hex(uuid.getnode())
}

acceleration_packet = {
    "packet_type": 3,
    "vector": ("3*float:32",),
    "sensor_id": 0 # Is some other sensor_id format is used?
#     See:
#     https://github.com/carl-anders/slimevr-wrangler/blob/main/protocol/src/test_deku.rs and
#     https://github.com/SlimeVR/SlimeVR-Rust/blob/63ee7f7d1d77334d115c050df469c720b5a20736/firmware/src/main.rs#L84
#     for reference.
}

ping_packet = {
    "packet_type": 10,
    "sensor_id": 0
}

sensor_info_packet = {
    "packet_type": 15,
    "sensor_id": 0,
    "sensor_state": 1,
    "sensor_type": 0
}

rotation_data_packet = {
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

def set_vector(acceleration_dictionary, vector):
    acceleration_dictionary["vector"] = (acceleration_dictionary["vector"][0], *vector)
    return acceleration_dictionary

def set_quaternion(rotation_dictionary, quaternion):
    rotation_dictionary["quaternion"] = (rotation_dictionary["quaternion"][0], *quaternion)
    return rotation_dictionary

def packet_to_bitarray(packet):
    bitarray = BitArray()
    for index, item in enumerate(packet):
        # Index Checks
        if index == 0:
            bitarray.append(f"uint:32={packet[item]}, uint:64=0")
            continue
        # Type Checks
        if isinstance(packet[item], int):
            bitarray.append(f"uint:8={packet[item]}")
        elif isinstance(packet[item], tuple):
            bitarray.append(pack(*packet[item]))
        else:
            bitarray.append(packet[item])
    return bitarray

def to_signed(val: int, bit_width=11) -> int:
    val_12_bit = val & (2**(bit_width+1)-1)
    val_sign = val_12_bit & (1 << bit_width)
    val_abs = val_12_bit & ~(1 << bit_width)
    val_signed = (-(2**bit_width))+val_abs if val_sign else val_abs
    return val_signed

def right_shift_unsigned(val, n):
    return (val % 0x100000000) >> n

# === MAIN ===

def notification_handler(sensor_id, sender: BleakGATTCharacteristic, data: bytearray):
    xOri = (data[1] & 0x03) << 11 | (data[2] & 0xFF) << 3 | (data[3] & 0xE0) >> 5
    xOri = to_signed(xOri)
    # xOri = (xOri << 19) >> 19
    xOri *= (2 * math.pi / 4095.0)

    yOri = (data[3] & 0x1F) << 8 | (data[4] & 0xFF)
    yOri = to_signed(yOri)
    # yOri = (yOri << 19) >> 19
    yOri *= (2 * math.pi / 4095.0)

    zOri = (data[5] & 0xFF) << 5 | (data[6] & 0xF8) >> 3
    zOri = to_signed(zOri)
    # zOri = (zOri << 19) >> 19
    zOri *= (2 * math.pi / 4095.0)

    orientation_vector = [xOri, yOri, zOri]

    orientation = quaternion.from_rotation_vector(orientation_vector)

    r = quaternion.as_float_array(orientation) # w x y z
    rotation_quaternion = (r[1], r[3], r[2], r[0]) # x z y w

    # Printing orientation items
    # (for testing purposes).
    # for number in r:
    #     print(format(number, '.2f'), end="\t")
    # print("")

    xAcc = (data[6] & 0x07) << 10 | (data[7] & 0xFF) << 2 | (data[8] & 0xC0) >> 6
    xAcc = to_signed(xAcc)
    # xAcc = (xAcc << 19) >> 19
    xAcc *= ((8 * 9.8) / 4095.0)

    yAcc = (data[8] & 0x3F) << 7 | right_shift_unsigned((data[9] & 0xFE), 1)
    yAcc = to_signed(yAcc)
    # yAcc = (yAcc << 19) >> 19
    yAcc *= (8 * 9.8 / 4095.0)

    zAcc = (data[9] & 0x01) << 12 | (data[10] & 0xFF) << 4 | (data[11] & 0xF0) >> 4
    zAcc = to_signed(zAcc)
    # zAcc = (zAcc << 19) >> 19
    zAcc *= (8 * 9.8 / 4095.0)

    acceleration_vector = [xAcc, yAcc, zAcc]

    # Printing acceleration items
    # (for testing purposes).
    # for number in acceleration_vector:
    #     print(format(number, '.2f'), end="\t")
    # print("")

    # Checking whether the touchpad
    # is clicked (for testing purposes).
    # isClickDown:bool = (data[18] & CLICK_BUTTON) > 0
    # print(isClickDown)

    # Send heartbeat packet
    connection.sendto(packet_to_bitarray(heartbeat_packet).bytes, slimevr_server_address)
    
    # Send ping packet
    local_ping_packet = ping_packet
    local_ping_packet = set_id(ping_packet, sensor_id)
    connection.sendto(packet_to_bitarray(local_ping_packet).bytes, slimevr_server_address)

    # Send rotation data packet
    local_rotation_data_packet = rotation_data_packet
    local_rotation_data_packet = set_id(local_rotation_data_packet, sensor_id)
    local_rotation_data_packet = set_quaternion(local_rotation_data_packet, rotation_quaternion)
    connection.sendto(packet_to_bitarray(local_rotation_data_packet).bytes, slimevr_server_address)

    # Send acceleration packet
    local_acceleration_packet = acceleration_packet
    local_acceleration_packet = set_vector(local_acceleration_packet, acceleration_vector)
    #connection.sendto(packet_to_bitarray(local_acceleration_packet).bytes, slimevr_server_address)

# ============

async def connect_to_device(device: BLEDevice, sensor_id):
    print("Connecting to", device.name)
    try:
        # Send handshake packet (establishes initial connection to server)
        connection.sendto(packet_to_bitarray(handshake_packet).bytes, slimevr_server_address)

        # Send sensor info packet
        local_sensor_info_packet = sensor_info_packet
        local_sensor_info_packet = set_id(local_sensor_info_packet, sensor_id)
        connection.sendto(packet_to_bitarray(local_sensor_info_packet).bytes, slimevr_server_address) 
        
        while True:
            async with BleakClient(device, use_cached=False) as client:
            
            # BleakClient must be a device and not an address,
            # otherwise a second float parameter is needed for
            # the the timeout parameter of implicitly called
            # BleakScanner.discover(). This implicit call is not
            # preferred and can be problematic.
            # See link:
            # https://bleak.readthedocs.io/en/latest/api/client.html#:~:text=for%20backwards%20compatibility.-,Warning,-Although%20example%20code

                try:
                    if (not client.is_connected):
                        client.connect()
                        await asyncio.sleep(1.0)
                    
                    connected = client.is_connected

                    if connected:
                        print(f"Connected: {client.address}")

                        # client.set_disconnected_callback(on_disconnect)
                        
                        # Create a BleakGATTCharacteristic object for use in the following blocks
                        characteristic: BleakGATTCharacteristic = client.services.get_characteristic(CHARACTERISTIC_UUID)

                        await client.start_notify(
                            characteristic,
                            partial(notification_handler, sensor_id),
                        )

                        while True:
                            if not connected:
                                break
                            await asyncio.sleep(1.0)
                    else:
                        print(f"Failed to connect to {device.name}")
                except:
                    # await client.stop_notify(characteristic)
                    # Notifications are stopped automatically on disconnect,
                    # so there is no need to stop notifications directly here.
                    # See link:
                    # https://bleak.readthedocs.io/en/latest/api/client.html#:~:text=object%20representing%20it.-,Tip,-Notifications%20are%20stopped
                    
                    if (client.is_connected):
                        await client.disconnect()
                    
                    print("Connection to", device.name, "terminated, reconnecting...")

                    await connect_to_device(device, sensor_id)
    except:
        print("Error communicating with client, retrying...")
        await(connect_to_device(device, sensor_id))

async def main():
    devices = await BleakScanner.discover()
    for index, device in enumerate(devices):
        if device.name == DAYDREAM_NAME:
            print("[BLE Discovery] {}".format(device.address))
            sensor_id = index
            asyncio.create_task(connect_to_device(device, sensor_id))
            await asyncio.sleep(1.0)

if __name__ == "__main__":
    asyncio.run(main())