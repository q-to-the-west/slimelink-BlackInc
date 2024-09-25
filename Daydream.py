import asyncio, math, uuid, quaternion, time, subprocess
import numpy as np
from threading import Event
from functools import partial
from bitstring import BitArray, pack
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.characteristic import BleakGATTCharacteristic

# Extra helpful sources:
# https://gist.github.com/abb128/ec7ba822114508a92fc0156de655e6c7
# https://github.com/SlimeVR/slimevr-node/blob/master/apps/serial-to-tracker/src/index.ts

# TODO: Get battery readings working!

# Globals

buffer: bytes = []

tg = set()
application_name = "Python3 SlimeVR Example"
slimevr_server_address = ('127.0.0.1', 6969)
hobos = False
# connections_complete = False

# Constants

# Bluetooth Constants
DAYDREAM_NAME = "Daydream controller"
SERVICE_UUID = uuid.UUID('0000fe55-0000-1000-8000-00805f9b34fb')
DATA_CHARACTERISTIC_UUID = uuid.UUID('00000001-1000-1000-8000-00805f9b34fb')
BATTERY_CHARACTERISTIC_UUID = uuid.UUID('00002a19-0000-1000-8000-00805f9b34fb')
# DATA_NOTIFICATION_UUID = uuid.UUID('00002902-0000-1000-8000-00805f9b34fb')

# Server Data Constants
READY_TYPE = 0x00
ROTATION_DATA_TYPE = 0x01
ACCELERATION_DATA_TYPE = 0x02
BATTERY_DATA_TYPE = 0x03
PING_DATA_TYPE = 0x0A

# Controller Data Constants
CLICK_BUTTON = 0x1
HOME_BUTTON = 0x2
APP_BUTTON = 0x4
VOLUME_DOWN_BUTTON = 0x8
VOLUME_UP_BUTTON = 0x10
MAX_DEGREES = 180

# Packets
# idek what this is for tbh, seems pointless
heartbeat_packet = {
    "packet_type": 0
}

# Use this packet to reset a handshake???
# See: https://github.com/carl-anders/slimevr-wrangler/blob/main/protocol/src/lib.rs
# See: https://github.com/carl-anders/slimevr-wrangler/blob/main/src/joycon/communication.rs
user_action_packet = {
    "packet_type": 21,
    #"packet_id": ("uint:64",),
    "typ": ("uint:8",)
}

handshake_packet = {
    "packet_type": 3,
    "packet_id": ("uint:64", 1),

    # 1 to 12 are known and specific. 4 is "Custom" so I
    # will use that. Using any other number makes board
    # recognized as "Unknown" by server.
    "board_type": ("int:32", 4),

    # 1 to 9 are known and specific. Using any other number
    # makes sensor recognized as "Unknown" by server.
    "imu_type": ("int:32", 0),

    # 1 and 2 are known and specific. Using any other number
    # makes sensor recognized as "Unknown" by server.
    "mcu_type": ("int:32", 0),

    "imu_info": ("3*int:32", 0, 0, 0),

    # 7 in the example from here:
    # https://github.com/SlimeVR/SlimeVR-Rust/blob/63ee7f7d1d77334d115c050df469c720b5a20736/networking/firmware_protocol/src/serverbound.rs#L172
    # Was previously 8.
    "firmware_build": ("int:32", 7),

    "firmware_name": application_name.encode(),
    "mac": hex(uuid.getnode())
}

# See:
# https://github.com/carl-anders/slimevr-wrangler/blob/main/protocol/src/lib.rs
# https://github.com/SlimeVR/SlimeVR-Rust/blob/63ee7f7d1d77334d115c050df469c720b5a20736/firmware/src/main.rs
# https://github.com/SlimeVR/SlimeVR-Rust/blob/63ee7f7d1d77334d115c050df469c720b5a20736/networking/firmware_protocol/src/serverbound.rs
# for some protocol references.

ping_packet = {
    "packet_type": 10,
    "n1": ("uint:8",),
    "n2": ("uint:8",),
    "n3": ("uint:8",),
    "n4": ("uint:8",)
    # "id": ("4*uint:8", 0, 0, 0, 0)
}

sensor_info_packet = {
    "packet_type": 15,
    "sensor_id": 0,

    # 0 = Ok, 1 = Offline
    "sensor_state": 0,
    "sensor_type": 0
}

rotation_data_packet = {
    "packet_type": 17,
    "sensor_id": 0,

    # 1 = Normal, 2 = Correction
    "data_type": 1,
    "quaternion": ("4*float:32", 0.0, 0.0, 0.0, 0.0),
    "accuracy_info": 0
}

acceleration_data_packet = {
    "packet_type": 4,
    "vector": ("3*float:32", 0.0, 0.0, 0.0),
    "sensor_id": 0
}

battery_data_packet = {
    "packet_type": 12,
    "sensor_id": 0,
    "percentage": 0
}



tVars = {
    # Add variables that each
    # controller can track
    # individually here.

    # Variables should be accessed
    # in asynchronous functions
    # by searching trackedVars via:
    # trackedVars[sensor_id][variable_name]

    # For SlimeVR
    "battery_data": battery_data_packet,
    "sensor_info": sensor_info_packet,
    "rotation_data": rotation_data_packet,
    "acceleration_data": acceleration_data_packet,

    # For HoboVR
    "data": 0x00,
    "packetNum": 0,
    "oldTimeStamp": 0,
    "pastTotalTime": 0,
    "timeStamp": 0,
    "totalTime": 0,
    # "pastVelocity": [0, 0, 0],
    "pastAcceleration": [0, 0, 0],
    "universalMaximum": 0,
    "timeNano": time.time_ns(),
    "recenterToggle": False,
    "disconnectToggle": False,
}



# Logic

def display_bytes(the_bytes, items_per_line: int, name, letters: bool):
    iter = 1
    print(name + ":")
    for index, item in enumerate(the_bytes):
        if item != "":
            if letters:
                print(chr(int(item)), end="")
            else:
                print(item, end="")
            
            if (index == ((items_per_line * iter) - 1)):
                iter += 1
                print("")
            else:
                print("\t", end="")
    print("")

def set_dictionary_item(dictionary, item_name, item):
    dictionary[item_name] = item
    return dictionary

def set_tuple(dictionary, item_name, items):
    dictionary[item_name] = (dictionary[item_name][0], *items)
    return dictionary

# The SlimeVR buffer!!! or smthn idk
def packet_to_bitarray(packet):
    bitarray:BitArray = BitArray()
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

def to_signed(val: int, bit_width: int) -> int:
    # In the following example, I use a realistic
    # input value (val) of 7000 coming from the
    # variable for orientation on the x-axis (xOri,
    # detects roll of the controller) in the
    # data_notification_handler function after it is
    # assigned its raw data.

    # I had a val of 7000 coming into this function
    # from xOri in data_notification_handler by rolling
    # my Daydream controller about 90 degrees
    # counter-clockwise (controller facing forward,
    # buttons facing the left).

    # The binary value for unsigned integer 7000 is
    # 1101101011000 (binary, 13 bits); therefore,
    # val = 1101101011000 (binary, 13 bits)

    # Finding sign_check
    # Given bit_width = 12:
    # Perform the bitwise LEFT SHIFT operation (<<) on integer 1
    # 1 (binary, 1 bit) << bit_width               = 1000000000000 (binary, 13 bits)
    # Remember, val                                = 1101101011000 (binary, 13 bits)
    # Now we perform the bitwise AND operation (&)
    # sign_check                                   = 1000000000000 (binary, 13 bits)
    sign_check = val & (1 << bit_width)

    # Finding val_abs
    # Using the previous values of val and bit_width...
    # We already know that (1 << bit_width)             = 1000000000000 (binary, 13 bits)
    # Let's call that previous 13-bit binary number "num"
    # Now we perform the bitwise NOT operation (~)
    # ~num                                              = 0111111111111 (binary, 13 bits, all leading numbers are 1s)
    # Remember, val                                     = 1101101011000 (binary, 13 bits)
    # Finally, we perform the bitwise AND operation (&)
    # val_abs                                           = 0101101011000 (binary, 12 bits with 1 leading 0)
    val_abs = val & ~(1 << bit_width)

    # Finding our signed value (val_signed)
    # Using the previous values of val, bit_width, and val_abs...
    # sign_check = 1000000000000 (binary, 13 bits) = 4096 (int)
    # val_abs    =  101101011000 (binary, 12 bits) = 2904 (int)
    # sign_check does not equal 0, so the if branch is executed...
    # -4096 + 2904
    # val_signed = -1192
    val_signed = -sign_check + val_abs if sign_check else val_abs

    # Returning our value
    return val_signed

def clearVars(dreamer, sensor_id: int):
    dreamer.trackedVars[sensor_id] = dict(tVars)


# === MAIN ===

class SlimeProtocol:
    def __init__(self, dreamer):
        self.transport: asyncio.DatagramTransport = None
        self.dreamer = dreamer
        self.initial_send = True
        self.current_device = 0
        # self.ready = False
        # TODO: Handle handshake attempts after server is already connected.
        # This means that disabling the program and restarting it will
        # reconnect pre-established devices instead of creating new ones.
        self.hand_shaken = False
    
    def connection_made(self, transport: asyncio.DatagramTransport):
        self.transport = transport
        local_handshake_packet = dict(handshake_packet) # 616 bits
        # for index, item in enumerate(trackedVars):
            # set_dictionary_item(local_handshake_packet, "sensor_id", index)
        transport.sendto(packet_to_bitarray(local_handshake_packet).bytes)

    def datagram_received(self, data, addr):
        if self.initial_send:
            display_bytes(data, 8, "Handshake packet", True)
            self.initial_send = False
        
        # Periodic extra updates on the status of
        # controllers as requested by the server.
        elif len(self.dreamer.trackedVars) > 0:
            display_bytes(data, 8, "Incoming data", False)
            request_type: bytes = 0x00

            for item in data:
                if (item != 0):
                    request_type = item
                    # If you untext the if chain below this break,
                    # make sure to delete the break so everything
                    # else can execute.
                    break

            # Due to the code below,
            # The ping shown within SlimeVR is essentially
            # the amount of milliseconds that it takes for
            # one cycle of daydream controllers (starting
            # and ending with the current daydream controller)
            # to have received and sent ping info.
            if request_type == PING_DATA_TYPE:
                self.transport.sendto(packet_to_bitarray(ping_handler(data)).bytes)
                self.current_device += 1
                if self.current_device >= len(self.dreamer.trackedVars):
                    self.current_device = 0

            # Send heartbeat packet
            # Currently not sending, idk what this is even for
            # self.transport.sendto(packet_to_bitarray(heartbeat_packet).bytes)
            # print(request_type)

        # print("Received.")

    def error_received(self, exc):
        print('Error received:', exc)

    def connection_lost(self, exc):
        print("Connection closed.")


# ============

# Not used in the main handlers but used manually elsewhere
def handle_sensor_info(dreamer, sensor_id: int, transport: asyncio.DatagramTransport):
    local_sensor_info_packet = dict(sensor_info_packet)
    local_sensor_info_packet = set_dictionary_item(local_sensor_info_packet, "sensor_id", sensor_id)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "sensor_info", local_sensor_info_packet)
    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["sensor_info"]).bytes)

# Goated source: https://stackoverflow.com/questions/40730809/use-daydream-controller-on-hololens-or-outside-daydream
def handle_orientation(dreamer, sensor_id: int, data: bytearray):

    # First Bitmask:
    # 0x03 << 11 = 1100 0000 << 11 = 1 1000 0000 0000
    # 0xFF <<  3 = 1111 1111 <<  3 =    111 1111 1000
    # 0xE0 >>  5 = 1110 0000 >>  5 =        0000 0111
    xOri = (data[1] & 0x03) << 11 | (data[2] & 0xFF) << 3 | (data[3] & 0xE0) >> 5

    # 13 total bits: 1 sign and 12 binary digits.
    # This means we have a bit_width of 12 to provide the function.
    xOri = to_signed(xOri, 12)


    # Second Bitmask:
    # 0x1F << 8 = 0001 1111 << 8 = 1 1111 0000 0000
    # 0xFF      = 1111 1111      =        1111 1111
    yOri = (data[3] & 0x1F) << 8 | (data[4] & 0xFF)

    yOri = to_signed(yOri, 12)


    # Third Bitmask:
    # 0xFF << 5 = 1111 1111 << 5 = 1 1111 1110 0000
    # 0xF8 >> 3 = 1111 1000 >> 3 =        0001 1111
    zOri = (data[5] & 0xFF) << 5 | (data[6] & 0xF8) >> 3

    zOri = to_signed(zOri, 12)


    axis = [-xOri, yOri, zOri]
    vector = np.array([(2 * math.pi / 4095.0) * x for x in axis])
    orientation = quaternion.from_rotation_vector(vector)

    r = quaternion.as_float_array(orientation) # w x y z
    rotation_quaternion = (r[1], r[3], r[2], r[0]) # x z y w
    
    # Send rotation data packet
    local_rotation_data_packet = rotation_data_packet #dict(rotation_data_packet)
    local_rotation_data_packet = set_dictionary_item(local_rotation_data_packet, "sensor_id", sensor_id)
    local_rotation_data_packet = set_tuple(local_rotation_data_packet, "quaternion", rotation_quaternion)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "rotation_data", local_rotation_data_packet)

def handle_acceleration(dreamer, sensor_id: int, data: bytearray):
    # First Bitmask:
    # 0x07 << 10 = 0000 0111 << 10 = 1 1100 0000 0000
    # 0xFF <<  2 = 1111 1111 <<  2 =     11 1111 1100
    # 0xC0 >>  6 = 1100 0000 >>  6 =        0000 0011
    xAcc = (data[6] & 0x07) << 10 | (data[7] & 0xFF) << 2 | (data[8] & 0xC0) >> 6

    xAcc = to_signed(xAcc, 12)


    # Second Bitmask:
    # 0x3F << 7 = 0011 1111 = 1 1111 1000 0000
    # 0xFE >> 1 = 1111 1110 =        0111 1111
    yAcc = (data[8] & 0x3F) << 7 | (data[9] & 0xFE) >> 1

    yAcc = to_signed(yAcc, 12)


    # Third Bitmask:
    # 0x01 << 12 = 0000 0001 << 12 = 1 0000 0000 0000
    # 0xFF <<  4 = 1111 1111 <<  4 =   1111 1111 0000
    # 0xF0 >>  4 = 1111 0000 <<  4 =        0000 1111
    zAcc = (data[9] & 0x01) << 12 | (data[10] & 0xFF) << 4 | (data[11] & 0xF0) >> 4

    zAcc = to_signed(zAcc, 12)


    # X appears to be forward and back
    # Y appears to be left and right
    # Z appears to be up and down
    axis = [-xAcc, -yAcc, zAcc]
    acceleration_vector = ((8 * 9.8 / 4095.0) * x for x in axis)
    
    local_acceleration_data_packet = acceleration_data_packet #dict(rotation_data_packet)
    local_acceleration_data_packet = set_dictionary_item(local_acceleration_data_packet, "sensor_id", sensor_id)
    local_acceleration_data_packet = set_tuple(local_acceleration_data_packet, "vector", acceleration_vector)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "acceleration_data", local_acceleration_data_packet)

def report_battery_data(dreamer, sensor_id: int, data: bytearray, transport):
    percentage = data[0]
    local_battery_data_packet = battery_data_packet
    local_battery_data_packet = set_dictionary_item(local_battery_data_packet, "sensor_id", sensor_id)
    local_battery_data_packet = set_dictionary_item(local_battery_data_packet, "percentage", percentage)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "battery_data", local_battery_data_packet)

    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["battery_data"]).bytes)

def data_notification_handler(dreamer, sensor_id: int, protocol: SlimeProtocol, client: BleakClient, characteristic: BleakGATTCharacteristic, data: bytearray):
    # if protocol.ready:
    set_dictionary_item(dreamer.trackedVars[sensor_id], "data", data)
    handle_orientation(dreamer, sensor_id, data)
    handle_acceleration(dreamer, sensor_id, data)
    transport: asyncio.DatagramTransport = protocol.transport
    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["rotation_data"]).bytes)
    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["acceleration_data"]).bytes)

def ping_handler(info: bytes):
    length = len(info)
    local_ping_packet = dict(ping_packet)

    cur: int = 4
    count: int = 1
    while cur > 0:
        set_dictionary_item(local_ping_packet, "n" + str(count), info[length - cur])
        cur -= 1
        count += 1

    return local_ping_packet

# This notification doesn't trigger often; it might not trigger at all.
# My assumption is that this notification only triggers when the battery
# level of the Google Daydream controller changes from its current value.
def battery_notification_handler(dreamer, sensor_id: int, protocol: SlimeProtocol, characteristic: BleakGATTCharacteristic, data: bytearray):
    # if protocol.ready:
    percentage = data[0]
    local_battery_data_packet = battery_data_packet
    local_battery_data_packet = set_dictionary_item(local_battery_data_packet, "sensor_id", sensor_id)
    local_battery_data_packet = set_dictionary_item(local_battery_data_packet, "percentage", percentage)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "battery_data", local_battery_data_packet)

    transport: asyncio.DatagramTransport = protocol.transport
    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["battery_data"]).bytes)

def on_disconnect(dreamer, sensor_id: int, transport: asyncio.DatagramTransport):
    # TODO: Watch performance of this disconnect callback and observe if it performs properly
    local_sensor_info_packet = dict(sensor_info_packet)
    local_sensor_info_packet = set_dictionary_item(local_sensor_info_packet, "sensor_id", sensor_id)
    local_sensor_info_packet = set_dictionary_item(local_sensor_info_packet, "sensor_state", 1)
    set_dictionary_item(dreamer.trackedVars[sensor_id], "sensor_info", local_sensor_info_packet)
    transport.sendto(packet_to_bitarray(dreamer.trackedVars[sensor_id]["sensor_info"]).bytes)

# TODO: Write code to send a new sensor info packet that marks an offline controller as online when it reconnects
# helpful link: https://github.com/SlimeVR/slimevr-node/blob/1108c6d9b260f663f2bb3510b987d09e047d4464/apps/serial-to-tracker/src/index.ts#L149
async def connect_to_device(dreamer, device: BLEDevice, sensor_id: int, protocol: SlimeProtocol):
    transport: asyncio.DatagramTransport = protocol.transport
    print("Connecting to", device.name)
    try:
        async with BleakClient(device, disconnected_callback=on_disconnect(dreamer, sensor_id, transport), use_cached=False) as client:
            handle_sensor_info(dreamer, sensor_id, transport)
        
        # BleakClient must be a device and not an address,
        # otherwise a second float parameter is needed for
        # the the timeout parameter of implicitly called
        # BleakScanner.discover(). This implicit call is not
        # preferred and can be problematic.
        # See link:
        # https://bleak.readthedocs.io/en/latest/api/client.html#:~:text=for%20backwards%20compatibility.-,Warning,-Although%20example%20code
        
            while True:
                # if (not connections_complete):
                #     await asyncio.sleep(0.5)
                #     continue
                try:
                    if (not client.is_connected):
                        await client.connect()
                    
                    connected = client.is_connected

                    if connected:
                        print(f"Connected: {client.address}")
                        # client.set_disconnected_callback(on_disconnect)

                        await client.pair()
                        
                        # Create BleakGATTCharacteristic objects for use in the following blocks
                        data_characteristic: BleakGATTCharacteristic = client.services.get_characteristic(DATA_CHARACTERISTIC_UUID)
                        battery_characteristic: BleakGATTCharacteristic = client.services.get_characteristic(BATTERY_CHARACTERISTIC_UUID)
                        
                        await client.start_notify(
                            data_characteristic,
                            partial(data_notification_handler, dreamer, sensor_id, protocol, client),
                        )
                        
                        await client.start_notify(
                           battery_characteristic,
                           partial(battery_notification_handler, dreamer, sensor_id, protocol),
                        )

                        while True:
                            if not connected:
                                await asyncio.sleep(0.5)
                                break
                            # TODO: Please get battery working ;-;
                            # Handling battery data here as well since the
                            # notification for it doesn't seem to update often.
                            # battery_data = await client.read_gatt_char(battery_characteristic)
                            # report_battery_data(dreamer, sensor_id, battery_data, transport)
                            await asyncio.sleep(0.2)
                        await client.unpair()
                    else:
                        print(f"Failed to connect to {device.name}")
                except AttributeError as e:
                        print(f"Error: {e}")
                        await connect_to_device(dreamer, device, sensor_id, protocol)
                except:
                    if (client.is_connected):
                        await client.disconnect()
                    
                    print("Connection to", device.name, "terminated, reconnecting...")
                    clearVars(dreamer, sensor_id)
                    await connect_to_device(dreamer, device, sensor_id, protocol)
    except:
        print("Error communicating with client or server, retrying...")
        await(connect_to_device(dreamer, device, sensor_id, protocol))


class SlimeDreamer:
    def __init__(self):
        self.trackedVars = []
        self.devices = []
        self.connections_initiated = False

    async def start(self):
        trackedVars_index = 0

        self.devices = await BleakScanner.discover()

        async with asyncio.TaskGroup() as tg:
            loop: asyncio.AbstractEventLoop = asyncio.get_running_loop()
            transport, protocol = await loop.create_datagram_endpoint(lambda: SlimeProtocol(self), remote_addr = slimevr_server_address)

            for index, device in enumerate(self.devices):
                if device.name == DAYDREAM_NAME:
                    print("[BLE Discovery] {}".format(device.address))
                    self.trackedVars.append(dict(tVars))
                    tg.create_task(connect_to_device(self, device, trackedVars_index, protocol))
                    trackedVars_index += 1
                    await asyncio.sleep(1.0)

            self.connections_initiated = True

async def main():
    daydreaming_process = SlimeDreamer()
    await daydreaming_process.start()

if __name__ == "__main__":
     asyncio.run(main())
