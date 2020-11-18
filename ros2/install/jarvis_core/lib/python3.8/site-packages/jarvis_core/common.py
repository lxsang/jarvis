import serial
import time
import struct
from geometry_msgs.msg import Point

FRAME_TYPE_DATA = 0
FRAME_TYPE_LOG = 1

LOG_EMERG = 0   # system is unusable
LOG_ALERT = 1   # action must be taken immediately
LOG_CRIT = 2    # critical conditions
LOG_ERR = 3     # error conditions
LOG_WARNING = 4  # warning conditions
LOG_NOTICE = 5  # normal but significant condition
LOG_INFO = 6    # informational
LOG_DEBUG = 7   # debug-level messages


class BaseFrame():
    type = None
    # raw_frame = bytearray()

    @classmethod
    def crc16_ccitt(cls, raw_frame):
        crc = 0xFFFF
        data_len = len(raw_frame)

        if (data_len == 0):
            return 0

        for dbyte in raw_frame:
            crc ^= (dbyte << 8) & 0xFFFF
            for _ in range(8):
                mix = crc & 0x8000
                crc = (crc << 1)
                if mix > 0:
                    crc = crc ^ 0x1021
        return crc & 0xffff

    @classmethod
    def cobs_encode(cls, raw_frame):
        pass

    @classmethod
    def cobs_decode(cls, encoded_frame):
        # check for the first bytes
        offset = 0
        while(offset < len(encoded_frame) and encoded_frame[offset] != 0):
            next_offset = encoded_frame[offset]
            encoded_frame[offset] = 0
            offset = offset + next_offset
        return encoded_frame[1:len(encoded_frame) - 1]

    @classmethod
    def decode(cls, encoded_frame):
        raw_data = cls.cobs_decode(encoded_frame)
        crc = int.from_bytes(raw_data[len(raw_data) - 2:], byteorder='big')
        raw_frame = raw_data[0:len(raw_data) - 2]
        # check the CRC value
        calculated_crc = cls.crc16_ccitt(raw_frame)
        if crc == calculated_crc:
            # check type of frame
            if raw_frame[0] == FRAME_TYPE_DATA:
                # data frame
                frame = DataFrame()
            else:
                if raw_frame[0] == FRAME_TYPE_LOG:
                    # log frame
                    frame = LogFrame()
                else:
                    print("Unknown frame type: %d. Ignore Frame" %
                          raw_frame[0])
                    frame = None
        else:
            print('Frame CRC is not correct. Ignore frame')
            frame = None

        if frame is None:
            return None

        frame.deserialize(raw_frame[1:])
        return frame

    def deserialize(self, raw_frame):
        pass

    def serialize(self):
        pass

    def print(self):
        pass


class LogFrame(BaseFrame):
    level = None
    log_str = None

    def __init__(self):
        self.type = FRAME_TYPE_LOG

    def deserialize(self, raw):
        self.level = raw[0]
        self.log_str = raw[1:].decode("utf-8")

    def __str__(self):
        return self.log_str


class DataFrame(BaseFrame):
    gyro = Point()
    accel = Point()
    mag = Point()
    battery = 0.0
    left_tick = 0
    right_tick = 0

    def __init__(self):
        self.type = FRAME_TYPE_DATA

    def deserialize(self, raw):
        offset = 0
        self.gyro.x = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.gyro.y = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.gyro.z = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4

        self.accel.x = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.accel.y = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.accel.z = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4

        self.mag.x = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.mag.y = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4
        self.mag.z = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4

        self.battery = struct.unpack('>f', raw[offset: offset + 4])[0]
        offset += 4

        self.left_tick = int.from_bytes(
            raw[offset: offset + 2], byteorder='big')
        offset += 2

        self.right_tick = int.from_bytes(
            raw[offset: offset + 2], byteorder='big')

    def __str__(self):
        return "Gyro: %s\nAccelerometer: %s\nMagnetometer: %s\nBattery: %.4f\nLeft tick: %d\nRight tick: %d\n" % (str(self.gyro), str(self.accel), str(self.mag), self.battery, self.left_tick, self.right_tick)


class JarvisSerial():
    """
    docstring
    """
    serial = None
    is_sync = False

    def __init__(self, dev='/dev/ttyACM0', baud=115200):
        self.serial = serial.Serial(dev, baud)

    def sync(self):
        if not self.is_sync:
            self.serial.read_until(b'\x00')
            self.is_sync = True

    def read_frame(self):
        # if self.serial.inWaiting() > 0:
        if self.is_sync == False:
            self.sync()
        bytes_frame = self.serial.read_until(b'\x00')
        return BaseFrame.decode(bytearray(bytes_frame))
        # else:
        #    return None
