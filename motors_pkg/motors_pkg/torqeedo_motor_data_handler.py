import queue
import numpy as np
from motors_pkg.Enums import TorqeedoMotorCmdEnum, TorqeedoMotorConstantsEnum


class TorqeedoMotorPacket():
    def __init__(self, cmd: int, data: bytes, crc: int, bytes_to_send: bytes):
        self.header = b'\xAC\x30'
        self.cmd = cmd
        self.data = data
        self.crc = crc
        self.terminator = b'\xAD'
        self.bytes_to_send = bytes_to_send

    def __repr__(self):
        return f'Bytes: {self.bytes_to_send}'


class TorqeedoMotorDataHandler():
    def __init__(self):
        self.CRC8INIT = 0x00
        self.CRC8POLY = 0x31
        self.header_1 = b'\xAC'
        self.header_2 = b'\x30'
        self.move_bytes = b'\x09\x64'
        self.stop_bytes = b'\x08\x00'
        self.terminator = b'\xAD'
        self.buffer = bytes(0)
        self.rx_queue = queue.Queue()
        self.state = 1

    def crc8(self, buffer: bytearray):
        crc = np.zeros(1, dtype=np.uint8)
        crc[0] = self.CRC8INIT
        for byte in buffer:
            crc[0] ^= self.reverse(byte)
            for _ in range(8):
                if crc[0] & 0x80:
                    crc[0] = (crc[0] << 1) ^ self.CRC8POLY
                else:
                    crc[0] <<= 1
        return self.reverse(crc[0])

    def reverse(self, crc: int):
        return (crc & 0xF0) >> 4 | (crc & 0x0F) << 4 | (crc & 0xCC) >> 2 | (crc & 0x33) << 2 | (crc & 0xAA) >> 1 | (
                    crc & 0x55) << 1

    def create_packet(self, state, speed: int = 0):
        if state == TorqeedoMotorCmdEnum.SET_SPEED_ID:
            command = TorqeedoMotorCmdEnum.SET_SPEED_ID.value
            bytes_for_crc = self.header_2 + bytes([command]) + (
                self.stop_bytes if speed == 0 else self.move_bytes) + speed.to_bytes(2, byteorder='big', signed=True)
        else:
            command = state.value
            bytes_for_crc = self.header_2 + bytes([command])

        crc_byte = bytes([self.crc8(bytes_for_crc)])
        bytes_to_send = self.header_1 + bytes_for_crc + crc_byte + self.terminator
        return TorqeedoMotorPacket(state, crc_byte[1:], crc_byte, bytes_to_send)

    def one(self):
        if self.byte == self.header_1:
            self.temp_data = bytes(0)
            self.temp_data += self.byte
            self.state += 1
        else:
            self.state = 1

    def two(self):
        self.temp_data += self.byte
        if len(self.temp_data) == TorqeedoMotorConstantsEnum.SMALL_PACKET_LENGTH.value and self.byte == self.terminator:
            self.state = 1
        elif len(
                self.temp_data) == TorqeedoMotorConstantsEnum.FULL_PACKET_LENGTH.value and self.byte == self.terminator:
            speed = int.from_bytes(self.temp_data[3: 5], byteorder='big', signed=True)
            self.rx_queue.put(speed)
            self.state = 1

    def state_machine(self, argument: int):
        switcher = {
            1: self.one,
            2: self.two,
        }
        func = switcher.get(argument, lambda: 'Invalid argument')
        func()

    def parse_byte(self, byte: bytes):
        self.byte = byte
        self.state_machine(self.state)

    def add_data_to_buffer(self, data: bytes):
        self.buffer += data

    def parse_buffer(self):
        for byte in self.buffer:
            self.parse_byte(bytes([byte]))
        self.buffer = bytes(0)


if __name__ == '__main__':
    handler = TorqeedoMotorDataHandler()
    handler.add_data_to_buffer(
        b'\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x1cP\x00E\x01 \x00\x05\x00\xfe\x00\x00\x85\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x1c\x94\x00:\x01\x1f\x00\x05\x01\x01\x00\x00.\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad')
    handler.parse_buffer()
    handler.add_data_to_buffer(
        b'\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x1c(\x00O\x01\x1d\x00\x05\x00\xf8\x00\x00\x03\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad\xac\x00\x00\x1c(\x00I\x01\x1b\x00\x05\x00\xfc\x00\x00\xe0\xad\xac\x00\x00\x00\xad\xac\x00\x00\x00\xad')
    handler.parse_buffer()
    print("hello")
