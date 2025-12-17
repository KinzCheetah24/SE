from serial import Serial
from serial.serialutil import SerialException
from PySide6.QtCore import QThread, Signal


class InputThread(QThread):
    temperature_ready = Signal(int)
    humidity_ready = Signal(float)
    noise_ready = Signal(int)
    fan_ready = Signal(int)
    leds_ready = Signal(list)
    co2_ready = Signal(int)
    luminosity_ready = Signal(float)

    serial: Serial

    def __init__(self, serial: Serial):
        super().__init__()
        self.serial = serial

    def read_uart(self) -> bytes:
        byte = self.serial.read(1)

        if byte != bytes.fromhex(''):
            if byte == bytes.fromhex('AA'):
                print('\n')
            print(f'0x{byte.hex()} | {byte.decode(errors="ignore")}')

        return byte

    def parse_frame_type(self, frame_command: bytes, payload: bytes):
        if frame_command == bytes.fromhex('00'):  # Noise
            self.noise_ready.emit(int.from_bytes(payload, 'big'))
        elif frame_command == bytes.fromhex('01'):  # luminosity
            self.luminosity_ready.emit(int.from_bytes(payload, 'big') * 0.005)
        elif frame_command == bytes.fromhex('02'):  # CO2
            self.co2_ready.emit(int.from_bytes(payload, 'big'))
        elif frame_command == bytes.fromhex('03'):  # humidity
            self.humidity_ready.emit(int.from_bytes(payload, 'big'))
            # self.humidity_ready.emit(int.from_bytes(payload, 'big') * (100/1023.0))
        elif frame_command == bytes.fromhex('04'):  # fan
            # self.fan_ready.emit(int.from_bytes(payload, 'big'))
            pass
        elif frame_command == bytes.fromhex('05'):  # leds
            # self.leds_ready.emit([payload[0], payload[1], payload[2], payload[3]])
            pass
        elif frame_command == bytes.fromhex('06'):  # temperature
            self.temperature_ready.emit(int.from_bytes(payload, 'big'))
            # self.temperature_ready.emit(int.from_bytes(payload, 'big') * (5/1023.0) * 100)
        else:
            print(f'Unknown command: 0x{frame_command.hex().upper()}')

    def loop(self):
        header: bytes = self.read_uart()
        if header == bytes.fromhex('AA'):  # Frame Header
            length: int = int.from_bytes(self.read_uart(), 'big')
            frame_command: bytes = self.read_uart()
            payload: bytes = b''.join([self.read_uart() for i in range(length - 1)])
            crc: bytes = b''.join([self.read_uart() for i in range(2)])

            self.parse_frame_type(frame_command, payload)

    def run(self):
        try:
            while True:
                self.loop()
        except SerialException:
            exit(-1)
