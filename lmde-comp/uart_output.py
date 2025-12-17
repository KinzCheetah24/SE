from typing import Dict
from queue import Queue
from time import sleep

from serial import Serial
from serial.serialutil import SerialException

from PySide6.QtCore import QThread


class OutputThread(QThread):
    serial: Serial
    queue_turn: int = 0
    out_queues: Dict[bytes, Queue]

    def __init__(self, serial: Serial):
        super().__init__()
        self.out_queues = {
            bytes.fromhex('04'): Queue(maxsize=1),
            bytes.fromhex('05'): Queue(maxsize=1)
        }
        self.serial = serial

    def send_command(self, command: bytes):
        queue: Queue = self.out_queues.get(command[2].to_bytes(1, 'big'))

        if queue.full():
            queue.get_nowait()
        queue.put_nowait(command)

        # print(f'Enqueued: 0x{command.hex()}')


    def send_uart(self, msg_out: bytes):
        for i in range(len(msg_out)):
            self.serial.write(msg_out[i].to_bytes(1, 'big'))
            self.msleep(20)
            # sleep(0.02)  # 20 ms

        # print(f'Sent: 0x{msg_out.hex()}')


    def loop(self):
        queue = list(self.out_queues.values())[self.queue_turn]

        if not queue.empty():
            msg_out: bytes = queue.get_nowait()
            self.send_uart(msg_out)

        self.queue_turn = (self.queue_turn + 1) % len(self.out_queues)


    def run(self):
        try:
            while True:
                self.loop()
        except SerialException:
            exit(-1)