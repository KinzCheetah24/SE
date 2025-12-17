# This Python file uses the following encoding: utf-8
import sys

from serial import Serial
from serial.serialutil import SerialException

# Important:
# You need to run the following command to generate the ui_form.py file
#     pyside6-uic form.ui -o ui_form.py
from ui_form import Ui_MainWindow
from PySide6.QtWidgets import QApplication, QMainWindow

from uart_input import InputThread
from uart_output import OutputThread


UART_PORT = '/dev/ttyUSB0'
UART_BAUDRATE = 9600


class MainWindow(QMainWindow):
    serial: Serial = None
    uart_input_thread: InputThread
    uart_output_thread: OutputThread

    def __init__(self, parent=None):
        super().__init__(parent)

        try:
            self.serial = Serial(UART_PORT, UART_BAUDRATE)
        except SerialException:
            print(f'No fue posible abrir {UART_PORT}')
            exit(f'Could not open serial port: {UART_PORT}')

        # Center window to primary screen
        self.move(
            QApplication.screens()[0].geometry().left() + (QApplication.screens()[0].geometry().width() - self.geometry().width()) // 2,
            QApplication.screens()[0].geometry().top() + (QApplication.screens()[0].geometry().height() - self.geometry().height()) // 2
        )
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.horizontalSlider_r.valueChanged.connect(self.send_color)
        self.ui.horizontalSlider_g.valueChanged.connect(self.send_color)
        self.ui.horizontalSlider_b.valueChanged.connect(self.send_color)
        self.ui.horizontalSlider_i.valueChanged.connect(self.send_color)
        self.ui.progressBar_fan.valueChanged.connect(self.send_fan)

        self.uart_input_thread = InputThread(self.serial)
        self.uart_output_thread = OutputThread(self.serial)

        self.uart_input_thread.temperature_ready.connect(self.update_temperature)
        self.uart_input_thread.humidity_ready.connect(self.update_humidity)
        self.uart_input_thread.noise_ready.connect(self.update_noise)
        self.uart_input_thread.fan_ready.connect(self.update_fan)
        self.uart_input_thread.leds_ready.connect(self.update_leds)
        self.uart_input_thread.co2_ready.connect(self.update_co2)
        self.uart_input_thread.luminosity_ready.connect(self.update_luminosity)


        self.uart_input_thread.start()
        self.uart_output_thread.start()


    def closeEvent(self, event):
        self.serial.close()
        self.uart_input_thread.terminate()
        self.uart_output_thread.terminate()
        self.uart_input_thread.wait()
        self.uart_output_thread.wait()
        event.accept()

    def send_color(self):
        self.ui.color.setStyleSheet(
            f'background-color: rgb('
            f'{self.ui.horizontalSlider_r.value() * (self.ui.horizontalSlider_i.value() * (100/31)) * 0.01:.0f},'
            f'{self.ui.horizontalSlider_g.value() * (self.ui.horizontalSlider_i.value() * (100/31)) * 0.01:.0f},'
            f'{self.ui.horizontalSlider_b.value() * (self.ui.horizontalSlider_i.value() * (100/31)) * 0.01:.0f}'
            f')'
        )

        self.uart_output_thread.send_command(
            bytes.fromhex('AA') +  # Header
            int(5).to_bytes(1, 'big') +  # Length
            bytes.fromhex('05') +  # Command
            int(self.ui.horizontalSlider_r.value()).to_bytes(1, 'big') +  # Payload
            int(self.ui.horizontalSlider_g.value()).to_bytes(1, 'big') +
            int(self.ui.horizontalSlider_b.value()).to_bytes(1, 'big') +
            int(self.ui.horizontalSlider_i.value()).to_bytes(1, 'big') +
            bytes.fromhex('00') +  # CRC
            bytes.fromhex('00')
        )

    def send_fan(self):
        self.uart_output_thread.send_command(
            bytes.fromhex('AA') +  # Header
            int(2).to_bytes(1, 'big') +  # Length
            bytes.fromhex('04') +  # Command
            int(self.ui.dial_fan.value()).to_bytes(1, 'big') +  # Payload
            bytes.fromhex('00') +  # CRC
            bytes.fromhex('00')
        )

    def update_temperature(self, data):
        self.ui.lcd_temp.display(data)

    def update_humidity(self, data):
        self.ui.lcd_humidity.display(data)

    def update_noise(self, data):
        self.ui.progressBar_noise.setValue(66 - (data * 33))

    def update_fan(self, data):
        self.ui.dial_fan.setValue(data)

    def update_leds(self, data):
        self.ui.horizontalSlider_r.setValue(data[0])
        self.ui.horizontalSlider_g.setValue(data[1])
        self.ui.horizontalSlider_b.setValue(data[2])
        self.ui.horizontalSlider_i.setValue(data[3])

    def update_co2(self, data):
        self.ui.lcd_co2.display(data)

    def update_luminosity(self, data):
        self.ui.lcd_luminosity.display(data)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
