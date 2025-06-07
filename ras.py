# 🧰 1. Cài đặt thư viện cần thiết
# Trên Raspberry Pi OS:
# Bạn nên cài thư viện RPi.GPIO hoặc gpiozero.
sudo apt update
sudo apt install python3-rpi.gpio python3-gpiozero

# 📌 2. Sử dụng RPi.GPIO trong Python

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)      # Sử dụng cách đánh số chân GPIO (không phải thứ tự vật lý)
GPIO.setup(18, GPIO.OUT)    # Cài đặt GPIO 18 là OUTPUT

GPIO.output(18, GPIO.HIGH)  # Bật chân 18 (HIGH là 3.3V)
time.sleep(1)
GPIO.output(18, GPIO.LOW)   # Tắt chân 18
time.sleep(1)

GPIO.cleanup()              # Giải phóng tài nguyên khi xong

# 🧾 3. Một số lệnh quan trọng

| Lệnh                          | Ý nghĩa                                |
| ----------------------------- | -------------------------------------- |
| `GPIO.setmode(GPIO.BCM)`      | Dùng đánh số theo chân GPIO (BoardCom) |
| `GPIO.setmode(GPIO.BOARD)`    | Dùng số thứ tự chân trên header vật lý |
| `GPIO.setup(pin, GPIO.OUT)`   | Cài đặt chân là Output                 |
| `GPIO.setup(pin, GPIO.IN)`    | Cài đặt chân là Input                  |
| `GPIO.output(pin, GPIO.HIGH)` | Gửi tín hiệu 3.3V                      |
| `GPIO.output(pin, GPIO.LOW)`  | Gửi tín hiệu 0V                        |
| `GPIO.input(pin)`             | Đọc trạng thái (HIGH/LOW) từ chân      |
| `GPIO.cleanup()`              | Giải phóng chân đã sử dụng             |

# 🚦 4. Sử dụng gpiozero (cách đơn giản hơn)

from gpiozero import LED
from time import sleep

led = LED(18)

while True:
    led.on()
    sleep(1)
    led.off()
    sleep(1)


# #UART   
# 🐍 Trên Raspberry Pi OS (dùng thư viện pyserial)
# 📦 Cài thư viện:

sudo apt install python3-serial

# 📜 Lệnh và hàm UART phổ biến:
| Lệnh / Hàm                      | Ý nghĩa                                               |
| ------------------------------- | ----------------------------------------------------- |
| `serial.Serial(port, baudrate)` | Mở cổng serial (ví dụ: `/dev/serial0`, tốc độ `9600`) |
| `ser.write(b"Hello\n")`         | Gửi dữ liệu (chuỗi bytes)                             |
| `ser.read(size)`                | Đọc `size` byte                                       |
| `ser.readline()`                | Đọc một dòng (kết thúc bởi `\n`)                      |
| `ser.read_all()`                | Đọc mọi dữ liệu có sẵn                                |
| `ser.in_waiting`                | Số byte còn trong buffer                              |
| `ser.close()`                   | Đóng cổng serial                                      |
| `ser.is_open`                   | Kiểm tra cổng còn đang mở không                       |
# 📘 Ví dụ đầy đủ:
import serial
ser = serial.Serial("/dev/serial0", 9600, timeout=1)

ser.write(b"Xin chao!\n")
data = ser.readline()
print(data.decode())
ser.close()

# I2C 
# 🔧 2. Bật I2C trên Raspberry Pi

sudo raspi-config
# Chọn Interface Options

# Chọn I2C → chọn Yes

# Khởi động lại: sudo reboot

# Cài công cụ kiểm tra:
sudo apt install -y i2c-tools
# Xem địa chỉ thiết bị đang nối:
i2cdetect -y 1
# 🐍 3. Dùng Python (smbus2) giao tiếp I2C
# ➤ Cài thư viện:
pip3 install smbus2
# ➤ Ví dụ đơn giản:
# Gửi 1 byte 0x01 tới thiết bị địa chỉ 0x27:
from smbus2 import SMBus

bus = SMBus(1)
address = 0x27
bus.write_byte(address, 0x01)
data = bus.read_byte(address)
print("Received:", data)
