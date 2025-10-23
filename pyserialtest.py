import serial
import time

# Seri port ayarlarını kendi Arduino portuna göre düzenle
PORT = '/dev/tty.usbserial-120'  # veya /dev/ttyACM0, /dev/ttyUSB0
BAUDRATE = 9600
TIMEOUT = 1

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    print(f"Serial port {PORT} opened.")
except serial.SerialException as e:
    print(f"Serial port error: {e}")
    exit(1)

print("Arduino'ya veri gönderiliyor ve cevap bekleniyor:")

try:
    while True:
        # Kullanıcıdan veri al ve Arduino'ya gönder
        user_input = input("Arduino'ya gönderilecek veri: ")
        ser.write((user_input + '\n').encode())

        # Arduino'dan gelen cevabı oku ve ekrana yaz
        time.sleep(0.2)
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            print("Arduino'dan gelen:", line)
except KeyboardInterrupt:
    print("Program sonlandırıldı.")
finally:
    ser.close()
    print("Serial port kapatıldı.")