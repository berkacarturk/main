from kivy.app import App # type: ignore
from kivy.uix.floatlayout import FloatLayout # type: ignore
from kivy.uix.button import Button # type: ignore
from kivy.uix.togglebutton import ToggleButton # type: ignore
from kivy.uix.label import Label # type: ignore
from kivy.graphics import Color, Ellipse, Line, Triangle, InstructionGroup,Rectangle # type: ignore
from kivy.core.window import Window # type: ignore
from kivy.uix.widget import Widget # type: ignore
from kivy.clock import Clock # type: ignore
import random
from kivy.config import Config # type: ignore
from kivy.uix.slider import Slider # type: ignore

# Pencere boyutunu 800x480 olarak ayarla
Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Config.set('graphics', 'resizable', False)
Config.write()
import serial # type: ignore
import json 
import math
import time
from kivy.uix.boxlayout import BoxLayout # type: ignore
from kivy.uix.gridlayout import GridLayout # type: ignore
import threading
from collections import deque

# DEBUG MOD
DEBUG = True
original_print = print
def print(*args, **kwargs):
    if DEBUG:
        original_print(*args, **kwargs)

class MadgwickFilter:
    def __init__(self, beta=0.1):
        self.beta = beta  # Algorithm gain
        self.q = [1.0, 0.0, 0.0, 0.0]  # Quaternion [w, x, y, z]
        
    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz, dt):
        """
        Madgwick AHRS algorithm update
        gx, gy, gz: Gyroscope measurements (rad/s)
        ax, ay, az: Accelerometer measurements (normalized)
        mx, my, mz: Magnetometer measurements (normalized)
        dt: Time step (seconds)
        """
        q1, q2, q3, q4 = self.q
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return
        norm = 1.0 / norm
        ax *= norm
        ay *= norm
        az *= norm
        
        # Normalize magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0.0:
            return
        norm = 1.0 / norm
        mx *= norm
        my *= norm
        mz *= norm
        
        # Auxiliary variables
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = 2.0 * q1 * q3
        _2q3q4 = 2.0 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4
        
        # Reference direction of Earth's magnetic field
        _2q1mx = 2.0 * q1 * mx
        _2q1my = 2.0 * q1 * my
        _2q1mz = 2.0 * q1 * mz
        _2q2mx = 2.0 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz
        
        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        
        # Normalize step magnitude
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        if norm != 0.0:
            norm = 1.0 / norm
            s1 *= norm
            s2 *= norm
            s3 *= norm
            s4 *= norm
        
        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4
        
        # Integrate to yield quaternion
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        
        # Normalize quaternion
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        if norm != 0.0:
            norm = 1.0 / norm
            self.q[0] = q1 * norm
            self.q[1] = q2 * norm
            self.q[2] = q3 * norm
            self.q[3] = q4 * norm
    
    def get_euler_angles(self):
        """Quaternion'dan Euler açılarını hesapla (derece)"""
        q0, q1, q2, q3 = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp) * 180.0 / math.pi
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) * 180.0 / math.pi
        else:
            pitch = math.asin(sinp) * 180.0 / math.pi
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp) * 180.0 / math.pi
        
        if yaw < 0:
            yaw += 360.0
            
        return roll, pitch, yaw

class ArrowButton(Button):
    def __init__(self, direction, label_text, **kwargs):
        super().__init__(**kwargs)
        self.direction = direction
        self.text = label_text
        self.font_size = 24
        self.background_normal = ''
        self.background_color = (0, 0, 0, 0)
        self.size_hint = (None, None)
        self.size = (60, 60)

        with self.canvas.before:
            self.bg_color = Color(0.2, 0.4, 0.7)  # Modern mavi
            self.bg_circle = Ellipse(pos=self.pos, size=self.size)

        self.bind(pos=self.update_graphics, size=self.update_graphics)
        self.canvas_instruction = InstructionGroup()
        self.canvas.add(self.canvas_instruction)

        self.bind(on_press=self.on_arrow_press)
        self.bind(on_release=self.on_arrow_release)  # Release event'i bağla

    def on_arrow_press(self, instance):
        # Ana ekrana erişip lock durumunu kontrol et
        parent = self.parent
        while parent and not hasattr(parent, 'send_to_arduino'):
            parent = parent.parent
        if parent and hasattr(parent, 'send_to_arduino'):
            # ADMIN MODE KONTROLÜ - Admin modu kapalıysa hiçbir komut gönderme
            if hasattr(parent, 'admin_mode_active') and not parent.admin_mode_active:
                print(f"❌ ADMIN MODU KAPALI - {self.text} komutu engellendi")
                return
            
            # Hangi eksenin butonu olduğunu belirle
            axis = self.text[0]  # X+, X-, Y+, Y-, Z+, Z- formatından eksen harfini al
            
            # NOT: Lock kontrolü KALDIRILDI - motorlar her zaman çalışmalı
            # Lock durumu sadece UI'da gösterilir, motor hareketini ETKİLEMEZ
            
            # Normal şekilde komut gönder
            print(f"✅ {self.text} komutu gönderiliyor")
            # Debug: Motor Arduino durumunu kontrol et
            if hasattr(parent, 'motor_arduino') and parent.motor_arduino:
                print(f"🔌 Motor Arduino bağlı: {parent.motor_arduino.port}")
            else:
                print(f"❌ Motor Arduino bağlı değil!")
            parent.send_to_arduino(f"{self.text}")

    def reset_button_color(self):
        """Buton rengini normale döndür"""
        with self.canvas.before:
            from kivy.graphics import Color
            Color(0.2, 0.4, 0.7)  # Modern mavi (normal renk)

    def on_arrow_release(self, instance):
        parent = self.parent
        while parent and not hasattr(parent, 'send_to_arduino'):
            parent = parent.parent
        if parent and hasattr(parent, 'send_to_arduino'):
            # ADMIN MODE KONTROLÜ - Admin modu kapalıysa hiçbir komut gönderme
            if hasattr(parent, 'admin_mode_active') and not parent.admin_mode_active:
                print(f"❌ ADMIN MODU KAPALI - STOP komutu engellendi")
                return
            
            # Hangi eksenin butonu olduğunu belirle
            axis = self.text[0]  # X+, X-, Y+, Y-, Z+, Z- formatından eksen harfini al
            
            # Lock kontrolü yap - lock edilmişse STOP komutu da gönderme
            if hasattr(parent, 'axis_locked') and parent.axis_locked.get(axis, False):
                print(f"❌ {axis} ekseni lock - {self.text} STOP komutu engellendi")
                return  # Lock edilmişse STOP komutu da gönderme
            
            # Lock edilmemişse normal şekilde STOP komutu gönder
            if "X" in self.text:
                parent.send_to_arduino("STOPX")
            elif "Y" in self.text:
                parent.send_to_arduino("STOPY")
            elif "Z" in self.text:
                parent.send_to_arduino("STOPZ")

    def update_graphics(self, *args):
        self.bg_circle.pos = self.pos
        self.bg_circle.size = self.size

        self.canvas_instruction.clear()
        self.canvas_instruction.add(Color(0.851, 0.851, 0.851))
        cx, cy = self.center
        s = 25
        if self.direction == 'up':
            self.canvas_instruction.add(Triangle(points=[cx, cy + s, cx - s, cy - s, cx + s, cy - s]))
        elif self.direction == 'down':
            self.canvas_instruction.add(Triangle(points=[cx, cy - s, cx - s, cy + s, cx + s, cy + s]))
        elif self.direction == 'left':
            self.canvas_instruction.add(Triangle(points=[cx - s, cy, cx + s, cy - s, cx + s, cy + s]))
        elif self.direction == 'right':
            self.canvas_instruction.add(Triangle(points=[cx + s, cy, cx - s, cy - s, cx - s, cy + s]))

class GyroDisplay(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.gyro_x = 0
        self.gyro_y = 0
        
        # STABILIZASYON İÇİN FILTRE
        self.last_x = None
        self.last_y = None
        self.update_threshold = 0.2  # Daha küçük değişimleri de algıla
        
        # LOW-PASS FILTER için önceki değerler
        self.filtered_x = 0
        self.filtered_y = 0
        self.filter_alpha = 0.3  # Daha hızlı tepki için artırıldı
        
        # ARDUINO VERİ KONTROLÜ
        self.last_data_time = 0
        self.data_timeout = 1.0  # 1 saniye veri gelmezse gyro'yu sıfırla (2'den 1'e düşürüldü)
        
        # İlk başta su terazisini çiz
        Clock.schedule_once(self.update_display, 0.1)
        Clock.schedule_interval(self.check_data_timeout, 1.0)  # Her saniye kontrol et
        
    def update_gyro_data(self, gyro_x, gyro_y):
        self.last_data_time = time.time()  # Veri geldiği zamanı kaydet
        
        # LOW-PASS FILTER uygula - gürültüyü azalt
        self.filtered_x = (self.filter_alpha * gyro_x) + ((1 - self.filter_alpha) * self.filtered_x)
        self.filtered_y = (self.filter_alpha * gyro_y) + ((1 - self.filter_alpha) * self.filtered_y)
        
        # DEAD ZONE - küçük titreşimleri yok say (azaltıldı)
        dead_zone = 0.3  # 0.8'den 0.3'e düşürüldü - daha hassas
        if abs(self.filtered_x) < dead_zone:
            self.filtered_x = 0
        if abs(self.filtered_y) < dead_zone:
            self.filtered_y = 0
        
        # PERFORMANS: Sadece önemli değişikliklerde güncelle
        if (self.last_x is None or 
            abs(self.filtered_x - self.last_x) > self.update_threshold or 
            abs(self.filtered_y - self.last_y) > self.update_threshold):
            
            self.gyro_x = self.filtered_x
            self.gyro_y = self.filtered_y
            self.last_x = self.filtered_x
            self.last_y = self.filtered_y
            self.update_display()
    
    def check_data_timeout(self, dt):
        """Arduino'dan veri gelmediği durumda gyro'yu tamamen durdur"""
        import time
        current_time = time.time()
        
        if self.last_data_time == 0:
            # İlk kez başlatılıyorsa merkezi göster
            self.gyro_x = 0
            self.gyro_y = 0
            self.filtered_x = 0
            self.filtered_y = 0
            self.update_display()
            return
        
        if (current_time - self.last_data_time) > self.data_timeout:
            # Veri gelmiyorsa TAMAMEN durdur - kayma olmasın
            self.gyro_x = 0
            self.gyro_y = 0
            self.filtered_x = 0
            self.filtered_y = 0
            self.last_x = 0
            self.last_y = 0
            self.update_display()
        
    def update_display(self, *args):
        self.canvas.clear()
        with self.canvas:
            # Daire halkaları
            Color(0.8, 0.8, 0.8)
            for r in [20, 40, 60]:
                Line(circle=(self.center_x, self.center_y, r), width=1.2)
            
            # Merkez çizgiler
            Color(0.6, 0.6, 0.6)
            Line(points=[self.center_x - 70, self.center_y, self.center_x + 70, self.center_y], width=1)
            Line(points=[self.center_x, self.center_y - 70, self.center_x, self.center_y + 70], width=1)
            
            # HASSASİYET 5 KATINA ÇIKARILDI
            limited_x = max(-8, min(8, self.gyro_x))
            limited_y = max(-8, min(8, -self.gyro_y))

            # Ölçekleme faktörü 5 katına çıkarıldı
            point_x = self.center_x + (limited_x * 5)
            point_y = self.center_y + (limited_y * 5)

            # Nokta sınırları içinde kalsın
            max_radius = 65
            distance_from_center = ((point_x - self.center_x)**2 + (point_y - self.center_y)**2)**0.5
            if distance_from_center > max_radius:
                ratio = max_radius / distance_from_center
                point_x = self.center_x + (point_x - self.center_x) * ratio
                point_y = self.center_y + (point_y - self.center_y) * ratio
            
            # Su terazisi noktası
            Color(0.3, 0.7, 1, 1)  # Açık mavi nokta
            Ellipse(pos=(point_x - 10, point_y - 10), size=(20, 20))
            
            # Merkez noktası
            Color(0.1, 0.9, 0.5, 1)  # Cyan-yeşil merkez
            Ellipse(pos=(self.center_x - 4, self.center_y - 4), size=(8, 8))

class ColoredToggleButton(ToggleButton):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.background_normal = ''
        self.size_hint = (None, None)
        self.background_color = (0.2, 0.4, 0.7, 1)  # Modern mavi
        self.size = kwargs.get('size', (60, 40))
        self.default_color = (0.2, 0.4, 0.7, 1)  # Modern mavi
        self.active_color = (0.1, 0.3, 0.9, 1)  # Daha parlak mavi (aktif)
        with self.canvas.before:
            self.bg_color = Color(*self.default_color)
            self.bg_rect = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self.update_graphics, size=self.update_graphics, state=self.update_color)
        Rectangle(size=(60, 40), pos=(0, 0), color=(0.351, 0.351, 0.351, 1))

    def update_graphics(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size

    def update_color(self, *args):
        if self.state == 'down':
            self.bg_color.rgba = self.active_color
        else:
            self.bg_color.rgba = self.default_color

class MainScreen(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        with self.canvas.before:
            Color(0.1, 0.15, 0.25, 1)  # Koyu lacivert arka plan
            Rectangle(size=(800, 480), pos=(0, 0))
            
        # DUAL SERİAL PORT SİSTEMİ - SENSÖR VE MOTOR ARDUİNO'LARI
        self.sensor_arduino = None   # Sensör verilerini okuyan Arduino
        self.motor_arduino = None    # Motor komutlarını alan Arduino
        
        # Mac için Arduino portlarını dinamik olarak bul
        import glob
        import os
        
        # Mevcut portları tara
        usb_ports = glob.glob('/dev/tty.usb*') + glob.glob('/dev/cu.usb*') + glob.glob('/dev/tty.wch*') + glob.glob('/dev/cu.wch*')
        
        possible_ports = [
            # ÖNCE BİLİNEN PORTLAR - Motor: 120, Sensör: 110
            '/dev/cu.usbserial-11330',   # Sensör Arduino (JSON verisi)
            '/dev/tty.usbserial-110',
            '/dev/cu.usbserial-1120',   # Sensör Arduino (JSON verisi)
            '/dev/cu.usbserial-1110',   # Motor Arduino (SWITCHES/MOTORS verisi)
            '/dev/tty.usbserial-1120',
            '/dev/tty.usbserial-1110',
            # Mac USB-Serial dönüştürücü portları
            '/dev/tty.usbserial-120',    # TYPO DÜZELTİLDİ  
            '/dev/tty.usbmodem14101', 
            '/dev/cu.usbserial-120',
            '/dev/cu.usbmodem14101',
            '/dev/tty.wchusbserial14120',
            '/dev/cu.wchusbserial14120',
            # Raspberry Pi portları (geliştirme için)
            '/dev/ttyUSB0',    
            '/dev/ttyUSB1',    
            '/dev/ttyACM0',    
            '/dev/ttyACM1',    
            '/dev/ttyAMA0',    
            '/dev/serial0'
        ] + usb_ports  # Dinamik bulunan portları da ekle
        
        # Mevcut portları kontrol et
        available_ports = []
        for port in possible_ports:
            if os.path.exists(port):
                available_ports.append(port)
        
        if DEBUG:
            print(f"🔍 Toplam {len(possible_ports)} port taranacak, {len(available_ports)} port mevcut")
            print(f"📋 Mevcut portlar: {available_ports}")
        
        # ÇİFT ARDUINO SİSTEMİ - Basit port ataması
        sensor_arduino_found = False
        motor_arduino_found = False
        
        print(f"🔍 {len(available_ports)} mevcut port taranacak...")
        
        for i, port in enumerate(available_ports):
            if sensor_arduino_found and motor_arduino_found:
                break
                
            print(f"📡 Port {i+1}/{len(available_ports)}: {port}")
            
            try:
                test_port = serial.Serial(
                    port=port, 
                    baudrate=9600, 
                    timeout=1,
                    write_timeout=1
                )
                print(f"✅ Port açıldı: {port}")
                
                # Buffer temizle
                test_port.reset_input_buffer()
                test_port.reset_output_buffer()
                time.sleep(0.5)
                
                # Basit tanıma: İlk port = Sensör, İkinci port = Motor
                if not sensor_arduino_found:
                    self.sensor_arduino = test_port
                    sensor_arduino_found = True
                    print(f"📊 SENSÖR Arduino atandı: {port}")
                    
                elif not motor_arduino_found:
                    self.motor_arduino = test_port
                    motor_arduino_found = True
                    print(f"⚙️  MOTOR Arduino atandı: {port}")
                    
                else:
                    print(f"❌ Fazla port - kapatılıyor: {port}")
                    test_port.close()
                    
            except Exception as e:
                print(f"❌ Port hatası {port}: {e}")
                continue
        
        # Bağlantı durumu kontrolü
        if not self.sensor_arduino and not self.motor_arduino:
            print("❌ HİÇBİR ARDUINO BULUNAMADI!")
            print("   Mevcut portlar:", available_ports)
        elif not self.sensor_arduino:
            print("⚠️  Sensör Arduino bulunamadı!")
            if self.motor_arduino:
                print(f"   ✅ Motor Arduino bağlı: {self.motor_arduino.port}")
        elif not self.motor_arduino:
            print("⚠️  Motor Arduino bulunamadı!")
            if self.sensor_arduino:
                print(f"   ✅ Sensör Arduino bağlı: {self.sensor_arduino.port}")
        else:
            print(f"✅ Çift Arduino sistemi hazır:")
            print(f"   📊 Sensör: {self.sensor_arduino.port}")
            print(f"   ⚙️  Motor: {self.motor_arduino.port}") 
        
        # Eski serial_port değişkenini geriye uyumluluk için tut
        self.serial_port = self.sensor_arduino
        
        with self.canvas.before:
            Color(0.05, 0.1, 0.2, 1)  # Daha koyu mavi tonları
            Rectangle(pos=(0, 240), size=(800, 230))
            Rectangle(pos=(620, 0), size=(180, 440))
            Rectangle(pos=(0, 0), size=(120, 440))
            Rectangle(pos=(0, 0), size=(800, 90), color=(0.2, 0.4, 0.8, 1))  # Mavi başlık



        # BUFFER SİSTEMİ - Çift Arduino için ayrı buffer'lar
        self.sensor_data_buffer = deque(maxlen=50)  # Sensör veriler için
        self.motor_data_buffer = deque(maxlen=20)   # Motor feedback için
        self.serial_thread_running = True
        
        # BAĞLANTI DURUMU KONTROLÜ
        self.last_connection_warning = 0  # Son uyarı zamanı
        self.connection_warning_interval = 15  # 15 saniye
        
        # AXIS LOCK DURUMU - her eksen için lock durumunu takip et
        self.axis_locked = {'X': False, 'Y': False, 'Z': False}
        
        # ADMIN SWITCH DURUMU - Admin modu açık/kapalı
        self.admin_mode_active = True  # Başlangıçta aktif kabul et
        
        # FN KEY DURUMU - Fn tuşuna basılı tutulup tutulmadığını takip eder
        self.fn_key_pressed = False
        
        # MADGWICK FILTER - Python'da işle
        self.madgwick_filter = MadgwickFilter(beta=0.5)  # Daha hızlı tepki
        self.last_update_time = time.time()
        
        # Accelerometer max değerler için değişkenler
        self.max_accel_x = 0
        self.max_accel_y = 0
        self.max_accel_z = 0
        self.max_detector_x = 0
        self.max_detector_y = 0
        self.max_detector_z = 0

        # UI ELEMANLARI AYNI KALIYOR...
        YBP = 340  # Y Button Position
        XBP = 120   # X Button Position
        self.add_widget(ArrowButton(direction='up', label_text='Y+', pos=(XBP, YBP + 60)))
        self.add_widget(ArrowButton(direction='down', label_text='Y-', pos=(XBP, YBP - 60)))
        self.add_widget(ArrowButton(direction='left', label_text='X-', pos=(XBP-60, YBP)))
        self.add_widget(ArrowButton(direction='right', label_text='X+', pos=(XBP+60, YBP)))
        self.add_widget(ArrowButton(direction='up', label_text='Z+', pos=(XBP + 440, YBP + 60)))
        self.add_widget(ArrowButton(direction='down', label_text='Z-', pos=(XBP + 440   , YBP - 60)))

        # Su terazisi
        self.terazi = GyroDisplay(size_hint=(None, None), size=(150, 150), pos=(310, 295))
        self.add_widget(self.terazi)

        RIBP = 650
        
        # FN BUTTON - Sol alt köşede
        self.fn_button = ColoredToggleButton(
            text='Fn',
            size=(80, 50),
            pos=(20, 20),  # Sol alt köşe
            font_size=20,
            group=None  # Grup dışı - bağımsız toggle
        )
        self.fn_button.bind(state=self.on_fn_toggle)
        self.add_widget(self.fn_button)
        
        # FN STATUS LABEL
        self.fn_status_label = Label(
            text='Fn: OFF',
            font_size=14,
            font_name='RobotoMono-Regular',
            pos=(20, 75),
            size_hint=(None, None),
            size=(80, 20),
            color=(1, 0.5, 0.5, 1),
            halign='center',
            text_size=(80, 20)
        )
        self.add_widget(self.fn_status_label)
        
        # ADMIN MODE STATUS LABEL - Ekranın üst kısmında
        self.admin_status_label = Label(
            text='🔓 OPERATOR MODE',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(300, 450),
            size_hint=(None, None),
            size=(200, 30),
            color=(0.5, 1, 0.5, 1),  # Yeşil - aktif
            halign='center',
            text_size=(200, 30)
        )
        self.add_widget(self.admin_status_label)

        
        # PROTECTED BUTTONS - Fn ile korumalı butonlar
        self.calibrate_btn = Button(
            text='Calibrate\nGyro',
            font_size=16,
            pos=(RIBP, 10),
            size_hint=(None, None),
            size=(100, 50),
            background_color=(0.6, 0.6, 0.6, 1),  # Başlangıçta gri (disabled)
            disabled=True
        )
        self.calibrate_btn.bind(on_press=self.calibrate_gyro)
        self.add_widget(self.calibrate_btn)

        self.reset_max_btn = Button(
            text='Reset\nMax Values',
            font_size=15,
            pos=(RIBP, 58),
            size_hint=(None, None),
            size=(100, 50),
            background_color=(0.6, 0.6, 0.6, 1),  # Başlangıçta gri (disabled)
            disabled=True
        )
        self.reset_max_btn.bind(on_press=self.reset_max_values)
        self.add_widget(self.reset_max_btn)

   

        # ÜSTTE SİYAH ALANDA - Ana sensör bilgileri (y: 280-480)
        Sensor_X = 130 # sensor X ekseni posisyonu
        Sensor_Y = 200 # sensor Y ekseni posisyonu
        Platform_X = 420 # platform X ekseni posisyonu (450'den 420'ye)
        Platform_Y = 200 # platform Y ekseni posisyonu

        self.temp_label = Label(
            text='TEMPERATURE: --°C',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Sensor_X, Sensor_Y),  # Sol üst köşe
            size_hint=(None, None), 
            size=(250, 25), 
            color=(1, 1, 1, 1),
            halign='left', 
            text_size=(250, 25)
        )
        self.add_widget(self.temp_label)

        self.humidity_label = Label(
            text='HUMIDITY: --%',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Sensor_X, Sensor_Y - 25),  # Sol üst
            size_hint=(None, None), 
            size=(250, 25), 
            color=(1, 1, 1, 1),
            halign='left', 
            text_size=(250, 25)
        )
        self.add_widget(self.humidity_label)

        self.distance_label = Label(
            text='DISTANCE: --cm',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Sensor_X, Sensor_Y - 50),  # Sol üst
            size_hint=(None, None), 
            size=(250, 25), 
            color=(1, 1, 1, 1),
            halign='left', 
            text_size=(250, 25)
        )
        self.add_widget(self.distance_label)

        # SOL GRİ ALANDA - Motor durumu ve joystick bilgileri (x: 0-120, y: 0-440)
        Motor_Info_X = 10   # Motor bilgileri X pozisyonu
        Motor_Info_Y = 400  # Motor bilgileri Y pozisyonu (alttan başla)

        # Motor durumu başlığı
        self.add_widget(Label(
            text='MOTOR STATUS', 
            font_size=16,
            font_name='RobotoMono-Regular',
            pos=(Motor_Info_X, Motor_Info_Y),
            size_hint=(None, None), 
            size=(110, 20),
            color=(1, 1, 1, 1),
            halign='center',
            text_size=(110, 20)
        ))

        # Fren switch durumları
        self.brake_switches_label = Label(
            text='Fren: X✓ Y✓ Z✓', 
            font_size=12,
            font_name='RobotoMono-Regular',
            pos=(Motor_Info_X, Motor_Info_Y - 25),
            size_hint=(None, None), 
            size=(110, 20),
            color=(1, 1, 0.5, 1),  # Sarı
            halign='center',
            text_size=(110, 20)
        )
        self.add_widget(self.brake_switches_label)

        # Motor enable durumları
        self.motor_enable_label = Label(
            text='Motor: X- Y- Z-', 
            font_size=12,
            font_name='RobotoMono-Regular',
            pos=(Motor_Info_X, Motor_Info_Y - 45),
            size_hint=(None, None), 
            size=(110, 20),
            color=(0.8, 0.8, 0.8, 1),  # Açık gri
            halign='center',
            text_size=(110, 20)
        )
        self.add_widget(self.motor_enable_label)

        # Joystick değerleri
        self.joystick_label = Label(
            text='Joy: x=512 y=512', 
            font_size=12,
            font_name='RobotoMono-Regular',
            pos=(Motor_Info_X, Motor_Info_Y - 65),
            size_hint=(None, None), 
            size=(110, 20),
            color=(0.5, 1, 0.8, 1),  # Açık yeşil
            halign='center',
            text_size=(110, 20)
        )
        self.add_widget(self.joystick_label)

        # PWM Speed göstergesi
        self.pwm_speed_label = Label(
            text='PWM: 80', 
            font_size=12,
            font_name='RobotoMono-Regular',
            pos=(Motor_Info_X, Motor_Info_Y - 85),
            size_hint=(None, None), 
            size=(110, 20),
            color=(1, 0.8, 0.5, 1),  # Turuncu
            halign='center',
            text_size=(110, 20)
        )
        self.add_widget(self.pwm_speed_label)
        self.add_widget(Label(
            text='ACCELEROMETER', 
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Platform_X, Platform_Y),
            size_hint=(None, None), 
            size=(200, 25),
            color=(1, 1, 1, 1),
            halign='center',
            text_size=(200, 25)
        ))

        self.platform_label = Label(
            text='pX=0  pY=0  pZ=0', 
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Platform_X, Platform_Y - 25),
            size_hint=(None, None), 
            size=(280, 25),  # Genişlik 200→280 (alan büyütüldü)
            color=(1, 1, 1, 1),
            halign='center',
            valign='middle'
        )
        self.add_widget(self.platform_label)

        self.detector_label = Label(
            text='dX=0  dY=0  dZ=0', 
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Platform_X, Platform_Y - 50),
            size_hint=(None, None), 
            size=(280, 25),  # Genişlik 200→280 (alan büyütüldü)
            color=(1, 1, 1, 1),
            halign='center',
            valign='middle'
        )
        self.add_widget(self.detector_label)

        # ORTA ALANDA - Max değerler (y: 60-280 arasında boş alan)
        self.max_platform_label = Label(
            text='MAX: pX=0  pY=0  pZ=0',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Platform_X - 35, Platform_Y - 70),
            size_hint=(None, None),
            size=(320, 25),  # Genişlik 250→320 (alan büyütüldü)
            color=(1, 1, 0, 1),
            halign='center',
            valign='middle'
        )
        self.add_widget(self.max_platform_label)

        self.max_detector_label = Label(
            text='MAX: dX=0  dY=0  dZ=0',
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(Platform_X - 35, Platform_Y - 90),
            size_hint=(None, None),
            size=(320, 25),  # Genişlik 250→320 (alan büyütüldü)
            color=(1, 1, 0, 1),
            halign='center',
            valign='middle'
        )
        self.add_widget(self.max_detector_label)

        # Speed label - orta alanda, butonların üstünde
        speed_label = Label(
            text="Speed: MIN",
            font_size=18,
            font_name='RobotoMono-Regular',
            pos=(330, 70),  # Butonların üstünde
            size_hint=(None, None),
            size=(140, 10),
            color=(1, 1, 1, 1),
            halign='center',
            text_size=(140, 30)
        )  
        self.speed_label = speed_label
        self.add_widget(speed_label)

        # 5 adet hız butonu - ekranın ortasında dizilmiş
        self.mult_buttons = []
        speed_options = [("MİN", 0), ("%25", 1), ("%50", 2), ("%75", 3), ("MAX", 4)]
        
        # Butonları ortada konumlandırmak için başlangıç pozisyonu
        total_width = 5 * 60 + 4 * 10  # 5 buton * 60px genişlik + 4 aralık * 10px
        start_x = (800 - total_width) // 2  # Ekranın ortasında başla
        
        for idx, (label, _) in enumerate(speed_options):
            btn = ColoredToggleButton(
                text=label,
                size=(60, 40),
                pos=(start_x + idx * 70, 10),  # Y pozisyonunu da ortaya getir
                font_size=20,
                group="mult"
            )
            btn.bind(on_press=self.on_mult_button)
            self.mult_buttons.append(btn)
            self.add_widget(btn)
        self.mult_buttons[0].state = 'down'

        # PROTECTED AXIS BUTTONS - Fn ile korumalı unlock butonları
        axis_labels = ['Unlock X', 'Unlock Y', 'Unlock Z']
        self.axis_buttons = []
        for i, label in enumerate(axis_labels):
            axis_btn = ColoredToggleButton(
                text=label,
                size=(100, 40),
                pos=(RIBP, 260 + i * 80),
                font_size=18,
                disabled=True  # Başlangıçta disabled
            )
            # Disabled görünümü için gri renk
            axis_btn.default_color = (0.6, 0.6, 0.6, 1)
            axis_btn.active_color = (0.6, 0.6, 0.6, 1)
            axis_btn.background_color = (0.6, 0.6, 0.6, 1)
            axis_btn.bind(state=self.on_axis_toggle)
            self.axis_buttons.append(axis_btn)
            self.add_widget(axis_btn)

        self.lock_status_labels = []
        axis_names = ['X', 'Y', 'Z']
        for i, axis in enumerate(axis_names):
            lbl = Label(
                text=f"{axis}: unlocked",
                font_size=16,
                font_name='RobotoMono-Regular',
                pos=(RIBP, 238 + i * 80),
                size_hint=(None, None),
                size=(100, 30),
                color=(1, 1, 1, 1),
                halign='center',
                text_size=(100, 30)
            )
            self.lock_status_labels.append(lbl)
            self.add_widget(lbl)

        # ÇİFT THREAD İLE SERİAL OKUMA BAŞLAT
        if self.sensor_arduino:
            print(f"🚀 Sensör Arduino okuma thread'i başlatılıyor... Port: {self.sensor_arduino.port}")
            self.sensor_thread = threading.Thread(target=self.sensor_read_thread, daemon=True)
            self.sensor_thread.start()
            print(f"✅ Sensör thread aktif - Thread ID: {self.sensor_thread.ident}")
        else:
            
            print("❌ Sensör thread başlatılamadı - port bulunamadı!")
            print(f"   Motor Arduino var mı? {self.motor_arduino is not None}")
            
        if self.motor_arduino:
            print("🚀 Motor Arduino okuma thread'i başlatılıyor...")
            self.motor_thread = threading.Thread(target=self.motor_read_thread, daemon=True)
            self.motor_thread.start()
            print("✅ Motor thread aktif")
        else:
            print("❌ Motor thread başlatılamadı - port bulunamadı")
        
        # VERİ İŞLEME TIMER'I - Daha hızlı (60 FPS)
        print("⏰ Çift veri işleme timer'ı başlatılıyor (60 FPS)...")
        Clock.schedule_interval(self.process_serial_data, 1/60)  # 30 FPS'den 60 FPS'ye artırıldı
        
        # Arduino'lara test komutları gönder
        Clock.schedule_once(lambda dt: self.send_test_commands(), 2.0)
        
        # Motor durum sorgusu ve enable - 3 saniye sonra
        Clock.schedule_once(lambda dt: self.enable_all_motors(), 3.0)
    
    def send_test_commands(self):
        """Arduino'lara test komutları gönder"""
        if self.sensor_arduino:
            try:
                self.sensor_arduino.write(b'TEST\n')
                self.sensor_arduino.flush()
                print("📡 Sensör Arduino'ya TEST komutu gönderildi")
            except Exception as e:
                print(f"❌ Sensör TEST komutu hatası: {e}")
        
        if self.motor_arduino:
            try:
                self.motor_arduino.write(b'TEST\n')
                self.motor_arduino.flush()
                print("📡 Motor Arduino'ya TEST komutu gönderildi")
            except Exception as e:
                print(f"❌ Motor TEST komutu hatası: {e}")

    def sensor_read_thread(self):
        """THREAD: Sürekli sensör Arduino'sundan veri oku - JSON formatı için optimize edildi"""
        buffer = ""
        consecutive_errors = 0
        last_data_time = time.time()
        
        print("🔄 Sensör Arduino okuma thread'i başlatıldı...")
        
        while self.serial_thread_running:
            try:
                if self.sensor_arduino and self.sensor_arduino.in_waiting:
                    chunk = self.sensor_arduino.read(self.sensor_arduino.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    consecutive_errors = 0
                    last_data_time = time.time()
                    
                    # Sessiz veri alımı - sadece kritik durumlar için log
                    
                    # JSON parçalarını birleştirme algoritması
                    while True:
                        # JSON başlangıcını bul
                        start_idx = buffer.find('{')
                        if start_idx == -1:
                            # JSON başlangıcı yok, buffer'ı temizle
                            if buffer.strip():
                                print(f"📝 JSON olmayan veri atılıyor: {buffer.strip()[:100]}...")
                            buffer = ""
                            break
                        
                        # JSON başlangıcından önceki kısmı at
                        if start_idx > 0:
                            discarded = buffer[:start_idx]
                            if discarded.strip():
                                print(f"📝 Sensör - Atılan veri: {discarded.strip()[:100]}...")
                            buffer = buffer[start_idx:]
                        
                        # JSON sonunu bul - parantez sayımı ile
                        brace_count = 0
                        end_idx = -1
                        in_string = False
                        escape_next = False
                        
                        for i, char in enumerate(buffer):
                            if escape_next:
                                escape_next = False
                                continue
                            
                            if char == '\\':
                                escape_next = True
                                continue
                                
                            if char == '"' and not escape_next:
                                in_string = not in_string
                                continue
                                
                            if not in_string:
                                if char == '{':
                                    brace_count += 1
                                elif char == '}':
                                    brace_count -= 1
                                    if brace_count == 0:
                                        end_idx = i
                                        break
                        
                        if end_idx == -1:
                            # Tam JSON bulunamadı, daha fazla veri bekle
                            print(f"🔄 Eksik JSON - buffer'da {len(buffer)} byte var, daha fazla veri bekleniyor...")
                            
                            # Buffer çok büyükse temizle (memory koruması)
                            if len(buffer) > 512:
                                print(f"🗑️  Buffer çok büyük ({len(buffer)} byte), temizleniyor...")
                                buffer = ""
                            break
                        
                        # Tam JSON'ı çıkar
                        json_str = buffer[:end_idx + 1]
                        buffer = buffer[end_idx + 1:]
                        
                        print(f"🔍 JSON adayı bulundu: {len(json_str)} byte")
                        
                        # JSON'ı doğrula ve işle
                        try:
                            # Yaygın JSON hatalarını düzelt
                            cleaned_json = json_str
                            
                            # Kontrol karakterlerini temizle (CR, LF vs.)
                            import re
                            # ASCII kontrol karakterlerini kaldır (0-31 arası, 127)
                            cleaned_json = re.sub(r'[\x00-\x1F\x7F]', '', cleaned_json)
                            # Ekstra boşlukları temizle
                            cleaned_json = re.sub(r'\s+', ' ', cleaned_json)
                            cleaned_json = cleaned_json.strip()
                            
                            # Bozuk Madgwick alanını düzelt
                            if 'null:{"roll"' in cleaned_json:
                                cleaned_json = cleaned_json.replace('null:{"roll"', '"madgwick":{"roll"')
                                print("🔧 null: hatası düzeltildi")
                            
                            # Eksik değerleri düzelt
                            if '"temperature_c":n' in cleaned_json:
                                cleaned_json = cleaned_json.replace('"temperature_c":n', '"temperature_c":null')
                                print("🔧 temperature_c hatası düzeltildi")
                                
                            if '"humidity":nul' in cleaned_json:
                                cleaned_json = cleaned_json.replace('"humidity":nul', '"humidity":null')
                                print("🔧 humidity hatası düzeltildi")
                            
                            # Eksik virgülleri ekle (basit regex ile)
                            import re
                            cleaned_json = re.sub(r'(\d+)\s*"', r'\1,"', cleaned_json)
                            cleaned_json = re.sub(r'}\s*"', r'},"', cleaned_json)
                            
                            # JSON'ı test et
                            test_json = json.loads(cleaned_json)
                            
                            # Başarılı JSON'ı buffer'a ekle
                            self.sensor_data_buffer.append(cleaned_json)
                            
                            # Her 100 veri de bir debug yazdır (çok sık olmasın)
                            if hasattr(self, 'sensor_data_count'):
                                self.sensor_data_count += 1
                            else:
                                self.sensor_data_count = 1
                                
                            if self.sensor_data_count % 50 == 0:
                                print(f"📊 Sensör veri sayısı: {self.sensor_data_count}")
                                print(f"📦 Buffer durumu: Sensör={len(self.sensor_data_buffer)}, Motor={len(self.motor_data_buffer)}")
                                print(f"🔑 JSON anahtarları: {list(test_json.keys())}")
                            
                        except json.JSONDecodeError as e:
                            print(f"❌ JSON decode hatası: {e}")
                            print(f"📄 Sorunlu JSON (ilk 200 karakter): {json_str[:200]}...")
                            
                            # Hata durumunda buffer'ı temizle
                            buffer = ""
                            continue
                        except Exception as e:
                            print(f"❌ JSON işleme hatası: {e}")
                            continue
                                    
                    time.sleep(0.01)  # 10ms bekleme
                else:
                    # Sessizce bekle - ana thread'de 15 saniyede bir uyarı verilecek
                    time.sleep(1)
                    
            except Exception as e:
                consecutive_errors += 1
                print(f"❌ Sensör okuma hatası #{consecutive_errors}: {e}")
                
                if consecutive_errors > 10:  # 5'ten 10'a artırıldı
                    print("🔄 Çok fazla sensör hatası - bağlantı kopmuş olabilir")
                    self.handle_arduino_disconnection('sensor')
                    break
                else:
                    time.sleep(0.1)

    def motor_read_thread(self):
        """THREAD: Motor Arduino'sundan feedback verilerini oku"""
        buffer = ""
        consecutive_errors = 0
        
        print("🔄 Motor Arduino okuma thread'i başlatıldı...")
        
        while self.serial_thread_running:
            try:
                if self.motor_arduino and self.motor_arduino.in_waiting:
                    chunk = self.motor_arduino.read(self.motor_arduino.in_waiting).decode('utf-8', errors='ignore')
                    buffer += chunk
                    consecutive_errors = 0
                    
                    # Satır satır işle (motor feedback genellikle düz metin)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.motor_data_buffer.append(line)
                            
                            # Motor debug'ı sadece önemli mesajlarda
                            if hasattr(self, 'motor_feedback_count'):
                                self.motor_feedback_count += 1
                            else:
                                self.motor_feedback_count = 1
                                
                            # Her 50 feedback'de bir ya da önemli durumlarda göster
                            show_debug = (self.motor_feedback_count % 50 == 0 or 
                                        'LED_TEST' in line or 'MOTOR_STATUS' in line or 
                                        'admin=1' in line)
                            
                            if show_debug:
                                print(f"⚙️  Motor feedback #{self.motor_feedback_count}: {line}")
                            
                            # LED test durumu kontrolü
                            if 'LED_TEST' in line:
                                print(f"💡 Motor Arduino LED test modunda")
                            elif 'MOTOR_STATUS' in line:
                                print(f"🔧 Motor durumu: {line}")
                            elif 'admin=1' in line:
                                print(f"🔑 Admin aktif tespit edildi!")
                                
                elif self.motor_arduino:
                    import time
                    time.sleep(0.01)  # 10ms bekleme
                else:
                    # Sessizce bekle - ana thread'de 15 saniyede bir uyarı verilecek
                    import time
                    time.sleep(1)
                    
            except Exception as e:
                consecutive_errors += 1
                print(f"❌ Motor okuma hatası #{consecutive_errors}: {e}")
                
                if consecutive_errors > 5:
                    print("🔄 Çok fazla motor hatası - bağlantı kopmuş olabilir")
                    self.handle_arduino_disconnection('motor')
                    break
                else:
                    import time
                    time.sleep(0.1)

    def process_serial_data(self, dt):
        """Ana thread: Her iki Arduino'dan gelen verileri işle"""
        processed_count = 0
        max_per_frame = 3  # Frame başına maksimum işlem (5'ten 3'e düşürüldü)

        # SENSÖR VERİLERİNİ İŞLE
        while self.sensor_data_buffer and processed_count < max_per_frame:
            try:
                line = self.sensor_data_buffer.popleft()
                # Sessiz işlem - sadece hata durumunda log
                
                # JSON parse et
                data = json.loads(line)

                # Debug: Ana veri tiplerini göster
                main_keys = [k for k in data.keys() if k not in ['quaternion']]
                print(f"📊 Sensör Arduino verisi anahtarları: {main_keys}")
                
                # Veri içeriğini kontrol et
                has_temp = 'temperature_c' in data and data['temperature_c'] is not None
                has_humidity = 'humidity' in data and data['humidity'] is not None
                has_distance = 'distance_cm' in data and data['distance_cm'] is not None
                has_raw_sensors = 'raw_sensors' in data and data['raw_sensors'] is not None
                has_adxl345 = 'adxl345' in data and data['adxl345'] is not None
                
                print(f"📈 Veri içeriği: Temp={has_temp}, Hum={has_humidity}, Dist={has_distance}, Raw={has_raw_sensors}, ADXL={has_adxl345}")
                
                # Tüm sensör verilerini tek seferde güncelle
                self.update_sensor_data(data)
                processed_count += 1
                
                print(f"✅ Sensör verisi başarıyla işlendi")
                
            except json.JSONDecodeError as e:
                print(f"❌ Sensör JSON decode hatası: {e}")
                print(f"📄 Sorunlu veri: {line[:200] if len(line) > 200 else line}")
                processed_count += 1
                continue
            except Exception as e:
                print(f"❌ Sensör veri işleme hatası: {e}")
                print(f"📄 Hata verisi: {line[:100] if len(line) > 100 else line}")
                processed_count += 1

        # MOTOR FEEDBACKLERİNİ İŞLE
        motor_processed = 0
        while self.motor_data_buffer and motor_processed < 2:  # Motor feedback daha az sıklıkta (3'ten 2'ye)
            try:
                line = self.motor_data_buffer.popleft()
                print(f"🔧 Motor feedback işleniyor: {line}")
                self.process_motor_feedback(line)
                motor_processed += 1
                
            except Exception as e:
                print(f"❌ Motor feedback işleme hatası: {e}")
                motor_processed += 1
        
        # Buffer durumu bilgisi - sadece kritik durumlarda
        if len(self.sensor_data_buffer) > 40:  # %80 dolmuşsa uyar
            print(f"⚠️  Sensör buffer kritik: {len(self.sensor_data_buffer)}/50")
        if len(self.motor_data_buffer) > 15:  # %75 dolmuşsa uyar
            print(f"⚠️  Motor buffer kritik: {len(self.motor_data_buffer)}/20")
            
        # Bağlantı durumu kontrolü - 15 saniyede bir
        import time
        current_time = time.time()
        if current_time - self.last_connection_warning > self.connection_warning_interval:
            if not self.sensor_arduino and not self.motor_arduino:
                print(f"❌ HİÇBİR ARDUINO BAĞLI DEĞİL!")
                self.last_connection_warning = current_time
            elif not self.motor_arduino:
                print(f"❌ MOTOR ARDUINO BAĞLI DEĞİL - MOTORLAR ÇALIŞMAZ!")
                self.last_connection_warning = current_time
            elif not self.sensor_arduino:
                print(f"❌ SENSÖR ARDUINO BAĞLI DEĞİL!")
                self.last_connection_warning = current_time

    def process_motor_feedback(self, feedback_line):
        """Motor Arduino'sundan gelen feedback mesajlarını işle"""
        print(f"⚙️  Motor feedback işleniyor: {feedback_line}")
        
        # Switch durumları (Admin kontrolü kaldırıldı)
        if feedback_line.startswith('SWITCHES:'):
            try:
                switches_data = feedback_line.replace('SWITCHES:', '')
                switches = {}
                for item in switches_data.split(','):
                    key, value = item.split('=')
                    switches[key] = value == '1'
                
                # Fren switch durumları güncelle
                x_brake = "✓" if switches.get('xFren', False) else "✗"
                y_brake = "✓" if switches.get('yFren', False) else "✗"
                z_brake = "✓" if switches.get('zFren', False) else "✗"
                self.brake_switches_label.text = f'Fren: X{x_brake} Y{y_brake} Z{z_brake}'
                
            except Exception as e:
                print(f"❌ Switch data parse hatası: {e}")
        
        # Motor enable durumları
        elif feedback_line.startswith('MOTORS:'):
            try:
                # Format: MOTORS:xEna=1,yEna=0,zEna=0
                motors_data = feedback_line.replace('MOTORS:', '')
                motors = {}
                for item in motors_data.split(','):
                    key, value = item.split('=')
                    motors[key] = value == '1'
                
                # Motor enable durumları güncelle
                x_motor = "+" if motors.get('xEna', False) else "-"
                y_motor = "+" if motors.get('yEna', False) else "-"
                z_motor = "+" if motors.get('zEna', False) else "-"
                self.motor_enable_label.text = f'Motor: X{x_motor} Y{y_motor} Z{z_motor}'
                
                print(f"🔧 Motor durumları güncellendi")
                
            except Exception as e:
                print(f"❌ Motor data parse hatası: {e}")
        
        # Joystick değerleri
        elif feedback_line.startswith('JOYSTICK:'):
            try:
                # Format: JOYSTICK:x=523,y=501
                joystick_data = feedback_line.replace('JOYSTICK:', '')
                joystick = {}
                for item in joystick_data.split(','):
                    key, value = item.split('=')
                    joystick[key] = int(value)
                
                x_val = joystick.get('x', 512)
                y_val = joystick.get('y', 512)
                self.joystick_label.text = f'Joy: x={x_val} y={y_val}'
                
                # Joystick değerlerini su terazisinde de gösterebiliriz (opsiyonel)
                # Merkez: 512, deadzone: 100
                x_centered = x_val - 512
                y_centered = y_val - 512
                if abs(x_centered) > 100 or abs(y_centered) > 100:
                    print(f"�️  Joystick aktif: X={x_centered}, Y={y_centered}")
                
            except Exception as e:
                print(f"❌ Joystick data parse hatası: {e}")
        
        # PWM hız değeri
        elif feedback_line.startswith('PWM_SPEED:'):
            try:
                pwm_value = int(feedback_line.replace('PWM_SPEED:', ''))
                
                # PWM değerine göre hız seviyesini belirle
                if pwm_value <= 80:
                    speed_level = "MIN"
                elif pwm_value <= 120:
                    speed_level = "25%"
                elif pwm_value <= 160:
                    speed_level = "50%"
                elif pwm_value <= 200:
                    speed_level = "75%"
                else:
                    speed_level = "MAX"
                
                # PWM değerini yüzde olarak hesapla
                pwm_percent = int((pwm_value / 255) * 100)
                
                self.pwm_speed_label.text = f'{speed_level}: PWM {pwm_value} ({pwm_percent}%)'
                print(f"🔥 PWM HIZ DEĞİŞTİ: {speed_level} - PWM {pwm_value}/255 ({pwm_percent}%)")
                print(f"🔥 LED parlaklığı şimdi {pwm_percent}% olmalı!")
                
            except Exception as e:
                print(f"❌ PWM speed parse hatası: {e}")
        
        # Motor hareket onayları
        elif 'MOTOR_MOVED_' in feedback_line:
            print(f"✅ Motor hareketi onaylandı: {feedback_line}")
        
        # Motor disabled uyarısı
        elif 'MOTOR_DISABLED_' in feedback_line:
            axis = feedback_line.split('_')[-1]
            print(f"❌ MOTOR DİSABLED: {axis} ekseni disabled - enable yapılması gerekiyor!")
        
        # Motor enable/disable onayları
        elif '_ENABLED' in feedback_line or '_DISABLED' in feedback_line:
            print(f"🔧 Motor enable durumu: {feedback_line}")
        
        # Hız değişikliği onayı
        elif 'SPEED_SET' in feedback_line:
            print(f"⚡ Hız ayarlandı: {feedback_line}")
        
        # Axis lock/unlock onayı
        elif 'AXIS_LOCKED' in feedback_line or 'AXIS_UNLOCKED' in feedback_line:
            print(f"🔒 Axis durumu: {feedback_line}")
        
        # Motor durdurma onayı
        elif 'MOTOR_STOPPED' in feedback_line:
            print(f"⏹️  Motor durduruldu: {feedback_line}")
        
        # Fren aktif uyarısı
        elif 'BRAKE_ACTIVE' in feedback_line:
            print(f"🛑 Fren aktif: {feedback_line}")
        
        # Admin switch kapalı uyarısı
        elif 'ADMIN_SWITCH_OFF' in feedback_line:
            print(f"⚠️  Admin switch kapalı!")
        
        # Admin modu açık/kapalı mesajları - Arduino'dan gelen
        elif 'admin modu açık' in feedback_line.lower():
            self.admin_mode_active = True
            self.admin_status_label.text = '🔓 OPERATOR MODE'
            self.admin_status_label.color = (0.5, 1, 0.5, 1)  # Yeşil
            print(f"✅ ADMIN MODU AÇIK - Touchscreen aktif")
        
        elif 'admin modu kapalı' in feedback_line.lower():
            self.admin_mode_active = False
            self.admin_status_label.text = '🔒 VIEW MODE'
            self.admin_status_label.color = (1, 0.3, 0.3, 1)  # Kırmızı
            print(f"🔒 ADMIN MODU KAPALI - Touchscreen salt okunur")
        
        # Test yanıtı
        elif 'MOTOR_ARDUINO_TEST_OK' in feedback_line:
            print(f"✅ Motor Arduino test başarılı")
        
        # Genel feedback
        else:
            print(f"📢 Motor genel feedback: {feedback_line}")

    def update_sensor_data(self, data):
        """Arduino'dan gelen birleşik JSON verisini işle"""
        try:
            if DEBUG:
                print(f"🔄 Arduino verisi işleniyor: {list(data.keys())}")
                
                # Debug: None değerleri tespit et
                none_values = []
                for key, value in data.items():
                    if value is None:
                        none_values.append(key)
                    elif isinstance(value, dict):
                        for sub_key, sub_value in value.items():
                            if sub_value is None:
                                none_values.append(f"{key}.{sub_key}")
                
                if none_values:
                    print(f"⚠️  None değerler tespit edildi: {none_values}")
                
                # Arduino'dan gelen tüm veri tiplerini kontrol et
                print(f"🔍 Detaylı veri analizi:")
                for key, value in data.items():
                    if isinstance(value, dict):
                        print(f"   {key}: {value}")
                    else:
                        print(f"   {key}: {value} (type: {type(value)})")
            
            # DHT22 sensörleri - sıcaklık ve nem (None kontrolü ile)
            if 'temperature_c' in data:
                temp_val = data['temperature_c']
                if DEBUG:
                    print(f"🌡️  Temperature verisi: {temp_val} (type: {type(temp_val)})")
                if temp_val is not None:
                    try:
                        temp_value = float(temp_val)
                        temp_text = f"TEMPERATURE: {temp_value:.1f}°C"
                        self.temp_label.text = temp_text
                        if DEBUG:
                            print(f"✅ Sıcaklık güncellendi: {temp_text}")
                    except (ValueError, TypeError) as e:
                        self.temp_label.text = "TEMPERATURE: --°C"
                        if DEBUG:
                            print(f"⚠️  Geçersiz sıcaklık verisi: {temp_val}, error: {e}")
                else:
                    self.temp_label.text = "TEMPERATURE: --°C"
                    if DEBUG:
                        print(f"⚠️  Sıcaklık verisi None")
            else:
                if DEBUG:
                    print(f"❌ Temperature verisi JSON'da yok!")
                
            if 'humidity' in data:
                hum_val = data['humidity']
                if DEBUG:
                    print(f"💧 Humidity verisi: {hum_val} (type: {type(hum_val)})")
                if hum_val is not None:
                    try:
                        hum_value = float(hum_val)
                        hum_text = f"HUMIDITY: {hum_value:.1f}%"
                        self.humidity_label.text = hum_text
                        if DEBUG:
                            print(f"✅ Nem güncellendi: {hum_text}")
                    except (ValueError, TypeError) as e:
                        self.humidity_label.text = "HUMIDITY: --%"
                        if DEBUG:
                            print(f"⚠️  Geçersiz nem verisi: {hum_val}, error: {e}")
                else:
                    self.humidity_label.text = "HUMIDITY: --%"
                    if DEBUG:
                        print(f"⚠️  Nem verisi None")
            else:
                if DEBUG:
                    print(f"❌ Humidity verisi JSON'da yok!")
            
            # Ultrasonik sensör - mesafe (None kontrolü ile)
            if 'distance_cm' in data and data['distance_cm'] is not None:
                try:
                    dist_value = float(data['distance_cm'])

                    if dist_value > 400:  # Ultrasonik sensör max menzil kontrolü
                        dist_text = "DISTANCE: >400 cm"
                    else:
                        dist_text = f"DISTANCE: {dist_value:.1f} cm"
                    self.distance_label.text = dist_text
                    if DEBUG:
                        print(f"📏 Mesafe güncellendi: {dist_text}")
                except (ValueError, TypeError):
                    self.distance_label.text = "DISTANCE: --cm"
                    if DEBUG:
                        print(f"⚠️  Geçersiz mesafe verisi: {data['distance_cm']}")
            elif 'distance_cm' in data:
                self.distance_label.text = "DISTANCE: --cm"
                if DEBUG:
                    print(f"⚠️  Mesafe verisi None")
            
            # Ham sensör verileri - Python Madgwick Filter'dan geçir
            if 'raw_sensors' in data and data['raw_sensors'] is not None:
                raw = data['raw_sensors']
                if DEBUG:
                    print(f"📊 Raw sensors verisi: {raw}")
                
                # Ham sensör değerlerini topla
                gyro_data = raw.get('gyro', {})
                accel_data = raw.get('accel', {})
                mag_data = raw.get('mag', {})  # Magnetometer verisi varsa
                
                if gyro_data and accel_data:
                    try:
                        # Gyroscope verileri (rad/s'ye çevir)
                        gx = math.radians(float(gyro_data.get('x', 0) or 0))
                        gy = math.radians(float(gyro_data.get('y', 0) or 0))
                        gz = math.radians(float(gyro_data.get('z', 0) or 0))
                        
                        # Accelerometer verileri
                        ax = float(accel_data.get('x', 0) or 0)
                        ay = float(accel_data.get('y', 0) or 0)
                        az = float(accel_data.get('z', 0) or 0)
                        
                        # Magnetometer verileri (yoksa sıfır)
                        mx = float(mag_data.get('x', 0) or 0) if mag_data else 0
                        my = float(mag_data.get('y', 0) or 0) if mag_data else 0
                        mz = float(mag_data.get('z', 0) or 0) if mag_data else 0
                        
                        # Zaman farkını hesapla
                        current_time = time.time()
                        dt = current_time - self.last_update_time
                        self.last_update_time = current_time
                        
                        # Madgwick filter güncelle
                        if dt > 0 and dt < 1.0:  # Makul zaman aralığı
                            self.madgwick_filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt)
                            
                            # Euler açılarını al
                            roll, pitch, yaw = self.madgwick_filter.get_euler_angles()
                            
                            # Su terazisini güncelle
                            self.terazi.update_gyro_data(roll, pitch)
                            if DEBUG:
                                print(f"✅ Python Madgwick: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
                        
                        # Detector verilerini göster (ham accelerometer)
                        det_text = f"dX={ax:.2f}  dY={ay:.2f}  dZ={az:.2f}"
                        self.detector_label.text = det_text
                        if DEBUG:
                            print(f"✅ Detector güncellendi: {det_text}")
                        
                        # Max değerler güncelle
                        if abs(ax) > abs(self.max_detector_x):
                            self.max_detector_x = ax
                        if abs(ay) > abs(self.max_detector_y):
                            self.max_detector_y = ay
                        if abs(az) > abs(self.max_detector_z):
                            self.max_detector_z = az
                        
                        self.max_detector_label.text = f"MAX: dX={self.max_detector_x:.2f}  dY={self.max_detector_y:.2f}  dZ={self.max_detector_z:.2f}"
                        
                    except (ValueError, TypeError) as e:
                        if DEBUG:
                            print(f"⚠️  Madgwick hesaplama hatası: {e}")
                else:
                    if DEBUG:
                        print(f"❌ Gyro veya Accel verisi eksik!")
            else:
                if DEBUG:
                    print(f"❌ Raw sensors verisi JSON'da yok veya None!")
                
            # Arduino Madgwick sonuçlarını artık kullanma (Python hesapladığı için)
            # if 'madgwick' in data and data['madgwick'] is not None:
            #     print("ℹ️  Arduino Madgwick verisi mevcut ama Python hesaplaması kullanılıyor")
            
            # ADXL345 - Platform için (None kontrolü ile)
            if 'adxl345' in data and data['adxl345'] is not None:
                adxl = data['adxl345']
                if DEBUG:
                    print(f"📊 ADXL345 verisi: {adxl}")
                try:
                    dx = float(adxl.get('x', 0) or 0)
                    dy = float(adxl.get('y', 0) or 0)
                    dz = float(adxl.get('z', 0) or 0)
                    plat_text = f"pX={dx:.2f}  pY={dy:.2f}  pZ={dz:.2f}"
                    self.platform_label.text = plat_text
                    if DEBUG:
                        print(f"✅ Platform güncellendi: {plat_text}")
                    
                    # Max değerler güncelle
                    if abs(dx) > abs(self.max_accel_x):
                        self.max_accel_x = dx
                        if DEBUG:
                            print(f"🔝 Yeni max platform X: {self.max_accel_x}")
                    if abs(dy) > abs(self.max_accel_y):
                        self.max_accel_y = dy
                        if DEBUG:
                            print(f"🔝 Yeni max platform Y: {self.max_accel_y}")
                    if abs(dz) > abs(self.max_accel_z):
                        self.max_accel_z = dz
                        if DEBUG:
                            print(f"🔝 Yeni max platform Z: {self.max_accel_z}")
                    
                    self.max_platform_label.text = f"MAX: pX={self.max_accel_x:.2f}  pY={self.max_accel_y:.2f}  pZ={self.max_accel_z:.2f}"
                    if DEBUG:
                        print(f"✅ Max platform label güncellendi")
                except (ValueError, TypeError) as e:
                    if DEBUG:
                        print(f"⚠️  Platform veri hatası: {e}")
            else:
                if DEBUG:
                    print(f"❌ ADXL345 verisi JSON'da yok veya None!")
        
        except Exception as e:
            print(f"❌ update_sensor_data hatası: {e}")
            import traceback
            traceback.print_exc()

    def on_fn_toggle(self, instance, value):
        """Fn tuşu durumunu değiştir ve korumalı butonları etkinleştir/devre dışı bırak"""
        if value == 'down':
            self.fn_key_pressed = True
            self.fn_status_label.text = 'Fn: ON'
            self.fn_status_label.color = (0.5, 1, 0.5, 1)  # Yeşil
            
            # Korumalı butonları etkinleştir
            self.calibrate_btn.disabled = False
            self.calibrate_btn.background_color = (0.2, 0.6, 0.2, 1)  # Yeşil
            
            self.reset_max_btn.disabled = False 
            self.reset_max_btn.background_color = (0.8, 0.2, 0.2, 1)  # Kırmızı
            
            # Axis butonlarını etkinleştir
            for btn in self.axis_buttons:
                btn.disabled = False
                btn.default_color = (0.545, 0.133, 0.196, 1)  # Normal renk
                btn.active_color = (0.545, 0.133, 0.196, 1)
                btn.background_color = (0.545, 0.133, 0.196, 1)
        else:
            self.fn_key_pressed = False
            self.fn_status_label.text = 'Fn: OFF'
            self.fn_status_label.color = (1, 0.5, 0.5, 1)  # Kırmızı
            
            # Korumalı butonları devre dışı bırak
            self.calibrate_btn.disabled = True
            self.calibrate_btn.background_color = (0.6, 0.6, 0.6, 1)  # Gri
            
            self.reset_max_btn.disabled = True
            self.reset_max_btn.background_color = (0.6, 0.6, 0.6, 1)  # Gri
            
            # Axis butonlarını devre dışı bırak
            for btn in self.axis_buttons:
                btn.disabled = True
                btn.state = 'normal'  # Toggle durumunu sıfırla
                btn.default_color = (0.6, 0.6, 0.6, 1)  # Gri
                btn.active_color = (0.6, 0.6, 0.6, 1)
                btn.background_color = (0.6, 0.6, 0.6, 1)

    def enable_all_motors(self):
        """Tüm motorları enable yap"""
        print("🚀 Tüm motorları enable yapılıyor...")
        if self.motor_arduino:
            try:
                # Tüm motorları enable yap
                self.motor_arduino.write(b'ENABLE_X\n')
                self.motor_arduino.flush()
                print("✅ X motoru enable edildi")
                
                self.motor_arduino.write(b'ENABLE_Y\n')
                self.motor_arduino.flush()
                print("✅ Y motoru enable edildi")
                
                self.motor_arduino.write(b'ENABLE_Z\n')
                self.motor_arduino.flush()
                print("✅ Z motoru enable edildi")
                
                # Durum sorgusu da yap
                self.query_motor_status()
                
            except Exception as e:
                print(f"❌ Motor enable hatası: {e}")
        else:
            print("❌ Motor Arduino bağlı değil - enable yapılamıyor")

    def query_motor_status(self):
        """Motor Arduino'dan switch durumlarını sorgula"""
        if self.motor_arduino:
            try:
                self.motor_arduino.write(b'TEST\n')
                self.motor_arduino.flush()
                print("📡 Motor Arduino'ya durum sorgusu gönderildi")
            except Exception as e:
                print(f"❌ Motor durum sorgusu gönderilemedi: {e}")
        else:
            print("⚠️  Motor Arduino bağlı değil - durum sorgulanamıyor")

    def reset_max_values(self, instance):
        """Maksimum değerleri sıfırla - Sadece Fn tuşu basılıyken çalışır"""
        if not self.fn_key_pressed:
            return
            
        self.max_accel_x = 0
        self.max_accel_y = 0
        self.max_accel_z = 0
        self.max_detector_x = 0
        self.max_detector_y = 0
        self.max_detector_z = 0
        
        self.max_platform_label.text = 'MAX: pX=0  pY=0  pZ=0'
        self.max_detector_label.text = 'MAX: dX=0  dY=0  dZ=0'

    def calibrate_gyro(self, instance):
        """Gyroscope'u kalibre et - Sadece Fn tuşu basılıyken çalışır"""
        if not self.fn_key_pressed:
            return
            
        instance.text = "Calibrating..."
        instance.background_color = (0.8, 0.8, 0.2, 1)
        self.send_to_arduino("CALIBRATE")
        Clock.schedule_once(lambda dt: self.reset_calibrate_button(instance), 3)

    def reset_calibrate_button(self, button):
        button.text = "Calibrate\nGyro"
        if self.fn_key_pressed:
            button.background_color = (0.2, 0.6, 0.2, 1)
        else:
            button.background_color = (0.6, 0.6, 0.6, 1)

    def test_data(self, instance):
        """Test data reception from Arduino"""
        if self.serial_port:
            print(f"📊 Test butonu basıldı - Port: {self.serial_port.port}")
            print(f"📈 Buffer'da {len(self.data_buffer)} veri var")
            self.send_to_arduino("TEST")
            instance.text = "Testing..."
            Clock.schedule_once(lambda dt: setattr(instance, 'text', 'Test\nData'), 2)
        else:
            print("❌ Serial port bulunamadı!")
            instance.text = "No Port!"
            Clock.schedule_once(lambda dt: setattr(instance, 'text', 'Test\nData'), 2)

    def on_mult_button(self, instance):
        # ADMIN MODE KONTROLÜ
        if not self.admin_mode_active:
            print(f"❌ ADMIN MODU KAPALI - Hız değiştirme engellendi")
            # Buton durumunu eski haline döndür
            for btn in self.mult_buttons:
                if btn.state == 'down':
                    return  # Zaten seçili olan butonu koru
            return
        
        self.speed_label.text = f"Speed: {instance.text}"
        for btn in self.mult_buttons:
            btn.state = 'normal'
        instance.state = 'down'
        self.send_to_arduino(f"S{instance.text}")

    def on_axis_toggle(self, instance, value):
        """Axis unlock/lock toggle - Sadece Fn tuşu basılıyken çalışır"""
        # ADMIN MODE KONTROLÜ
        if not self.admin_mode_active:
            print(f"❌ ADMIN MODU KAPALI - Axis toggle engellendi")
            instance.state = 'normal'  # Toggle durumunu geri al
            return
        
        if not self.fn_key_pressed:
            instance.state = 'normal'  # Toggle durumunu geri al
            return
            
        if value == 'down':
            axis = instance.text.replace("Unlock ", "").replace("Lock ", "")
            instance.text = f"Lock {axis}"
            self.axis_locked[axis] = True  # Lock durumunu güncelle
            self.send_to_arduino(f"l{axis}")
            print(f"🔒 {axis} ekseni lock edildi")
        else:
            axis = instance.text.replace("Lock ", "").replace("Unlock ", "")
            instance.text = f"Unlock {axis}"
            self.axis_locked[axis] = False  # Unlock durumunu güncelle
            self.send_to_arduino(f"ul{axis}")
            print(f"🔓 {axis} ekseni unlock edildi")
            
        # Lock status labellarını güncelle
        for i, ax in enumerate(['X', 'Y', 'Z']):
            status = "locked" if self.axis_locked[ax] else "unlocked"
            self.lock_status_labels[i].text = f"{ax}: {status}"

    def send_to_arduino(self, message):
        """Arduino'ya mesaj gönder - Motor komutları motor Arduino'suna, diğerleri sensör Arduino'suna"""
        try:
            # Motor komutlarını belirle
            motor_commands = ['X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-', 'STOPX', 'STOPY', 'STOPZ']
            speed_commands = ['SMIN', 'S%25', 'S%50', 'S%75', 'SMAX']
            axis_commands = ['lX', 'lY', 'lZ', 'ulX', 'ulY', 'ulZ']  # lock/unlock commands
            
            # Debug: Speed command kontrolü
            is_speed_command = any(message.startswith(cmd) for cmd in speed_commands)
            if is_speed_command:
                original_print(f"🔥 SPEED COMMAND DETECTED: {message}")
                original_print(f"🔥 Motor Arduino connected: {self.motor_arduino is not None}")
            
            # Hangi Arduino'ya gönderileceğini belirle
            is_motor_command = (
                message in motor_commands or 
                is_speed_command or
                message in axis_commands
            )
            
            if is_motor_command and self.motor_arduino:
                # Motor komutunu motor Arduino'suna gönder
                try:
                    self.motor_arduino.write((message + '\n').encode())
                    self.motor_arduino.flush()
                    original_print(f"⚙️  Motor Arduino'ya gönderildi: {message}")
                except Exception as e:
                    original_print(f"❌ Motor Arduino gönderim hatası: {e}")
                    self.handle_arduino_disconnection('motor')
                
            elif not is_motor_command and self.sensor_arduino:
                # Sensör komutunu sensör Arduino'suna gönder (kalibrasyon vb.)
                try:
                    self.sensor_arduino.write((message + '\n').encode())
                    self.sensor_arduino.flush()
                    original_print(f"📊 Sensör Arduino'ya gönderildi: {message}")
                except Exception as e:
                    original_print(f"❌ Sensör Arduino gönderim hatası: {e}")
                    self.handle_arduino_disconnection('sensor')
                
            elif is_motor_command and not self.motor_arduino:
                original_print(f"⚠️  Motor Arduino bağlı değil - komut gönderilemedi: {message}")
                # Speed komutları için fallback YAPMA - sadece hata ver
                if is_speed_command:
                    original_print(f"❌ SPEED COMMAND FAILED: Motor Arduino required for {message}")
                    return
                # Diğer motor komutları için fallback
                elif self.sensor_arduino:
                    original_print(f"🔄 Fallback: Sensör Arduino'ya motor komutu gönderiliyor...")
                    try:
                        self.sensor_arduino.write((message + '\n').encode())
                        self.sensor_arduino.flush()
                    except:
                        pass
                
            elif not is_motor_command and not self.sensor_arduino:
                original_print(f"⚠️  Sensör Arduino bağlı değil - komut gönderilemedi: {message}")
                # Fallback: Motor Arduino'ya gönder
                if self.motor_arduino:
                    original_print(f"🔄 Fallback: Motor Arduino'ya sensör komutu gönderiliyor...")
                    try:
                        self.motor_arduino.write((message + '\n').encode())
                        self.motor_arduino.flush()
                    except:
                        pass
                
            else:
                # Fallback: Hangi Arduino varsa ona gönder
                if self.motor_arduino:
                    try:
                        self.motor_arduino.write((message + '\n').encode())
                        self.motor_arduino.flush()
                        original_print(f"🔄 Fallback - Motor Arduino'ya gönderildi: {message}")
                    except Exception as e:
                        original_print(f"❌ Fallback motor gönderim hatası: {e}")
                elif self.sensor_arduino:
                    try:
                        self.sensor_arduino.write((message + '\n').encode())
                        self.sensor_arduino.flush()
                        original_print(f"🔄 Fallback - Sensör Arduino'ya gönderildi: {message}")
                    except Exception as e:
                        original_print(f"❌ Fallback sensör gönderim hatası: {e}")
                else:
                    original_print(f"❌ Hiçbir Arduino bağlı değil!")
                    
        except Exception as e:
            original_print(f"❌ Arduino gönderim hatası: {e}")

    def handle_arduino_disconnection(self, arduino_type):
        """Arduino bağlantısı koptuğunda işlem yap"""
        if arduino_type == 'motor':
            original_print("🔌 Motor Arduino bağlantısı koptu!")
            try:
                if self.motor_arduino:
                    self.motor_arduino.close()
            except:
                pass
            self.motor_arduino = None
            
        elif arduino_type == 'sensor':
            original_print("🔌 Sensör Arduino bağlantısı koptu!")
            try:
                if self.sensor_arduino:
                    self.sensor_arduino.close()
            except:
                pass
            self.sensor_arduino = None
            self.serial_port = None  # Geriye uyumluluk için
            
        # Yeniden bağlanma denemesi için timer başlat
        Clock.schedule_once(lambda dt: self.attempt_reconnection(arduino_type), 3.0)

    def attempt_reconnection(self, arduino_type):
        """Kopan Arduino'ya yeniden bağlanmayı dene"""
        original_print(f"🔄 {arduino_type.title()} Arduino'ya yeniden bağlanmaya çalışılıyor...")
        
        import glob
        import os
        
        # Mevcut portları yeniden tara
        usb_ports = glob.glob('/dev/tty.usb*') + glob.glob('/dev/cu.usb*') + glob.glob('/dev/tty.wch*') + glob.glob('/dev/cu.wch*')
        possible_ports = [
            '/dev/tty.usbserial-120', '/dev/tty.usbmodem14101', 
            '/dev/cu.usbserial-120', '/dev/cu.usbmodem14101',
            '/dev/tty.wchusbserial14120', '/dev/cu.wchusbserial14120',
            '/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1'
        ] + usb_ports
        
        available_ports = [port for port in possible_ports if os.path.exists(port)]
        
        for port in available_ports:
            # Zaten bağlı portları atla
            if ((self.sensor_arduino and port == self.sensor_arduino.port) or 
                (self.motor_arduino and port == self.motor_arduino.port)):
                continue
                
            try:
                test_port = serial.Serial(port=port, baudrate=9600, timeout=1)
                test_port.write(b'IDENTIFY\n')
                test_port.flush()
                time.sleep(1)
                
                if test_port.in_waiting > 0:
                    response = test_port.read(test_port.in_waiting).decode('utf-8', errors='ignore')
                    
                    if arduino_type == 'sensor' and ('SENSOR_ARDUINO' in response or '{' in response):
                        self.sensor_arduino = test_port
                        self.serial_port = test_port  # Geriye uyumluluk
                        original_print(f"✅ Sensör Arduino yeniden bağlandı: {port}")
                        
                        # Yeni thread başlat
                        self.sensor_thread = threading.Thread(target=self.sensor_read_thread, daemon=True)
                        self.sensor_thread.start()
                        return
                        
                    elif arduino_type == 'motor' and ('MOTOR_ARDUINO' in response or 'LED_TEST' in response):
                        self.motor_arduino = test_port
                        original_print(f"✅ Motor Arduino yeniden bağlandı: {port}")
                        
                        # Yeni thread başlat
                        self.motor_thread = threading.Thread(target=self.motor_read_thread, daemon=True)
                        self.motor_thread.start()
                        return
                
                test_port.close()
                
            except Exception as e:
                try:
                    test_port.close()
                except:
                    pass
                continue
        
        original_print(f"❌ {arduino_type.title()} Arduino yeniden bağlanamadı")
        # 10 saniye sonra tekrar dene
        Clock.schedule_once(lambda dt: self.attempt_reconnection(arduino_type), 10.0)

class MainApp(App):
    def build(self):
        return MainScreen()

if __name__ == '__main__':
    MainApp().run()




