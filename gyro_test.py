#!/usr/bin/env python3

"""
Gyro Stabilizasyonu Test - Arduino olmadan çalışır
Bu test, GyroDisplay sınıfının stabilizasyon özelliklerini test eder.
"""

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.graphics import Color, Ellipse, Line, Rectangle, RoundedRectangle
from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.config import Config
import math
import time
import random

# Pencere ayarları
Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Window.size = (800, 480)

class GyroDisplay(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Gyro değerleri
        self.roll = 0.0
        self.pitch = 0.0
        
        # Filtreleme parametreleri
        self.filter_alpha = 0.1  # Düşük geçiş filtresi katsayısı (0.0-1.0)
        self.dead_zone = 0.8     # Ölü bölge threshold
        self.data_timeout = 1.0  # Veri timeout süresi (saniye)
        
        # Durum takibi
        self.last_data_time = time.time()
        self.data_active = False
        
        # Ekran güncellemesi
        Clock.schedule_interval(self.update_display, 1/60)  # 60 FPS
    
    def update_gyro_data(self, new_roll, new_pitch):
        """Yeni gyro verisi aldığında çağır"""
        current_time = time.time()
        
        # Veri aktif
        self.data_active = True
        self.last_data_time = current_time
        
        # Ölü bölge kontrolü
        if abs(new_roll) < self.dead_zone and abs(new_pitch) < self.dead_zone:
            # Çok küçük değişimler - sıfırla
            self.roll = 0.0
            self.pitch = 0.0
            return
        
        # Düşük geçiş filtresi uygula
        self.roll = self.roll * (1 - self.filter_alpha) + new_roll * self.filter_alpha
        self.pitch = self.pitch * (1 - self.filter_alpha) + new_pitch * self.filter_alpha
    
    def check_data_timeout(self):
        """Veri timeout kontrolü"""
        current_time = time.time()
        
        if self.data_active and (current_time - self.last_data_time) > self.data_timeout:
            # Timeout - gyro değerlerini sıfırla
            print("🛑 Arduino veri timeout - gyro sıfırlanıyor")
            self.roll = 0.0
            self.pitch = 0.0
            self.data_active = False
    
    def update_display(self, dt):
        # Timeout kontrolü
        self.check_data_timeout()
        
        self.canvas.clear()
        with self.canvas:
            # Arka plan
            Color(1, 0.7, 0, 1)  # Turuncu
            Ellipse(pos=self.pos, size=self.size)
            
            # Merkez noktalar
            center_x = self.center_x
            center_y = self.center_y
            
            # Konsantrik daireler
            Color(0, 0, 0, 1)  # Siyah çizgiler
            for r in [20, 40, 60, 80]:
                Line(circle=(center_x, center_y, r), width=1.5)
            
            # Merkez çizgiler
            Line(points=[center_x - 100, center_y, center_x + 100, center_y], width=2)
            Line(points=[center_x, center_y - 100, center_x, center_y + 100], width=2)
            
            # Gyro etkisi ile nokta konumu
            # Roll ve pitch değerlerini piksel değerine çevir
            max_offset = 80  # Maksimum piksel ofset
            
            x_offset = -self.roll * max_offset / 45.0  # 45 derece = maksimum ofset
            y_offset = self.pitch * max_offset / 45.0
            
            # Sınırları kontrol et
            x_offset = max(-max_offset, min(max_offset, x_offset))
            y_offset = max(-max_offset, min(max_offset, y_offset))
            
            # Gyro noktası
            point_x = center_x + x_offset
            point_y = center_y + y_offset
            
            # Nokta rengi - veri durumuna göre
            if self.data_active:
                Color(1, 0, 0, 1)  # Kırmızı - veri aktif
            else:
                Color(0.5, 0.5, 0.5, 1)  # Gri - veri yok
            
            Ellipse(pos=(point_x - 6, point_y - 6), size=(12, 12))
            
            # Merkez noktası
            Color(0, 0, 1, 1)  # Mavi
            Ellipse(pos=(center_x - 3, center_y - 3), size=(6, 6))

class TestScreen(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        # Arka plan
        with self.canvas.before:
            Color(1, 0.9, 0.7, 1)  # Sarımsı arka plan
            Rectangle(size=Window.size, pos=(0, 0))
            
            # Ana beyaz panel
            Color(1, 1, 1, 1)
            panel_width = Window.width * 0.8
            panel_height = Window.height * 0.8
            self.main_panel = RoundedRectangle(
                size=(panel_width, panel_height),
                pos=(Window.width * 0.1, Window.height * 0.1),
                radius=[30, 30, 30, 30]
            )
        
        # Gyro göstergesi
        self.gyro_display = GyroDisplay(
            size_hint=(None, None), 
            size=(200, 200),
            pos_hint={'center_x': 0.5, 'center_y': 0.65}
        )
        self.add_widget(self.gyro_display)
        
        # Test butonları
        btn_layout = FloatLayout(size_hint=(1, 0.3), pos_hint={'x': 0, 'y': 0})
        
        # Gyro değerlerini test et
        btn_roll_left = Button(
            text="Roll -10°",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.1, 'center_y': 0.7}
        )
        btn_roll_left.bind(on_press=lambda x: self.test_gyro(-10, 0))
        btn_layout.add_widget(btn_roll_left)
        
        btn_roll_right = Button(
            text="Roll +10°",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.25, 'center_y': 0.7}
        )
        btn_roll_right.bind(on_press=lambda x: self.test_gyro(10, 0))
        btn_layout.add_widget(btn_roll_right)
        
        btn_pitch_up = Button(
            text="Pitch +10°",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.4, 'center_y': 0.7}
        )
        btn_pitch_up.bind(on_press=lambda x: self.test_gyro(0, 10))
        btn_layout.add_widget(btn_pitch_up)
        
        btn_pitch_down = Button(
            text="Pitch -10°",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.55, 'center_y': 0.7}
        )
        btn_pitch_down.bind(on_press=lambda x: self.test_gyro(0, -10))
        btn_layout.add_widget(btn_pitch_down)
        
        btn_zero = Button(
            text="Sıfırla",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.7, 'center_y': 0.7}
        )
        btn_zero.bind(on_press=lambda x: self.test_gyro(0, 0))
        btn_layout.add_widget(btn_zero)
        
        # Random hareket testi
        btn_random = Button(
            text="Random\nHareket",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.1, 'center_y': 0.4}
        )
        btn_random.bind(on_press=self.start_random_movement)
        btn_layout.add_widget(btn_random)
        
        # Timeout test
        btn_timeout = Button(
            text="Timeout\nTesti",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.25, 'center_y': 0.4}
        )
        btn_timeout.bind(on_press=self.test_timeout)
        btn_layout.add_widget(btn_timeout)
        
        # Ölü bölge testi
        btn_deadzone = Button(
            text="Ölü Bölge\nTesti",
            size_hint=(None, None),
            size=(120, 60),
            pos_hint={'x': 0.4, 'center_y': 0.4}
        )
        btn_deadzone.bind(on_press=self.test_deadzone)
        btn_layout.add_widget(btn_deadzone)
        
        # Durum etiketi
        self.status_label = Label(
            text="Gyro Stabilizasyon Testi\nKırmızı nokta: Veri aktif\nGri nokta: Veri yok",
            size_hint=(None, None),
            size=(400, 100),
            pos_hint={'center_x': 0.65, 'center_y': 0.4},
            color=(0, 0, 0, 1),
            halign='center'
        )
        self.status_label.text_size = self.status_label.size
        btn_layout.add_widget(self.status_label)
        
        self.add_widget(btn_layout)
        
        # Test için değişkenler
        self.random_movement_active = False
    
    def test_gyro(self, roll, pitch):
        """Manuel gyro testi"""
        self.gyro_display.update_gyro_data(roll, pitch)
        self.status_label.text = f"Test: Roll={roll:.1f}°, Pitch={pitch:.1f}°"
    
    def start_random_movement(self, instance):
        """Random hareket başlat/durdur"""
        if not self.random_movement_active:
            self.random_movement_active = True
            instance.text = "Durdur"
            Clock.schedule_interval(self.random_movement_update, 0.1)
            self.status_label.text = "Random hareket başladı"
        else:
            self.random_movement_active = False
            instance.text = "Random\nHareket"
            Clock.unschedule(self.random_movement_update)
            self.status_label.text = "Random hareket durdu"
    
    def random_movement_update(self, dt):
        """Random hareket güncelleme"""
        if self.random_movement_active:
            roll = random.uniform(-20, 20)
            pitch = random.uniform(-20, 20)
            self.gyro_display.update_gyro_data(roll, pitch)
    
    def test_timeout(self, instance):
        """Timeout testini başlat"""
        self.status_label.text = "Timeout testi: 5 saniye veri gönderilecek, sonra timeout"
        
        # 5 saniye veri gönder
        def send_data(dt):
            roll = random.uniform(-5, 5)
            pitch = random.uniform(-5, 5)
            self.gyro_display.update_gyro_data(roll, pitch)
        
        Clock.schedule_interval(send_data, 0.1)
        
        # 5 saniye sonra durdur
        def stop_data(dt):
            Clock.unschedule(send_data)
            self.status_label.text = "Veri durdu - timeout testi aktif"
        
        Clock.schedule_once(stop_data, 5)
    
    def test_deadzone(self, instance):
        """Ölü bölge testini başlat"""
        self.status_label.text = "Ölü bölge testi: Küçük değişimler gönderiliyor"
        
        # Küçük değerler gönder (ölü bölge altında)
        def send_small_data(dt):
            roll = random.uniform(-0.5, 0.5)  # Çok küçük
            pitch = random.uniform(-0.5, 0.5)
            self.gyro_display.update_gyro_data(roll, pitch)
        
        Clock.schedule_interval(send_small_data, 0.1)
        
        # 3 saniye sonra normal değerler gönder
        def send_normal_data(dt):
            Clock.unschedule(send_small_data)
            
            def send_normal(dt2):
                roll = random.uniform(-10, 10)
                pitch = random.uniform(-10, 10)
                self.gyro_display.update_gyro_data(roll, pitch)
            
            Clock.schedule_interval(send_normal, 0.1)
            self.status_label.text = "Normal değerler gönderiliyor"
            
            # 3 saniye sonra durdur
            def stop_normal(dt3):
                Clock.unschedule(send_normal)
                self.status_label.text = "Ölü bölge testi tamamlandı"
            
            Clock.schedule_once(stop_normal, 3)
        
        Clock.schedule_once(send_normal_data, 3)

class GyroTestApp(App):
    def build(self):
        return TestScreen()

if __name__ == '__main__':
    GyroTestApp().run()