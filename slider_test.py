from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.label import Label
from kivy.graphics import Color, Ellipse, Line, Triangle, InstructionGroup
from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.clock import Clock
import random
from kivy.config import Config
from kivy.uix.slider import Slider


class ArrowButton(Button):
    def __init__(self, direction, label_text, **kwargs):
        super().__init__(**kwargs)
        self.direction = direction
        self.text = label_text
        self.color = (1, 1, 1, 1)
        self.font_size = 24
        self.background_normal = ''
        self.background_color = (0, 0, 0, 0)
        self.size_hint = (None, None)
        self.size = (80, 80)

        with self.canvas.before:
            self.bg_color = Color(0.2, 0.6, 1, 1)
            self.bg_circle = Ellipse(pos=self.pos, size=self.size)

        self.bind(pos=self.update_graphics, size=self.update_graphics)
        self.canvas_instruction = InstructionGroup()
        self.canvas.add(self.canvas_instruction)

    def update_graphics(self, *args):
        self.bg_circle.pos = self.pos
        self.bg_circle.size = self.size

        self.canvas_instruction.clear()
        self.canvas_instruction.add(Color(1, 1, 1, 1))
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


class MainScreen(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Yön butonları
        self.add_widget(ArrowButton(direction='up', label_text='Z+', pos=(100, 240)))
        self.add_widget(ArrowButton(direction='down', label_text='Z-', pos=(100, 85)))
        self.add_widget(ArrowButton(direction='left', label_text='X-', pos=(20, 160)))
        self.add_widget(ArrowButton(direction='right', label_text='X+', pos=(180, 160)))
        # Y ekseni butonları
        self.add_widget(ArrowButton(direction='up', label_text='Y+', pos=(600, 240)))
        self.add_widget(ArrowButton(direction='down', label_text='Y-', pos=(600, 85)))

        # Su terazisi (güncel hali)
        #self.terazi = GyroDisplay(size_hint=(None, None), size=(150, 150), pos=(310, 310))
       # self.add_widget(self.terazi)

        # Toggle butonlar
        unlock_buttons = ['Unlock X', 'Unlock Y', 'Unlock Z']
        for i, label in enumerate(unlock_buttons):
            self.add_widget(ToggleButton(
                text=label,
                size_hint=(None, None),
                size=(100, 40),
                pos=(350, 100 + i * 50),
                background_normal='',
                background_color=(0.6, 0.6, 0.6, 1),
                font_size=20))

        # Sensör verileri
        sensor_data = {
            'left': ['TEMPERATURE: 24°C', 'HUMIDITY: 60%','DISTANCE: 120cm'],
            'midleft': [],
            'mid': ['DEDECTOR:  X=0  Y=0  Z=0', 'PLATFORM:  X=0  Y=0  Z=0'],
        }

        x_positions = {
            'left': 50,
            'midleft': 320,
            'mid': 580,
        }

        for group, labels in sensor_data.items():
            for i, text in enumerate(labels):
                if group == "left":
                    lbl = Label(
                        text=text,
                        font_size=20,
                        pos=(x_positions[group], 350 + i*30),
                        size_hint=(None, None),
                        size=(200, 30),
                        color=(1, 1, 1, 1),
                        halign='left',
                        text_size=(200, 30)
                    )
                else:
                    lbl = Label(
                        text=text,
                        font_size=20,
                        pos=(x_positions[group], 350 + i*30),
                        size_hint=(None, None),
                        color=(1, 1, 1, 1)
                    )
                self.add_widget(lbl)

        # Slider ekle (motion butonları yerine)
        speed_label = Label(
            text="Speed",
            font_size=20,
            pos=(350, 55),  # Ortaya alındı
            size_hint=(None, None),
            size=(100, 40),
            color=(1, 1, 1, 1)
        )
        self.add_widget(speed_label)

        slider = Slider(
            min=0, max=100, value=50,
            size_hint=(None, None),
            size=(300, 40),
            pos=(300, 10)  # Ortaya alındı
        )
        self.add_widget(slider)
   
class MainApp(App):
    def build(self):
        return MainScreen()

if __name__ == '__main__':
    MainApp().run()

