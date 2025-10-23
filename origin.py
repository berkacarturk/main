from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.label import Label
from kivy.uix.slider import Slider
from kivy.graphics import Color, Ellipse, Line, Triangle, InstructionGroup, Rectangle, RoundedRectangle
from kivy.core.window import Window
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.uix.gridlayout import GridLayout
import random
from kivy.config import Config
Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Window.size = (800, 480)

class DirectionButton(Button):
    def __init__(self, text='', **kwargs):
        super().__init__(**kwargs)
        self.text = text
        self.background_normal = ''
        self.background_color = (0, 0, 0, 0)
        self.label = Label(text=self.text, font_size=24, color=(1, 1, 1, 1))
        self.add_widget(self.label)
        self.bind(pos=self.update_label_pos)
        with self.canvas.before:
            Color(0, 0, 0, 1)
            self.circle = Ellipse(pos=self.pos, size=self.size)
        self.bind(pos=self.update_graphics, size=self.update_graphics)

    def update_graphics(self, *args):
        self.circle.pos = self.pos
        self.circle.size = self.size

    def update_label_pos(self, *args):
        self.label.pos = self.pos
        self.label.size = self.size

class GyroDisplay(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        Clock.schedule_interval(self.update_display, 1 / 10)

    def update_display(self, dt):
        self.canvas.clear()
        with self.canvas:
            Color(1, 0.7, 0, 1) # Turuncu renk
            Ellipse(pos=self.pos, size=self.size)
            Color(0, 0, 0) # Siyah çizgiler
            for r in [20, 40, 60, 80]:
                Line(circle=(self.center_x, self.center_y, r), width=1.5)
            Line(points=[self.center_x - 100, self.center_y, self.center_x + 100, self.center_y], width=2)
            Line(points=[self.center_x, self.center_y - 100, self.center_x, self.center_y + 100], width=2)
            x = random.uniform(-1, 1) * 40 + self.center_x
            y = random.uniform(-1, 1) * 40 + self.center_y
            Color(1, 0, 0)
            Ellipse(pos=(x - 6, y - 6), size=(6, 6))

class MainScreen(FloatLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Arka plan ve ana panel çizimi (sadece bir kez!)
        with self.canvas.before:
            # Sarımsı arka plan
            Color(1, 0.9, 0.7, 1)
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
            
    
        self.terazi = GyroDisplay(size_hint=(None, None), size=(300, 300),
                                  pos_hint={'center_x': 0.5, 'center_y': 0.77})
                                            # Ana beyaz panel
        # ...widget eklemeleri aynı şekilde devam edecek...
        self.add_widget(self.terazi)

        # Sol yön tuşları
        left_grid = GridLayout(cols=3, rows=3,
                               size_hint=(None, None), size=(300, 300),
                               pos_hint={'center_x': 0.2, 'center_y': 0.35},
                               spacing=10)
        left_grid.add_widget(Widget())
        left_grid.add_widget(DirectionButton(text="Z+", size_hint=(None, None), size=(100, 100)))
        left_grid.add_widget(Widget())
        left_grid.add_widget(DirectionButton(text="X-", size_hint=(None, None), size=(100, 100)))
        left_grid.add_widget(Widget())
        left_grid.add_widget(DirectionButton(text="X+", size_hint=(None, None), size=(100, 100)))
        left_grid.add_widget(Widget())
        left_grid.add_widget(DirectionButton(text="Z-", size_hint=(None, None), size=(100, 100)))
        self.add_widget(left_grid)

        # Sağ Y yön tuşları
        y_buttons_layout = BoxLayout(orientation='vertical',
                                     size_hint=(None, None),
                                     size=(100, 240),
                                     spacing=10,
                                     pos_hint={'center_x': 0.75, 'center_y': 0.36})
        y_buttons_layout.add_widget(DirectionButton(text="Y+"))
        y_buttons_layout.add_widget(DirectionButton(text="Y-"))
        self.add_widget(y_buttons_layout)

        # Üst bilgi panelleri
        lbl_temp = Label(text="Temperature: 23C\nHumidity: 48%\nDistance: 600mm",
                         font_size=20, pos_hint={'center_x': 0.35, 'center_y': 0.8}, color=(0,0,0,1))
        self.add_widget(lbl_temp)
        lbl_acc = Label(text="Accelerometer\nX:0 Y:0 Z:0",
                        font_size=20, pos_hint={'center_x': 0.65, 'center_y': 0.8}, color=(0,0,0,1))
        self.add_widget(lbl_acc)

        # Axis Lock butonu paneli
        lock_panel = BoxLayout(orientation='vertical', spacing=10,
                               size_hint=(None, None), size=(300, 360),
                               pos_hint={'center_x': 0.5, 'center_y': 0.45})
        lock_panel.add_widget(Label(text="AXIS LOCK", font_size=16, color=(0,0,0,1)))
        btn_x = ToggleButton(text="Unlock X", size_hint_y=None, height=50)
        btn_y = ToggleButton(text="Unlock Y", size_hint_y=None, height=50)
        btn_z = ToggleButton(text="Unlock Z", size_hint_y=None, height=50)
        btn_x.bind(state=self.on_toggle_state)
        btn_y.bind(state=self.on_toggle_state)
        btn_z.bind(state=self.on_toggle_state)
        lock_panel.add_widget(btn_x)
        lock_panel.add_widget(btn_y)
        lock_panel.add_widget(btn_z)
        self.add_widget(lock_panel)

     
    def on_toggle_state(self, instance, value):
        if value == 'down':
            print(f"'{instance.text}' LOCKED.")
            instance.text = instance.text.replace("Unlock", "Lock")
        else:
            print(f"'{instance.text}' UNLOCKED.")
            instance.text = instance.text.replace("Lock", "Unlock")

class MainApp(App):
    def build(self):
        return MainScreen()

if __name__ == '__main__':
    MainApp().run()