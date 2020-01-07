import kivy
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.textinput import TextInput
from kivy.uix.image import Image
from kivy.config import Config
from kivy.base import runTouchApp
from kivy.uix.spinner import Spinner
from kivy.uix.popup import Popup
from functions_4_gui import *

import main_digital_world
from kivy.graphics import Rectangle
from kivy.graphics import Color
from kivy.uix.widget import Widget
from kivy.core.window import Window
import ctypes




# In[Settings for GUI]
spacing_Y = 0.1
size_hint_X = 0.2
size_hint_Y = 0.1
spacing_X_first = 0.0

spacing_X_right = 1 - size_hint_X - 0.01

pos_Y_1 = 1
pos_Y_11 = pos_Y_1 - spacing_Y
pos_Y_2 = pos_Y_11 - 2 * spacing_Y
pos_Y_3 = pos_Y_2 - spacing_Y
pos_Y_4 = pos_Y_3 - 2 * spacing_Y
pos_Y_5 = pos_Y_4 - spacing_Y
pos_Y_6 = pos_Y_5 - spacing_Y
pos_Y_7 = pos_Y_6 - spacing_Y

user32 = ctypes.windll.user32
(APP_W, APP_H) = Window.size
SCALE = min(user32.GetSystemMetrics(0)/APP_W*0.6,user32.GetSystemMetrics(1)/APP_H*0.6)

# In[start GUI]
# noinspection PyGlobalUndefined
class window(FloatLayout):
    def __init__(self, **kwargs):



        super(window, self).__init__(**kwargs)
        Window.clearcolor = (1, 1, 1, 1)
        Window.fullscreen = False
        Config.set('input', 'mouse', 'mouse,multitouch_on_demand')  # remove the circles at the right click

        # In[logo]
        logo = Image(source='logo.jpg', allow_stretch = True, size_hint = (SCALE,SCALE),pos_hint={'center_x': 0.5, 'center_y': .6})
        self.add_widget(logo)

        # In[Configuration button]
        start_robot = Button(pos_hint={'left': 1, 'top': 0.1},
                             size_hint=(0.7, 0.1), font_size='25', text="Start cutting by robot",
                             background_color=(0, 255, 0, 0.7))
        start_robot.bind(on_release=self.Start_Robot)
        self.add_widget(start_robot)

        # In[Camera calibration button]
        camera_calibration = Button(pos_hint={'right': 0.85, 'top': 0.1},
                                    size_hint=(0.15, 0.1), font_size='15', text="Calibrate camera",
                                    background_color=(0, 0, 255, 0.3))
        camera_calibration.bind(on_release=calibrate_camera_external)
        self.add_widget(camera_calibration)

        # In[Connect robot button]
        connect_to_robot = Button(pos_hint={'right': 1, 'top': 0.2},
                                  size_hint=(0.30, 0.1), font_size='15', text="Connect robot",
                                  background_color=(0, 0, 255, 0.3))
        connect_to_robot.bind(on_release=connect_robot)
        self.add_widget(connect_to_robot)
        #
        # # In[Useless button]
        # useless_button = Button(pos_hint={'right': 1, 'top': 0.2},
        #                         size_hint=(0.15, 0.1), font_size='15', text="Yet a useless button",
        #                         background_color=(0, 0, 255, 0.3))
        # useless_button.bind(on_release=useless)
        # self.add_widget(useless_button)

        # In[Print Checkerboard button]
        create_checkerboard = Button(pos_hint={'right': 1, 'top': 0.1},
                                     size_hint=(0.15, 0.1), font_size='15', text="Create calibration checkerboard",
                                     background_color=(0, 0, 255, 0.3))
        create_checkerboard.bind(on_release=checkerboard)
        self.add_widget(create_checkerboard)

        # In[Show instructions]
        l0 = Label(text='Instructions: \n', font_size='35', pos_hint={'x': 0.28, 'y': 0.35})
        self.add_widget(l0)
        l00 = Label(
            text=' 1. Place robot in van at the designated location \n 2. Plug in the ethernet cable \n 3. Plug in the '
                 'usb cable for the camera \n 4. Give the desired variables \n 5. Connect the robot \n 6. Click on "Start cutting by robot"',
            font_size='25', pos_hint={'x': 0.28, 'y': 0.2})
        self.add_widget(l00)

        # In[Change parameters]

        # Window Placement X
        self.WINDOW_PLACEMENT_X = TextInput(text="Placement X", height=100, input_filter="float",
                                            size_hint=(size_hint_X, size_hint_Y),
                                            pos_hint={'left': 1, 'top': pos_Y_5}, multiline=False)
        self.add_widget(self.WINDOW_PLACEMENT_X)
        l2 = Label(text='Horizontal placement of window. Measured from top left corner [mm]', font_size='15',
                   pos_hint={'x': -0.144, 'y': 0.4 - size_hint_Y / 2 - spacing_Y * 6})
        self.add_widget(l2)

        # Window Placement Y
        self.WINDOW_PLACEMENT_Y = TextInput(text='Placement Y', height=100, input_filter="float",
                                            size_hint=(size_hint_X, size_hint_Y),
                                            pos_hint={'left': 1, 'top': pos_Y_6}, multiline=False)
        self.add_widget(self.WINDOW_PLACEMENT_Y)
        l21 = Label(text='Vertical placement of window. Measured from top left corner [mm]', font_size='15',
                    pos_hint={'x': -0.15, 'y': 0.4 - size_hint_Y / 2 - spacing_Y * 7})
        self.add_widget(l21)

        # Dropdown for van selector
        dropdown_vans = Spinner(
            # default value shown
            text='Select a van ',
            # available values
            values=('Ducato L2H2', 'Sprinter 2019', 'Renault Master', 'Peugeot Boxer'),
            # just for positioning in our example
            size_hint=(size_hint_X, size_hint_Y),
            pos_hint={'left': 1, 'top': pos_Y_1})

        # Dropdown for window sizes
        dropdown_sizes = Spinner(
            # default value shown
            text='Select window size',
            # available values
            values=('300x300', '300x400', '300x500', '400x500'),
            # just for positioning in our example
            size_hint=(size_hint_X, size_hint_Y),
            pos_hint={'left': 1, 'top': pos_Y_11})

        def show_selected_size(dropdown_sizes, text):
            global size_window
            size_window = dropdown_sizes.text

            # Split the value from the string to a list
            # Example: '300x300' to [300,300]
            size_window = map(int, size_window.split('x'))
            size_window = list(size_window)

        def show_selected_van(dropdown_vans, text):
            global brand_van
            brand_van = dropdown_vans.text

        dropdown_sizes.bind(text=show_selected_size)
        dropdown_vans.bind(text=show_selected_van)

        self.add_widget(dropdown_vans)
        self.add_widget(dropdown_sizes)

    def Start_Robot(self, obj):
        max_window_size_x = int(size_window[0])/1000
        max_window_size_y = int(size_window[1])/1000
        window_placement_x = int(self.WINDOW_PLACEMENT_X.text)/1000
        window_placement_y = int(self.WINDOW_PLACEMENT_Y.text)/1000

        starting_robot_external(max_window_size_x, max_window_size_y, window_placement_x, window_placement_y)

    def Create_popup(self, text1):
        popup = Popup(title='Test popup',
                      content=Label(text=text1),
                      size_hint=(None, None), size=(400, 400))
        popup.open()

class NameApp(App):
    def build(self):
        self.title = 'Interface cutting vans'
        gui = window()
        return gui


if __name__ == "__main__":
    NameApp().run()
