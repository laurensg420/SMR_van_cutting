import kivy
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.textinput import TextInput
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout
from kivy.config import Config
from kivy.base import runTouchApp
from kivy.uix.spinner import Spinner
from kivy.graphics import Rectangle
from kivy.graphics import Color
from kivy.uix.widget import Widget
from kivy.core.window import Window
import ctypes
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.pagelayout import PageLayout
from kivy.uix.screenmanager import ScreenManager, Screen, FadeTransition
from kivy.lang import Builder
from kivy.uix.dropdown import DropDown
from kivy.base import runTouchApp
from kivy.uix.spinner import Spinner
from kivy.properties import BooleanProperty, ListProperty
from kivy.factory import Factory

from kivy.clock import Clock
import main_digital_world
import time
import threading
from queue import Queue


# user32 = ctypes.windll.user32
# (APP_W, APP_H) = Window.size
# SCALE = min(user32.GetSystemMetrics(0)/APP_W*0.6,user32.GetSystemMetrics(1)/APP_H*0.6)
van_select = []

class FunctionThread(threading.Thread):
    """Threaded File Downloader"""

    def __init__(self, queue):
        """Initialize the thread"""
        threading.Thread.__init__(self)
        self.queue = queue

    def run(self):
        """Run the thread"""
        while not self.queue.empty():
            # gets the url from the queue
            connect_function = self.queue.get()
            connect_function()
            # download the file
            print('end of executing {}'.format(connect_function))
            # send a signal to the queue that the job is done
            self.queue.task_done()
        print('end of thread')

class MainScreen(Screen):
    pass


class ConnectScreen(Screen):
    connect_robot = BooleanProperty(False)
    connect_camera = BooleanProperty(False)
    connect_arduino = BooleanProperty(False)
    connecting = BooleanProperty(False)

    def connect_to_robot_thread(self):
        app = CuttingApp.get_running_app()
        try:
            device_count, imgs = app.stepHandler.aruco_list_cameras()
            camera_index = device_count - 1
            port_list = app.stepHandler.arduino_list_ports()
            self.connect_robot,  self.connect_camera, self.connect_arduino = app.stepHandler.connect(camera_index, port_list[0].device)
        except:
            pass
        self.connecting = False

    def connect_to_robot(self, *args):
        self.connecting = True
        threading.Thread( target=self.connect_to_robot_thread ).start()


class SelectVanScreen(Screen):
    def MercedesSprinter(self, *args):
        print("Mercedes Sprinter")
        van_select.append("Mercedes Sprinter")

    def VolkswagenCrafter(self, *args):
        print("Volkswagen Crafter")
        van_select.append("Volkswagen Crafter")

    def FordTransit(self, *args):
        print("Ford Transit")
        van_select.append("Ford Transit")

    def FiatDucato(self, *args):
        print("Fiat Ducato")
        van_select.append("Fiat Ducato")
        return van_select

    def RenaultMaster(self, *args):
        print("Renault Master")
        van_select.append("Renault Master")

    def RamProMaster(self, *args):
        print("Ram ProMaster")
        van_select.append("Ram ProMaster")

class SelectVanSizeScreen(Screen):
    def L3H2(self, *args):
        print("L3H2")
        print("dimensions")
        van_select.append("L3H2")

    def L2H2(self, *args):
        print("L2H2")
        print("dimensions")
        van_select.append("L2H2")

    def L2H1(self, *args):
        print("L2H1")
        print("dimensions")
        van_select.append("L2H1")

    def L1H1(self, *args):
        print("L1H1")
        print("dimensions")
        van_select.append("L1H1")

class SelectWindowSizeScreen(Screen):
    def Skylight(self, *args):
        print("SkyLight")
        print("x = 200 mm, y = 200 mm")
        van_select.append(200)
        van_select.append(200)

    def RoofVent(self, *args):
        print("RoofVent")
        print("x = 150 mm, y = 150mm")
        van_select.append(150)
        van_select.append(150)

    def FlushCurved(self, *args):
        print("Flush Curved")
        print("x = 280 mm, y = 280 mm")
        van_select.append(280)
        van_select.append(280)

    def FlushStraight(self, *args):
        print("FlushStraight")
        print("x = 300 mm, y = 300 mm")
        van_select.append(300)
        van_select.append(300)
        print(van_select)

        #testing
        selected_vans = (str(van_select)[1:-1])
        print('selected_vans ', selected_vans)
        return selected_vans

class LabelGrid(GridLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.init_table()

    def init_table(self):
        self.clear_widgets()
        self.no_of_rows = 0
        app = CuttingApp.get_running_app()
        for key, step in app.stepHandler.ordered_steps:
            if key == 15:
                break
            self.add_row([key], step['desc'])

    def add_row(self, step_no, step_desc):
        self.add_widget(StepLabel(text='  x  ', size_hint=(0.1, 0.2)))
        self.add_widget(StepLabel(text='Step', size_hint=(0.1, 0.2)))
        self.add_widget(StepLabel(text=str(step_no), size_hint=(0.1, 0.2)))
        self.add_widget(StepLabel(text=str(step_desc)))
        self.no_of_rows += 1

    def change_row(self, row_key=0, text='',elements_per_row=4):
        index = (self.no_of_rows-row_key-1)*4+3
        item: StepLabel = self.children[index]
        print('item', item)
        item.text = str(text)
        item.texture_update()

class StepLabel(Label):
    pass

class StepLabelStatus(Label):
    pass

class StartCalibrationScreen(Screen):
    stopped = BooleanProperty(False)
    calibrating = BooleanProperty(False)
    calibrated = BooleanProperty(False)

    def calibration_thread(self):
        app = CuttingApp.get_running_app()
        app.stepHandler.set_window( 0.10, 0.12, 0.40, 0.40 )
        label_grid = self.ids.step_table
        label_grid.init_table()

        print( label_grid.children )
        i = 0
        for key, step in app.stepHandler.ordered_steps:
            if key == 15: # step that wont be executed
                self.calibrated = True
                break
            label_grid.change_row( i, 'BUSY' )
            step['func']()
            label_grid.change_row( i, 'DONE' )
            i += 1
            if self.stopped:
                print('stop button pressed')
                break
        self.calibrating = False

    def start_calibration(self, *args):
        # run main digital world
        # turn on to test
        self.stopped = False
        if not self.calibrated:
            self.calibrating = True
            threading.Thread( target=self.calibration_thread ).start()


    def stop(self, *args):
        self.stopped = True

class StartCutScreen(Screen):
    moved_to_offset = BooleanProperty(True)
    safety_checked = BooleanProperty(True)
    cut_done = BooleanProperty(False)

    def move_offset_thread(self):
        app = CuttingApp.get_running_app()
        app.stepHandler.step15()
        self.moved_to_offset = False

    def move_offset(self,*args):
        threading.Thread( target=self.move_offset_thread ).start()

    def check_position(self, *args):
        app = CuttingApp.get_running_app()
        self.safety_checked = False

    def start_cut_thread(self):
        app = CuttingApp.get_running_app()
        app.stepHandler.step17()

    def start_cut(self, *args):
        threading.Thread( target=self.start_cut_thread ).start()
        # run window cut program
        self.cut_done = True

class ScreenManager(ScreenManager):
    pass

class CuttingApp( App ):
    stepHandler = main_digital_world.RobotSteps(plotting=False)

    def build(self):
        global van_select
        sm = ScreenManager()
        sm.add_widget(MainScreen())
        sm.add_widget(ConnectScreen())
        sm.add_widget(SelectVanScreen())
        sm.add_widget(SelectVanSizeScreen())
        sm.add_widget(SelectWindowSizeScreen())
        sm.add_widget(StartCalibrationScreen())
        sm.add_widget(StartCutScreen())

        Window.clearcolor = (1, 1, 1, 1)
        Window.fullscreen = True
        Config.set('input', 'mouse', 'mouse,multitouch_on_demand')
        return sm

if __name__ == "__main__":
    CuttingApp().run()

