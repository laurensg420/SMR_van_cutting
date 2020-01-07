import serial
import serial.tools.list_ports
import time


class Arduino( object ):
    def __init__(self, com="COM8", baud=9600):
        # Declare size window and the IR stopping distance
        self.delta_x = 0.2
        self.delta_z = -0.3
        self.x_off = 0.1
        self.z_off = -0.2
        self.safe_IR_distance = 0.3
        self.baud = baud
        self.port = com
        self.speed_of_sound = 340.0  # m/s
        self.connect()
        self._connected = False

    def connect(self):
        # Declare the port and baudrate for the Arduino
        self.serial = serial.Serial( self.port, self.baud, timeout=1 )
        time.sleep( 1 )
        if self.get_duration_top == 0:
            raise Exception( "Couldn't get duration top from Arduino" )
        if self.get_duration_side == 0:
            raise Exception( "Couldn't get duration side from Arduino" )
        self._connected = True
        return self.connected

    @staticmethod
    def list_ports():
        ports = list( serial.tools.list_ports.comports() )
        return ports

    @property
    def connected(self):
        return self._connected

    def extend(self):
        self.serial.write( b'd' )
        time.sleep( 0.8 )

    def collapse(self):
        self.serial.write( b'c' )
        time.sleep( 0.8 )

    def get_duration(self, ultrasonic_index):
        if ultrasonic_index == 0:
            self.serial.write( b'a' )
        elif ultrasonic_index == 1:
            self.serial.write( b'b' )
        else:
            raise Exception( "Didn't recognize ultrasonic index of {}".format( ultrasonic_index ) )
        self.serial.flush()
        duration = self.serial.readline().decode( "utf-8" ).strip()  # Gets duration of ultrasonic sensor [us]
        try:
            duration = int( duration[14:] )
        except:
            duration = 0
        return duration

    def get_duration_top(self):
        return self.get_duration( 0 )

    duration_top = property( get_duration_top )

    def get_duration_side(self):
        return self.get_duration( 1 )

    duration_side = property( get_duration_side )

    def _get_distance(self, ultrasonic_index):  # Read the value from the Arduino serial and return the value to read it
        distance = self.get_duration( ultrasonic_index ) * (0.340 / 2.0)  # [mm]
        return distance  # return distance in [mm]

    def get_distance_top(self):
        return self._get_distance( 0 )

    distance_top = property( get_distance_top )

    def get_distance_side(self):
        return self._get_distance( 1 )

    distance_side = property( get_distance_side )

    def get_multiple_average(self, function, i=1):
        a = []
        j = 0
        k = 0
        for i in range( 0, i ):
            value = function()
            a.append( value )
            # print(value)
            if value > 2000:
                j += 1
            if j >= 10:
                print('break of arduino multiple average')
                break

            if value == 0:
                k += 1
            if k >= 10:
                raise Exception("Ultrasonic not connected: {}".format(function))

        average = sum( a ) / len( a )
        print( "Distance measurement: ", average, function )
        return average
