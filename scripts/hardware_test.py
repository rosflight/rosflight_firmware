#!/usr/bin/env python
from __future__ import print_function

__author__ = "James Jackson"
__copyright__ = "Copyright 2017, ROSflight"
__license__ = "BSD-3"
__version__ = "0.1"
__maintainer__ = "James Jackson"
__email__ = "superjax08@gmail.com"

import time
import rospy
from sensor_msgs.msg import Imu, MagneticField
from rosflight_msgs.msg import Attitude, Barometer, OutputRaw, RCRaw, Status, Airspeed

RST  = "\x1B[0m"
KRED  = "\x1B[31m"
KMAG  = "\x1B[35m"
KGRN  = "\x1B[32m"
KYEL  = "\x1B[33m"
KBLU  = "\x1B[34m"

BOLD = "\x1B[1m"
UNDL = "\x1B[4m"

CLEAR = "\x1b[2K\r"
UPLINE = "\033[F"
STARTLINE = "\r"
END = "\x1b[200C"
CLEAR_SCREEN = "\033[2J + \033[0;0f"
def GO_UP(x): return "\x1b[" + str(x) + "A"


def GO_FORWARD(x): return "\033[" + str(x) + "C"

MSG_INFO = 1
MSG_WARN = 1
MSG_ERROR = 2
MSG_FATAL = 3

class Message():
    def __init__(self, message, level):
        self.msg = message
        self.lvl = level

    def __str__(self):
        date_string = time.strftime("%Y:%m:%d %H:%M:%S")
        string = RST + "[ " + date_string + " ] - "
        if self.lvl == MSG_INFO:
            string += self.msg
        elif self.lvl == MSG_WARN:
            string += KYEL + self.msg + RST
        elif self.lvl == MSG_ERROR:
            string += KRED + self.msg + RST
        elif self.lvl == MSG_FATAL:
            string += BOLD + KRED + self.msg + RST
        return string

    def __repr__(self):
        return self.__str__()

INIT = 0
GOOD = 1
STALLED = 2


class Stats():
    def __init__(self, size, name):
        self.max = [0.0 for i in range(size)]
        self.min = [1e9 for i in range(size)]
        self.t_diff = 0.0
        self.last_t = 0.0
        self.last_val = [0.0 for i in range(size)]
        self.data = {'t': [], 'val': []}
        self.name = name
        self.start = time.time()
        self.state = INIT

        self.m = [0 for i in range(size)]
        self.s = [0 for i in range(size)]
        self.k = [0 for i in range(size)]
        self.var = [0 for i in range(size)]

    def update(self, val, t, fields=None):
        if fields is not None:
            val = [getattr(val, field) for field in fields]
        elif not isinstance(val, list):
            val = [val]
        elif isinstance(val,list):
            val = val[:8]
        self.max = [max(internal, new) for internal, new in zip(self.max, val)]
        self.min = [min(internal, new) for internal, new in zip(self.max, val)]
        self.data['t'].append(t)
        self.data['val'].append(val)

        if val != self.last_val:
            self.last_val = val
            self.last_t = t
        self.t_diff = t - self.last_t


        if len(self.data['val']) > 10240:
            self.data['val'].pop(0)
            self.data['t'].pop(0)

        if self.state == INIT:
            print(Message(self.name + " found", MSG_INFO))
            self.state = GOOD
        elif self.state == GOOD and self.t_diff > 1.0:
            print(Message(self.name + " stalled out", MSG_ERROR))
            self.state = STALLED
        elif self.state == STALLED and self.t_diff < 1.0:
            print(Message(self.name + " recovered", MSG_INFO))
            self.state = GOOD





class Tester():
    def __init__(self):
        self.acc = Stats(3, 'acc')
        self.gyro = Stats(3, 'gyro')
        self.mag = Stats(3, 'mag')
        self.att = Stats(4, 'att')
        self.omega = Stats(3, 'omega')
        self.airspeed = Stats(1, 'airspeed')
        self.baro = Stats(1, 'baro')
        self.output = Stats(8, 'output')
        self.rc = Stats(8, 'rc')
        self.status = Stats(2, 'status')

        self.last_time = time.time()
        print(CLEAR_SCREEN, end='')
        print("===========  ROSFLIGHT HARDWARE CONTINUOUS INTEGRATION TEST ===========\n\n")
        print(Message("Start Test", MSG_INFO))

        self.run()


    def imu_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.acc.update(msg.linear_acceleration, t, fields=['x', 'y', 'z'])
        self.gyro.update(msg.angular_velocity, t, fields=['x', 'y', 'z'])

    def mag_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.mag.update(msg.magnetic_field, t, fields=['x','y','z'])

    def att_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.att.update(msg.attitude, t, fields=['w','x','y','z'])
        self.omega.update(msg.angular_velocity, t, fields=['x', 'y', 'z'])

    def baro_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.baro.update([msg.pressure], t)

    def airspeed_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.airspeed.update(msg.differential_pressure, t)

    def output_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.output.update(list(msg.values), t)

    def rc_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.rc.update(list(msg.values), t)

    def status_callback(self, msg):
        t = msg.header.stamp.to_sec()
        self.status.update(msg, t, fields=['num_errors', 'loop_time_us'])

    def print_progress(self):
        if not hasattr(self, 'start_time'):
            self.start_time = time.time()
            self.progress_index = 1
        progress_indicator = ['_', 'o', 'O', '0', 'O', 'o']
        if time.time() > (self.start_time + 0.12 * self.progress_index):
            self.progress_index += 1
            print(END + '%s%c%s' % (KGRN, progress_indicator[self.progress_index % len(progress_indicator)], RST),
                  end='\r')

    def got_msg(self, type):
        if hasattr(self, type) and len(getattr(self, type).data['val']) > 0:
            return True
        else:
            return False

    def print_status(self, type):
        if self.got_msg(type):
            print(RST+CLEAR, end='')
            string = type + "\r" + GO_FORWARD(8) + " -- | dt = {:10.4f} \t| ["
            values = [getattr(self,type).t_diff]
            for i, val in enumerate(getattr(self, type).data['val'][-1]):
                string += "{:10.4f}"
                values.append(val)
                if i != len(getattr(self, type).data['val'][-1]) -1:
                    string += ", "
            string += "]"
            print(string.format(*values), end='\n')
            return 1
        else:
            return 0


    def render(self):
        if time.time() > self.last_time + 0.03333:
            self.last_time = time.time()
            print("                                                                                                     ", end='')
            self.print_progress() # bouncy thing
            print("\n" + RST + CLEAR, end='\n')
            lines = 3
            lines += self.print_status('acc')
            lines += self.print_status('gyro')
            lines += self.print_status('mag')
            lines += self.print_status('att')
            lines += self.print_status('omega')
            lines += self.print_status('airspeed')
            lines += self.print_status('baro')
            # lines += self.print_status('output')
            # lines += self.print_status('rc')
            lines += self.print_status('status')
            print('\n', end='')
            print(RST + CLEAR, end='')
            print(GO_UP(lines), end='\r')


    def run(self):

        rospy.init_node("rosflight_hardware_test")
        self.att_sub = rospy.Subscriber('attitude', Attitude, self.att_callback)
        self.status_sub = rospy.Subscriber('status', Status, self.status_callback)
        self.rc_sub = rospy.Subscriber('rc_raw', RCRaw, self.rc_callback)
        self.output_sub = rospy.Subscriber('output_raw', OutputRaw, self.output_callback)
        self.baro_sub = rospy.Subscriber('baro', Barometer, self.baro_callback)
        self.mag_sub = rospy.Subscriber('mag', MagneticField, self.mag_callback)
        self.acc_sub = rospy.Subscriber('imu/data', Imu, self.imu_callback)
        self.pitot_sub = rospy.Subscriber('airspeed', Airspeed, self.airspeed_callback)

        start_time = time.time()

        while not rospy.is_shutdown():
            self.render()







if __name__ == '__main__':
    tester = Tester()