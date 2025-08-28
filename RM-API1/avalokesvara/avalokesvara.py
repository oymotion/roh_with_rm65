import asyncio
import os
import signal
import sys
import time
import keyboard

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.roh_registers_v1 import *
from common.robotic_arm import *

# Device filters
NUM_FINGERS = 5
COM_PORT = 1
BAUD_RATE = 115200
TIMEOUT = 1
ROH_ADDR = 2

# parameters
ARM_SPEED_LOW = 20
ARM_SPEED_HIGH = 50
TIME_DELAY = 0.4

GESTURES = {
    'REST':     [    0,     0,     0,     0,    0,     0], 
    'OK':       [44000, 29000,     0,     0,    0, 60000], 
    'OK_PLUS':  [30000, 37000, 12000, 10000, 8000, 62000], 
    'SIDE_PALM':[65535,     0,     0,     0,    0,     0]
}

ARM_L_0DEG = [-90.0, -90.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_0DEG = [90.0, 90.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_15DEG = [-90.0, -75.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_15DEG = [90.0, 75.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_30DEG = [-90.0, -60.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_30DEG = [90.0, 60.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_45DEG = [-90.0, -45.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_45DEG = [90.0, 45.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_60DEG = [-90.0, -30.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_60DEG = [90.0, 30.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_90DEG = [-90.0, 0.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_90DEG = [90.0, 0.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_120DEG = [-90.0, 30.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_120DEG = [90.0, -30.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_135DEG = [-90.0, 45.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_135DEG = [90.0, -45.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_150DEG = [-90.0, 60.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_150DEG = [90.0, -60.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_180DEG = [-90.0, 90.0, 0.0, 0.0, 0.0, -350.326]
ARM_R_180DEG = [90.0, -90.0, 0.0, 0.0, 0.0, -196.383]

ARM_L_STEP1 = [-90.914, -89.044, -1.065, 0.877, 3.076, -350.326]
ARM_L_STEP2 = [-92.999, -80.406, 6.243, -84.973, 6.421, -350.326]
ARM_L_STEP3 = [-93.051, -48.529, 6.254, -84.986, 6.549, -350.326]
ARM_L_STEP4 = [-94.346, -3.567, 7.562, -84.998, 6.38, -350.326]
ARM_L_STEP5 = [-83.649, 54.562, -3.842, -85.071, 8.517, -350.326]
ARM_L_STEP6 = [-89.471, 94.941, -3.383, -84.972, 6.021, -350.326]
ARM_L_STEP7 = [-62.119, 84.843, 32.272, -144.453, 40.433, -350.326]
ARM_L_STEP8 = [-15.173, 78.409, 72.589, -102.365, 84.425, -350.326]
ARM_L_STEP9 = [37.989, 96.037, 61.692, -48.383, 101.88, -350.326]

ARM_R_STEP1 = [ 90.914,  89.044,  1.065,  -0.877, -3.076, -196.383]
ARM_R_STEP2 = [ 92.999,  80.406, -6.243,  84.973, -6.421, -196.383]
ARM_R_STEP3 = [ 93.051,  48.529, -6.254,  84.986, -6.549, -196.383]
ARM_R_STEP4 = [ 94.346,   3.567, -7.562,  84.998, -6.38, -196.383]
ARM_R_STEP5 = [ 83.649, -54.562, 3.842,   85.071, -8.517, -196.383]
ARM_R_STEP6 = [ 89.471, -94.941, 3.383,   84.972, -6.021, -196.383]
ARM_R_STEP7 = [ 62.119, -84.843, -32.272, 144.453, -40.433, -196.383]
ARM_R_STEP8 = [ 15.173, -78.409, -72.589, 102.365, -84.425, -196.383]
ARM_R_STEP9 = [-37.989, -96.037, -61.692, 48.383, -101.88, -196.383]

# ROBOT_IP
ROBOT1_L_IP = "192.168.1.3"
ROBOT1_R_IP = "192.168.1.4"
ROBOT2_L_IP = "192.168.1.5"
ROBOT2_R_IP = "192.168.1.6"
ROBOT3_L_IP = "192.168.1.7"
ROBOT3_R_IP = "192.168.1.8"
ROBOT4_L_IP = "192.168.1.9"
ROBOT4_R_IP = "192.168.1.10"
ROBOT5_L_IP = "192.168.1.11"
ROBOT5_R_IP = "192.168.1.12"

robot1_l = Arm(RM65, ROBOT1_L_IP)
robot1_r = Arm(RM65, ROBOT1_R_IP)
robot2_l = Arm(RM65, ROBOT2_L_IP)
robot2_r = Arm(RM65, ROBOT2_R_IP)
robot3_l = Arm(RM65, ROBOT3_L_IP)
robot3_r = Arm(RM65, ROBOT3_R_IP)
robot4_l = Arm(RM65, ROBOT4_L_IP)
robot4_r = Arm(RM65, ROBOT4_R_IP)
robot5_l = Arm(RM65, ROBOT5_L_IP)
robot5_r = Arm(RM65, ROBOT5_R_IP)

robot1_l.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot1_r.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot2_l.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot2_r.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot3_l.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot3_r.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot4_l.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot4_r.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot5_l.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)
robot5_r.Set_Modbus_Mode(COM_PORT, BAUD_RATE, TIMEOUT, False)


class Application:

    def __init__(self):
        signal.signal(signal.SIGINT, lambda signal, frame: self._signal_handler())
        self.terminated = False

    def _signal_handler(self):
        print("You pressed ctrl-c, exit")
        self.terminated = True

    def write_registers(self, robot, address, values, node_id, time_delay):
        length = len(values)
        values_bytes = []
        for i in range(length):
            values_bytes.append((values[i] >> 8) & 0xFF)  # High byte
            values_bytes.append(values[i] & 0xFF) # Low byte


        ret = robot.Write_Registers(COM_PORT, address, length, values_bytes, node_id, True)
        
        if ret != 0:
            print("Write_Registers error: ", ret)
        time.sleep(time_delay)

    # 所有臂单步
    def group1_single_step_all(self, pos_l, pos_r, speed, r, time_delay):
        robot1_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot1_r.Movej_Cmd(pos_r, speed, 0, r, False)
        time.sleep(time_delay)

        robot2_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot2_r.Movej_Cmd(pos_r, speed, 0, r, False)
        time.sleep(time_delay)

        robot3_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot3_r.Movej_Cmd(pos_r, speed, 0, r, False)
        time.sleep(time_delay)

        robot4_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot4_r.Movej_Cmd(pos_r, speed, 0, r, False)
        time.sleep(time_delay)

        robot5_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot5_r.Movej_Cmd(pos_r, speed, 0, r, False)
        time.sleep(time_delay)

    # 莲花上
    def group1_up(self):
        self.group1_single_step_all(ARM_L_STEP1, ARM_R_STEP1, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP2, ARM_R_STEP2, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP3, ARM_R_STEP3, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP4, ARM_R_STEP4, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP5, ARM_R_STEP5, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP6, ARM_R_STEP6, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP7, ARM_R_STEP7, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP8, ARM_R_STEP8, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP9, ARM_R_STEP9, ARM_SPEED_LOW, 40, TIME_DELAY)

    # 莲花下
    def group1_down(self):
        self.group1_single_step_all(ARM_L_STEP9, ARM_R_STEP9, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP8, ARM_R_STEP8, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP7, ARM_R_STEP7, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP6, ARM_R_STEP6, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP5, ARM_R_STEP5, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP4, ARM_R_STEP4, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP3, ARM_R_STEP3, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP2, ARM_R_STEP2, ARM_SPEED_LOW, 40, TIME_DELAY)
        self.group1_single_step_all(ARM_L_STEP1, ARM_R_STEP1, ARM_SPEED_LOW, 40, TIME_DELAY)

    # 单臂单步
    def single_step(self, robot_l, robot_r, pos_l, pos_r, speed, r):
        robot_l.Movej_Cmd(pos_l, speed, 0, r, False)
        robot_r.Movej_Cmd(pos_r, speed, 0, r, False)
        
    # 不同角度上升
    def group1_open(self):
        self.single_step(robot1_l, robot1_r, ARM_L_0DEG, ARM_R_0DEG, ARM_SPEED_HIGH, 40)
        self.single_step(robot2_l, robot2_r, ARM_L_45DEG, ARM_R_45DEG, ARM_SPEED_HIGH, 40)
        self.single_step(robot3_l, robot3_r, ARM_L_90DEG, ARM_R_90DEG, ARM_SPEED_HIGH, 40)
        self.single_step(robot4_l, robot4_r, ARM_L_135DEG, ARM_R_135DEG, ARM_SPEED_HIGH, 40)
        self.single_step(robot5_l, robot5_r, ARM_L_180DEG, ARM_R_180DEG, ARM_SPEED_HIGH, 40)

    # 左升
    def group2_l_up(self):
        robot1_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot2_l.Movej_Cmd(ARM_L_45DEG, ARM_SPEED_LOW, 0, 40, False)
        robot3_l.Movej_Cmd(ARM_L_90DEG, ARM_SPEED_LOW, 0, 40, False)
        robot4_l.Movej_Cmd(ARM_L_135DEG, ARM_SPEED_LOW, 0, 40, False)
        robot5_l.Movej_Cmd(ARM_L_180DEG, ARM_SPEED_LOW, 0, 40, False)

    # 左降
    def group2_l_down(self):
        robot1_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot2_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot3_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot4_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot5_l.Movej_Cmd(ARM_L_0DEG, ARM_SPEED_LOW, 0, 40, False)

    # 右升
    def group2_r_up(self):
        robot1_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot2_r.Movej_Cmd(ARM_R_45DEG, ARM_SPEED_LOW, 0, 40, False)
        robot3_r.Movej_Cmd(ARM_R_90DEG, ARM_SPEED_LOW, 0, 40, False)
        robot4_r.Movej_Cmd(ARM_R_135DEG, ARM_SPEED_LOW, 0, 40, False)
        robot5_r.Movej_Cmd(ARM_R_180DEG, ARM_SPEED_LOW, 0, 40, False)

    # 右降
    def group2_r_down(self):
        robot1_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot2_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot3_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot4_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)
        robot5_r.Movej_Cmd(ARM_R_0DEG, ARM_SPEED_LOW, 0, 40, False)

    def move_all_hand(self, values):
        self.write_registers(robot1_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot1_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot2_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot2_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot3_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot3_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot4_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot4_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot5_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot5_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)

    def move_left_hand(self, values):
        self.write_registers(robot1_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot2_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot3_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot4_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot5_l, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)

    def move_right_hand(self, values):
        self.write_registers(robot1_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot2_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot3_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot4_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)
        self.write_registers(robot5_r, ROH_FINGER_POS_TARGET0, values, ROH_ADDR, 0)

    def show_finger(self, robot, loop_time):
        while loop_time:
            self.write_registers(robot, ROH_FINGER_POS_TARGET0, [45000], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET1, [65535], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET2, [65535], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET3, [65535], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET4, [65535], ROH_ADDR, 0.08)

            self.write_registers(robot, ROH_FINGER_POS_TARGET0, [0], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET1, [0], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET2, [0], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET3, [0], ROH_ADDR, 0.03)
            self.write_registers(robot, ROH_FINGER_POS_TARGET4, [0], ROH_ADDR, 0.08)

            loop_time -= 1  

    async def main(self):
        # Initial position
        self.group1_single_step_all(ARM_L_STEP1, ARM_R_STEP1, ARM_SPEED_LOW, 0, 0)
        self.move_all_hand(GESTURES['REST'])

        print('Press space to continue')  
        keyboard.wait('space')

        # 莲花上
        self.group1_up()

        time.sleep(1)
        print('Press space to continue')
        keyboard.wait('space')

        # 莲花下
        self.group1_down()
        # time.sleep(1) 
        print('Press space to continue')
        keyboard.wait('space')

        # 上升
        self.group1_open()
        self.move_all_hand(GESTURES['OK'])
        # time.sleep(1)
        print('Press space to continue')
        keyboard.wait('space')

        # 下降
        self.move_all_hand(GESTURES['REST'])
        self.group1_single_step_all(ARM_L_0DEG, ARM_R_0DEG, ARM_SPEED_HIGH, 40, 0)
        # time.sleep(1)
        print('Press space to continue')
        keyboard.wait('space')

        # 左升
        self.group2_l_up()
        # time.sleep(1)
        print('Press space to continue')
        keyboard.wait('space')

        self.move_left_hand(GESTURES['OK_PLUS'])
        time.sleep(1)

        print('Press space to continue')
        keyboard.wait('space')

        # 左降
        self.group2_l_down()
        self.move_left_hand(GESTURES['REST'])
        print('Press space to continue')
        keyboard.wait('space')

        # 右升
        self.group2_r_up()
        # time.sleep(2)
        print('Press space to continue')
        keyboard.wait('space')

        self.move_right_hand(GESTURES['OK_PLUS'])
        time.sleep(1)

        print('Press space to continue')
        keyboard.wait('space')

        # 右降
        self.group2_r_down()
        self.move_right_hand(GESTURES['REST'])

        print('Press space to continue')
        keyboard.wait('space')


if __name__ == "__main__":
    app = Application()
    while True:
        asyncio.run(app.main())
