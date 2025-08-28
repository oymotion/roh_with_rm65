import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.robotic_arm import *
from common.roh_registers_v1 import *

# 机械臂配置
ARM_IP = "192.168.1.18"
COM_PORT = 1
ROH_ADDR = 2
ARM_SPEED = 30

POSE_MIDDLE = [90, -90, 90, 0, 0, 58]
POSE_LEFT = [90, -90, 105, 0, 0, 58]
POSE_RIGHT = [90, -90, 75, 0, 0, 58]

HAND_POS = [
    [65535,     0, 65535, 65535, 65535,     0], # 1
    [65535,     0,     0, 65535, 65535,     0], # 2
    [65535,     0,     0,     0, 65535,     0], # 3
    [65535,     0,     0,     0,     0,     0], # 4
    [    0,     0,     0,     0,     0,     0], # 5
    [    0, 65535, 65535, 65535,     0,     0], # 6
    [30000, 30000, 30000, 65535, 65535, 65535], # 7
    [    0,     0, 65535, 65535, 65535,     0], # 8
    [30000, 20000, 65535, 65535, 65535,     0], # 9
    [65535, 65535, 65535, 65535, 65535,     0]  # 10
]

def write_registers(robot, address, values, node_id, time_delay):
    length = len(values)
    values_bytes = []
    for i in range(length):
        values_bytes.append(values[i] & 0xFF) # Low byte
        values_bytes.append((values[i] >> 8) & 0xFF) # High byte

    ret = robot.Write_Registers(COM_PORT, address, length, values_bytes, node_id, True)
    
    if ret != 0:
        print("Write_Registers error: ", ret)
    time.sleep(time_delay)

count = 0

def main():
    # 机械臂初始化
    robot = Arm(RM65, ARM_IP)
    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(COM_PORT, 115200, 1, True)

    while True:
        write_registers(robot, ROH_FINGER_POS_TARGET0, HAND_POS[4], ROH_ADDR, 1)

        robot.Movej_Cmd(POSE_MIDDLE, ARM_SPEED, 0, True)
        robot.Movej_Cmd(POSE_RIGHT, ARM_SPEED, 0, True)
        robot.Movej_Cmd(POSE_LEFT, ARM_SPEED, 0, True)
        robot.Movej_Cmd(POSE_MIDDLE, ARM_SPEED, 0, True)
        time.sleep(2)
        
        for finger_pos in HAND_POS:
            write_registers(robot, ROH_FINGER_POS_TARGET0, finger_pos, ROH_ADDR, 5)
        
        for finger_pos in reversed(HAND_POS):
            write_registers(robot, ROH_FINGER_POS_TARGET0, finger_pos, ROH_ADDR, 5)
        
        count += 1
        print("run times:", count)

if __name__ == "__main__":
    main()