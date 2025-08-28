import socket
import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.robotic_arm import *
from common.roh_registers_v1 import *
from arm_pose import *

# 机械臂配置
ARM_IP = "192.168.1.18"
COM_PORT = 1
ROH_ADDR = 2
ARM_SPEED = 60  # 机械臂速度/%
DELAY_TIME = 0.5  # 延时时间/s

RIGHT_HAND = 0
LEFT_HAND = 1

HAND_SELECTED = RIGHT_HAND #选择左手或者右手

target_fingers = [
    ROH_FINGER_POS_TARGET0,
    ROH_FINGER_POS_TARGET1,
    ROH_FINGER_POS_TARGET2,
    ROH_FINGER_POS_TARGET3,
    ROH_FINGER_POS_TARGET4,
    ROH_FINGER_POS_TARGET5,
]

def write_registers(robot, address, values, node_id, time_delay):
    length = len(values)
    values_bytes = []
    for i in range(length):
        values_bytes.append((values[i] >> 8) & 0xFF) # High byte
        values_bytes.append(values[i] & 0xFF) # Low byte


    ret = robot.Write_Registers(COM_PORT, address, length, values_bytes, node_id, True)
    
    if ret != 0:
        print("Write_Registers error: ", ret)
    time.sleep(time_delay)

def move_ball(robot, from_pos, to_pos):
    robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[from_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_READY_GRASP_BALL_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[from_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_GRASP_BALL[from_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_GRASP_BALL_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[from_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[from_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[to_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[to_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_GRASP_BALL[to_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_READY_GRASP_BALL_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BALL_HIGH[to_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_INITIAL_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BALL_BACK_HIGH[to_pos], ARM_SPEED, 0, True)


def move_bottle(robot, from_pos, to_pos):
    robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[from_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_READY_GRASP_BOTTLE_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_GRASP_BOTTLE[from_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_GRASP_BOTTLE_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BOTTLE_HIGH[from_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[from_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[to_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_LIFT_BOTTLE_HIGH[to_pos], ARM_SPEED, 0, True)
    robot.Movel_Cmd(POSE_GRASP_BOTTLE[to_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_READY_GRASP_BOTTLE_POS, ROH_ADDR, DELAY_TIME)

    robot.Movel_Cmd(POSE_LIFT_BOTTLE_BACK_HIGH[to_pos], ARM_SPEED, 0, True)

    write_registers(robot, target_fingers[0], HAND_INITIAL_POS, ROH_ADDR, DELAY_TIME)

def main():

    # 检查是否连接
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try: 
            s.connect((ARM_IP, 8080))
            s.close()
            break
        except socket.error:
            print("connect time out, try again")

        time.sleep(1)
        
    # 机械臂初始化
    robot = Arm(RM65, ARM_IP)
    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(COM_PORT, 115200, 1, True)

    # 选择手的方向
    set_handSelected(HAND_SELECTED)

    while True:
        write_registers(robot, target_fingers[0], HAND_INITIAL_POS, ROH_ADDR, DELAY_TIME)

        robot.Movej_Cmd(POSE_INITIAL, 30, 0, True)
        robot.Movej_Cmd(POSE_DANCE, 30, 0, True)  # 手指舞角度

        # 手指依次动
        for target in reversed(target_fingers):
            write_registers(robot, target, MAX_SINGLE_FINGER_POS, ROH_ADDR, DELAY_TIME)

        for target in target_fingers:
            write_registers(robot, target, MIN_SINGLE_FINGER_POS, ROH_ADDR, DELAY_TIME)

        # 握拳1
        write_registers(robot, target_fingers[5], MAX_SINGLE_FINGER_POS, ROH_ADDR, DELAY_TIME)
        write_registers(robot, target_fingers[0], HAND_FIST_CLOSE_POS, ROH_ADDR, DELAY_TIME)
        write_registers(robot, target_fingers[0], MAX_SINGLE_FINGER_POS, ROH_ADDR, 1)

        # 握拳2
        write_registers(robot, target_fingers[0], MIN_SINGLE_FINGER_POS, ROH_ADDR, DELAY_TIME)
        write_registers(robot, target_fingers[0], HAND_FIST_OPEN_POS, ROH_ADDR, DELAY_TIME)
        time.sleep(0.5)

        write_registers(robot, target_fingers[0], HAND_FIST_CLOSE_POS, ROH_ADDR, DELAY_TIME)
        write_registers(robot, target_fingers[0], MAX_SINGLE_FINGER_POS, ROH_ADDR, 1)
        write_registers(robot, target_fingers[0], MIN_SINGLE_FINGER_POS, ROH_ADDR, 1)
        write_registers(robot, target_fingers[0], HAND_FIST_OPEN_POS, ROH_ADDR, DELAY_TIME)
        write_registers(robot, target_fingers[5], MIN_SINGLE_FINGER_POS, ROH_ADDR, DELAY_TIME)

        time.sleep(1)

        # initial state: bottle @ middle, ball @ left 

        move_bottle(robot, POS_MIDDLE, POS_RIGHT)  # bottle -> right, ball @ left
        time.sleep(DELAY_TIME)

        move_ball(robot, POS_LEFT, POS_MIDDLE)     # bottle @ right, ball -> middle
        time.sleep(DELAY_TIME)

        move_bottle(robot, POS_RIGHT, POS_LEFT)    # bottle -> left, ball @ middle
        time.sleep(DELAY_TIME)

        move_ball(robot, POS_MIDDLE, POS_RIGHT)    # bottle @ left, ball -> right
        time.sleep(DELAY_TIME)

        move_bottle(robot, POS_LEFT, POS_MIDDLE)   # bottle -> middle, ball @ right
        time.sleep(DELAY_TIME)

        move_ball(robot, POS_RIGHT, POS_LEFT)     # bottle @ middle, ball -> left
        time.sleep(DELAY_TIME)

        robot.Movej_Cmd(POSE_VICTORY[POS_MIDDLE], 30, 0, True)  # 比耶

        write_registers(robot, target_fingers[0], HAND_VICTORY_POS, ROH_ADDR, DELAY_TIME)

        robot.Movej_Cmd(POSE_VICTORY[POS_LEFT], ARM_SPEED, 0, True)  # 比耶左
        robot.Movej_Cmd(POSE_VICTORY[POS_RIGHT], ARM_SPEED, 0, True)  # 比耶右

if __name__ == "__main__":
    main()