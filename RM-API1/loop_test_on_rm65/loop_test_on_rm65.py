import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.roh_registers_v1 import *
from common.robotic_arm import *

# Device filters
ARM_IP = "192.168.1.18"
COM_PORT = 1
ROH_ADDR = 2
NUM_FINGERS = 5
DELAY_TIME = 1.5

def write_registers(robot, address, values, node_id):
        length = len(values)
        values_bytes = []
        for i in range(length):
            values_bytes.append((values[i] >> 8) & 0xFF)  # High byte
            values_bytes.append(values[i] & 0xFF) # Low byte

        ret = robot.Write_Registers(COM_PORT, address, length, values_bytes, node_id, True)

        if ret != 0:
            print("Write_Registers error: ", ret)
            return False
        else:
            return True
        
def read_registers(robot, address, num, node_id):
        tag, ret = robot.Read_Multiple_Holding_Registers(COM_PORT, address, num, node_id)

        data = [0 for _ in range(num)]

        if tag != 0:
            print("Read_Registers error", tag)
            return None
        else:
            for i in range(num):
                data[i] = (ret[i]) | (ret[i + 1] << 8)
            return data

def main():
    # 机械臂初始化
    robot = Arm(RM65, ARM_IP)
    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(COM_PORT, 115200, 1, True)

    # Open all fingers
    write_registers(robot, ROH_FINGER_POS_TARGET0, [0, 0, 0, 0, 0], ROH_ADDR)
    time.sleep(DELAY_TIME)

    # Rotate thumb root to side
    write_registers(robot, ROH_FINGER_POS_TARGET5, [0], ROH_ADDR)
    time.sleep(DELAY_TIME)

    loop_time = 0

    while True:
        #
        # Close thumb then spread
        if not write_registers(robot, ROH_FINGER_POS_TARGET0, [65535], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        if not write_registers(robot, ROH_FINGER_POS_TARGET0, [0], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        #
        # Rotate thumb root

        if not write_registers(robot, ROH_FINGER_POS_TARGET5, [65535], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        if not write_registers(robot, ROH_FINGER_POS_TARGET5, [0], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        #
        # Close other fingers then spread

        if not write_registers(robot, ROH_FINGER_POS_TARGET1, [65535, 65535, 65535, 65535], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        target = read_registers(robot, ROH_FINGER_POS_TARGET0, NUM_FINGERS, ROH_ADDR)
        current = read_registers(robot, ROH_FINGER_POS0, NUM_FINGERS, ROH_ADDR)

        if target is not None and current is not None:
            print("target pos:{0}, current pos:{1}".format(target, current))

        if not write_registers(robot, ROH_FINGER_POS_TARGET1, [0, 0, 0, 0], ROH_ADDR):
            break
        time.sleep(DELAY_TIME)

        target = read_registers(robot, ROH_FINGER_POS_TARGET0, NUM_FINGERS, ROH_ADDR)
        current = read_registers(robot, ROH_FINGER_POS0, NUM_FINGERS, ROH_ADDR)

        if target is not None and current is not None:
            print("target pos:{0}, current pos:{1}".format(target, current))

        loop_time += 1
        print("\n Loop executed: \n", loop_time)

        if loop_time == 50:
            print("++++Loop over++++")
            break

if __name__ == "__main__":
    main()
