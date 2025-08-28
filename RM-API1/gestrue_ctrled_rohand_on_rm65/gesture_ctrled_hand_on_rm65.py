import os
import cv2
import sys
import time
import queue
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.roh_registers_v1 import *
from common.robotic_arm import *
from HandTrackingModule import HandDetector

# 机械臂配置
ARM_IP = "192.168.1.18"
COM_PORT = 1
ROH_ADDR = 2
NUM_FINGERS = 5
JOINT_READY = [90, 0, -90, 0, 90, -12]

#
# 摄像头初始化
gesture_queue = queue.Queue(maxsize=NUM_FINGERS)
image_queue = queue.Queue(maxsize=1)
file_path = os.path.abspath(os.path.dirname(__file__))
detector = HandDetector(maxHands=1, detectionCon=0.8)
video = cv2.VideoCapture(0) 

# 获取摄像头的分辨率
width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))  # 摄像头帧宽度
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 摄像头帧高度

detector = HandDetector(maxHands=1, detectionCon=0.8)

# 创建可调整大小的窗口
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
# 设置窗口大小为摄像头分辨率
cv2.resizeWindow("Video", width, height)

def write_registers(robot, address, values, node_id, time_delay):
        length = len(values)
        values_bytes = []
        for i in range(length):
            values_bytes.append((values[i] >> 8) & 0xFF)  # High byte
            values_bytes.append(values[i] & 0xFF) # Low byte

        ret = robot.Write_Registers(COM_PORT, address, length, values_bytes, node_id, True)

        if ret != 0:
            print("Write_Registers error: ", ret)
        time.sleep(time_delay)

def camera_thread():
    timer = 0
    interval = 10
    original_gesture0 = 0
    while True:
        _, img = video.read()
        img = cv2.flip(img, 1)
        hand = detector.findHands(img, draw=True)
        gesture_pic = cv2.imread(file_path + "/gestures/unknown.png")
        gesture = [45000, 65535, 65535, 65535, 65535, 65535]

        if hand:
            lmlist = hand[0]

            if lmlist and lmlist[0]:
                try:
                    finger_up = detector.fingersUp(lmlist[0])

                    for i in range(len(finger_up)):
                        gesture[i] = int(gesture[i] * (1 - finger_up[i]))

                except Exception as e:
                    print(str(e))

                if finger_up[:5] == [0, 0, 0, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/0.png")
                elif finger_up[:5] == [0, 1, 0, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/1.png")
                elif finger_up[:5] == [0, 1, 1, 0, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/2.png")
                elif finger_up[:5] == [0, 1, 1, 1, 0]:
                    gesture_pic = cv2.imread(file_path + "/gestures/3.png")
                elif finger_up[:5] == [0, 1, 1, 1, 1]:
                    gesture_pic = cv2.imread(file_path + "/gestures/4.png")
                elif finger_up[:5] == [1, 1, 1, 1, 1]:
                    gesture_pic = cv2.imread(file_path + "/gestures/5.png")
            else:
                gesture = [0, 0, 0, 0, 0, 0]

        if gesture_pic.any():
            gesture_pic = cv2.resize(gesture_pic, (161, 203))
            img[0:203, 0:161] = gesture_pic

        if(gesture[1] == 65535 and gesture[5] == 65535 and gesture[0] == 45000):
            if timer == 0:
                original_gesture0 = gesture[0]
            timer += 1

            if timer <= interval:
                gesture[0] = 0
            else:
                gesture[0] = original_gesture0
        else:
            if timer > 0:
                gesture[0] = original_gesture0
            timer = 0

        if not gesture_queue.full():
            gesture_queue.put(gesture)
        if not image_queue.full():
            image_queue.put(img)

def main():
    # 机械臂初始化
    robot = Arm(RM65, ARM_IP)
    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(COM_PORT, 115200, 1, True)

    prev_gesture = [0, 0, 0, 0, 0, 0]
    # 运动到准备姿态
    robot.Movej_Cmd(JOINT_READY, 30, 0, True)

    last_time = time.time()
    # 启动摄像头线程
    threading.Thread(target=camera_thread, daemon=True).start()

    while True:
        gesture = gesture_queue.get()
        if not image_queue.empty():
            img = image_queue.get()
            cv2.putText(img, "Try with gestures", (16, 272), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 5)
            cv2.imshow("Video", img)

        if (prev_gesture != gesture):
            # 当手势变化时间小于0.7秒时，拇指保持张开
            current_time = time.time()
            if (current_time - last_time < 0.7):
                gesture[0] = 0
            else:
                last_time = current_time

            write_registers(robot, ROH_FINGER_POS_TARGET0, gesture, ROH_ADDR, 0)
            prev_gesture = gesture

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
