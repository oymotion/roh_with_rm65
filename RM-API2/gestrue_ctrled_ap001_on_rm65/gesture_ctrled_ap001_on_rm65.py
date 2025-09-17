import os
import cv2
import sys
import queue
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from common.roh_registers_v2 import *
from common.robotic_arm import *
from HandTrackingModule import HandDetector

# Robot arm configuration
ARM_IP = "192.168.1.18"
COM_PORT = 1
ROH_ADDR = 2

NUM_FINGERS = 5
NUM_MOTORS = 6
JOINT_READY = [170, 0, -90, 0, 90, -12]

# Finger force params display position in video window
FINGER_LABELS = ["Thumb", "Index", "Middle", "Ring", "Little", "Palm"]
FINGER_FORCE_SUM_NAME_POS = [(530, 25), (530, 45), (530, 65), (530, 85), (530, 105), (530, 125)]
FINGER_FORCE_SUM_VALUE_POS = [(590, 25), (580, 45), (590, 65), (575, 85), (580, 105), (580, 125)]

#
# Camera initialization
gesture_queue = queue.Queue(maxsize=NUM_FINGERS)
image_queue = queue.Queue(maxsize=1)
file_path = os.path.abspath(os.path.dirname(__file__))
detector = HandDetector(maxHands=1, detectionCon=0.8)
video = cv2.VideoCapture(0)

# Get the resolution of the camera
width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))  # Camera frame width
height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))  # Camera frame height
detector = HandDetector(maxHands=1, detectionCon=0.8)
data_queue = queue.Queue(maxsize=5)

# Create a window with adjustable size
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
# Set window size to camera resolution
cv2.resizeWindow("Video", width, height)


def read_registers(robot, address, num, node_id):
    """
    Read data from device registers

    Args:
        robot: Instance of Robot Arm Controller
        address: Register address
        num: The register num to be read
        node_id: Node id

    Returns:
        Return the data list if read success
    """
    # Create read-write register structure
    read_params = rm_peripheral_read_write_params_t()
    read_params.port = COM_PORT
    read_params.device = node_id
    read_params.address = address
    read_params.num = num

    tag, ret = robot.Read_Registers(read_params)

    data = [0 for _ in range(num)]

    if tag != 0:
        print("Read_Registers error: ", tag)
        return None
    else:
        for i in range(num):
            data[i] = (ret[i]) | (ret[i + 1] << 8)
        return data


def write_registers(robot, address, values, node_id):
    """
    Write data to the device register.

    Args:
        robot: Instance of Robot Arm Controller
        address: Register address
        values: The value to be written
        node_id: Node id

    Returns:
        Return False if write fails
    """
    # Create read-write register structure
    write_params = rm_peripheral_read_write_params_t()
    write_params.port = COM_PORT
    write_params.device = node_id
    write_params.address = address

    length = len(values)
    write_params.num = length

    values_bytes = []
    for i in range(length):
        values_bytes.append((values[i] >> 8) & 0xFF)  # High byte
        values_bytes.append(values[i] & 0xFF)  # Low byte

    ret = robot.Write_Registers(write_params, values_bytes)

    if ret != 0:
        print("Write_Registers error: ", ret)


def camera_thread():
    """
    Camera thread function: Capture video frames, detect gestures, and queue gestures and images
    """
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

        if gesture[1] == 65535 and gesture[5] == 65535 and gesture[0] == 45000:
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


def data_reading(robot):
    """
    Data reading thread function: Read the force sensor data of the robotic arm and put it in the queue
    """
    finger_force_sum = [0 for _ in range(NUM_MOTORS)]
    while True:
        try:
            for i in range(NUM_MOTORS):
                for key, value in robot.Get_State_info()[1].items():
                    if key == "force":
                        finger_force_sum[i] = value[i]

            if not data_queue.full():
                data_queue.put(finger_force_sum)

        except Exception as e:
            print(f"Data reading thread err: {e}")


def main():
    # Arm initial
    robot = RobotArmController(ARM_IP, 8080, 3)

    # set plus mode and check communication
    set_plus_mode_res = robot.Set_Plus_Mode()
    print(f"Set_Plus_Mode: {set_plus_mode_res}")
    get_plus_base_info_res = robot.Get_Base_info()
    print(f"get_plus_base_info_res: {get_plus_base_info_res}")

    force_sensor = False
    if get_plus_base_info_res[1]["force"] == True:
        force_sensor = True

    prev_gesture = [0, 0, 0, 0, 0, 0]
    robot.Movej_Cmd(JOINT_READY, 30, 0, 0, 1)

    # Start camera thread
    threading.Thread(target=camera_thread, daemon=True).start()

    if force_sensor:
        threading.Thread(target=data_reading, args=(robot,), daemon=True).start()

    while True:
        gesture = gesture_queue.get()

        if not image_queue.empty():
            img = image_queue.get()
            cv2.putText(img, "Try with gestures", (16, 272), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)

            if force_sensor:
                # draw force name
                for force_index in range(NUM_MOTORS):
                    name = f"{FINGER_LABELS[force_index]}: "
                    cv2.putText(img, name, FINGER_FORCE_SUM_NAME_POS[force_index], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

                # draw force value
                if not data_queue.empty():
                    finger_force_sum = data_queue.get()
                    for force_index in range(NUM_MOTORS):
                        value = f"{finger_force_sum[force_index]}"
                        cv2.putText(
                            img, value, FINGER_FORCE_SUM_VALUE_POS[force_index], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
                        )

            # refresh window
            cv2.imshow("Video", img)

        if prev_gesture != gesture:
            robot.Set_Hand_Follow_pos(gesture, False)
            prev_gesture = gesture

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
