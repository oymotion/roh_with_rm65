# 基于RM65机械臂 手势控制 ROHand

## 准备

安装python和pip
进入命令环境，如windows下的command或者linux下的BASH
进入演示项目目录，例如：

```SHELL
cd gesture_ctrled_hand_on_rm65
```

安装依赖的python库：

```SHELL
pip install -r requirements.txt
```

## 运行

打开`gesture_ctrled_hand_on_rm65.py`并修改端口和设备地址，例如：

```python
ARM_IP = "192.168.1.18"
COM_PORT = 1
NODE_ID = 2
```

运行：

```python
python gesture_ctrled_hand_on_rm65.py
```

按'q'退出。
