# 基于RM65机械臂 手势控制 ROH-AP001

本项目基于睿尔曼生态协议实现，灵巧手需升级适配生态协议的固件后使用，请联系傲意售后获取。

## 准备

安装python和pip
进入命令环境，如windows下的command或者linux下的BASH
进入演示项目目录，例如：

```SHELL
cd gestrue_ctrled_ap001_on_rm65
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
ROH_ADDR = 2
```

运行：

```python
python gestrue_ctrled_ap001_on_rm65
```

按'q'退出。
