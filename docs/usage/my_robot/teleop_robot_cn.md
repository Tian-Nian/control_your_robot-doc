# 机械臂遥操

目前本项目给出的遥操示例都是面向于机械臂的, 经过测试直接复制模块接入人性机器人进行遥操是没问题的, 但是由于人性机器人代码大部份都较为闭源难以兼容, 因此没有集成进项目.

## 为什么机械臂遥操要单独列出?

对于机械臂运动, 由于遥操给出的目标坐标总是坐标较为连续, 并且控制频率较高, 因此部分机械臂会提供专门的遥操接口来保证遥操的同步性, 并且由于IK求解有强约束与弱约束两种方式(目标位置的准确度、当前机械臂各关节运动速度、幅度等约束条件), 对于基础的IK接口, 很可能出现位置不可达或者关节卡死的情况, 这就需要使用专门的弱约束IK来提升操控的体感.

## 快速实现一个遥操

需要注意, 遥操需要你自带一个遥操设备, 可以是QuestVR, agilex Pika, Apple Vision Pro等, 只要能获取相对运动信息的设备都可以, 甚至你可以基于IMU传感器DIY一个设备用来遥操.

首先你要在注册机器人的时候在传感器中绑定你的遥操设备, 以Pika为例:

``` python
def __init__(self, ...):
    self.sensors = {
            "image": {
                "cam_head": RealsenseSensor("cam_head"),
                "cam_left_wrist": RealsenseSensor("cam_left_wrist"),
                "cam_right_wrist": RealsenseSensor("cam_right_wrist"),
            }, 
            "teleop":{
                "pika_left": PikaRosSensor("left_pika"),
                "pika_right": PikaRosSensor("right_pika"),
            }
        }
```

然后需要在`set_up()`中初始化你的设备:
``` python
def set_up(self):
    ...
    self.sensors["teleop"]["pika_left"].set_up("/pika_pose_l","/gripper_l/joint_states")
    self.sensors["teleop"]["pika_right"].set_up("/pika_pose_r","/gripper_r/joint_states")

    self.set_collect_type({...,
                            "teleop": ["end_pose", "gripper"],
                            })

```

最后写一个main函数用于执行遥操, 或者直接在主函数中编写:

``` python
if __name__ == "__main__":
    import time

    # 只有使用了ros来控制才需要初始化一个ros节点
    import rospy
    rospy.init_node("rm_controller_node", anonymous=True)

    robot = MyRobot()
    robot.set_up()

    robot.reset()
    time.sleep(3)

    # 等待数据稳定
    while True:
        data = robot.get()
        if data[1]["pika_left"]["end_pose"] is not None and data[1]["pika_right"]["end_pose"] is not None and\
            data[0]["left_arm"]["qpos"] is not None and data[0]["left_arm"]["qpos"] is not None:
            break
        else:
            time.sleep(0.1)
    
    print("start teleop")

    time.sleep(3)
    
    # 将第一帧的坐标作为基坐标
    left_base_pose = data[0]["left_arm"]["qpos"]
    right_base_pose = data[0]["right_arm"]["qpos"]
    
    # 开始遥操
    while True:
        try:
            data = robot.get()

            # 这里获取到的坐标是四元数, 转为欧拉角
            left_delta_pose = matrix_to_xyz_rpy(data[1]["pika_left"]["end_pose"])
            right_delta_pose = matrix_to_xyz_rpy(data[1]["pika_right"]["end_pose"])

            # 计算基于基坐标的位置变化
            left_wrist_mat = apply_local_delta_pose(left_base_pose, left_delta_pose)
            right_wrist_mat = apply_local_delta_pose(right_base_pose, right_delta_pose)

            # 由于计算变化返回的是四元数, 这里再转回欧拉角
            l_data = matrix_to_xyz_rpy(left_wrist_mat)
            r_data = matrix_to_xyz_rpy(right_wrist_mat)

            print("left:", l_data.tolist())
            print("right:", r_data.tolist())

            # 设置移动的信息 (如果设备有提供专门遥操接口,则推荐实现此接口, 否则直接用qos接口)
            move_data = {
                "arm":{
                    "left_arm": {
                        "teleop_qpos":l_data},
                    "right_arm": {
                        "teleop_qpos":r_data},
                }
            }
            
            robot.move(move_data)
            time.sleep(0.02)
        except:
            print("data is none")
            time.sleep(0.1)
            

```