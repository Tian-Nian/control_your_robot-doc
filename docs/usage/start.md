# 使用总览

本项目主要目的是打造一个通用的具身智能数据传输模式, 涵盖机器人自定义, 数据采集, 模型部署, 机器人遥操四个方向.

---

## 如何设置自己的机器人
本项目将机器人的部件分为两类:  
`controller`: 拥有控制功能的部件, 如机械臂, 灵巧手, 底盘等...实现自己的controller请参考[controller](./controller/controller_cn.md)与对应类型的controller实现说明.
`sensor`: 只用于获取信息的部件, 如视觉传感器, 触觉传感器等...实现自己的sensor请参考[sensor](./sensor/sensor_cn.md)与对应类型的sensor实现说明.   
如果你的操作部件和传感器已经在controller/sensor中了, 那你可以直接调用, 否则的话你可以提issue, 我们会尽可能收集到该款机械臂, 并进行适配工作. 如果想自己实现的话, 可以参考developer_README.md, 欢迎各位完成适配后提交PR!

注意!   
 我们希望机械臂返回的的joint angle是弧度制, 即[-pi, pi], 夹爪是归一化的张合度[0,1], 末端6D坐标中x,y,z单位为米, rx,ry,rz单位为弧度制, 当然你也可以用四元数来控制机械臂, 这取决于你机械臂api的支持, 操控的对应数据单位相同与获取数据的单位.

在保证你所需要的部件都已经被定义后, 请模仿my_robot中的`test_robot`示例, 可以参考[自定义机器人示例](./my_robot/robot_cn.md)来组装你的机器人.
在完成组装后, 你可以编写示例, 来查看几个关键函数是否被正确实现:
``` python
if __name__=="__main__":
    import time
    robot = PiperSingle()
    # 采集测试
    data_list = []
    for i in range(100):
        print(i)
        data = robot.get()
        robot.collect(data)
        time.sleep(0.1)
    robot.finish()
    # 运动测试
    move_data = {
        "left_arm":{
        "qpos":[0.057, 0.0, 0.216, 0.0, 0.085, 0.0],
        "gripper":0.2,
        },
    }
    robot.move(move_data)
```

---

## 采集数据
采集数据需要你至少定义好需要采集的组件对应的controller与sensor, 如我的机器人是一个双臂带摄像头的机器人, 只有上半身, 那我至少要定义我机械臂对应的controller与摄像头对应的sensor.


然后在`example/collect/`中选择你希望使用的模式, 具体请参考[数据采集流程](./example/collect_cn.md).

---

## 部署模型
对于部署模型, 你需要了解该模型推理需要的参数与模型输出的参数, 并实现对应接口, 具体请参考[部署模型流程](./example/deploy_cn.md)来部署自己的模型.

---

## 机械臂遥操

遥操由于控制频率较高, 且对于IK逆解约束较强, 因此大部分机械臂提供的直接IK接口在遥操上表现比较差, 最好使用Canfd类型接口, 用于专门遥操IK. 

为了区分二者的末端控制, 专门在`arm_controller`中提供了`teleop_qpos`的关键词, 用于调用遥操接口. 

具体实现可以参考`my_robot/*_teleop.py`的实现, 实现说明请看[机器人遥操说明](./my_robot/teleop_robot_cn.md)