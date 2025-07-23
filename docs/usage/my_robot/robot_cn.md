# 自定义机器人

继承基类[base_robot](./base_robot_cn.md)后, 你只需要实现简单的几个函数既可以完成你的机器人定义.

## 必要实现: `__init__()`与`set_up()`函数
这两个函数用于初始化所有机器人的元件, 包括controller与sensor.
你需要将所有controller按照如下形式初始化:

**注意!!!**

对于所有的controller与sensor, 其名称不能相同, 否则无法正确使用`replay()`函数, 就算是不同controller_type的也不行.

```python
def __init(self, ...):
    ...
    self.controllers = {
        "controller_type_1":{
            "controller_11": controller_type_1(...),
            "controller_12": controller_type_1(...),
            ...
        },
        "controller_type_2":{
            "controller_21": controller_type_2(...),
            "controller_22": controller_type_2(...),
            ...
        },
        ...
    }

    self.sensors = {
        "cosensorntrollers_type_1":{
            "sensor_11": sensor_type_1(...),
            "sensor_12": sensor_type_1(...),
            ...
        },
        "sensor_type_2":{
            "sensor_21": sensor_type_2(...),
            "sensor_22": controller_type_2(...),
            ...
        },
        ...
    }

    def set_up(self, ...):
        self.controllers["controller_type_1"]["controller_11"].set_up()
        self.controllers["controller_type_1"]["controller_12"].set_up()
        ...
        self.sensors["sensor_type_1"]["sensor_11"].set_up()
        self.sensors["sensor_type_1"]["sensor_11"].set_up()
```

## 可选实现: `is_start()`与`reset()`
对于这两个函数, 都是用于采集数据的, is_start用于自动判断是否机械臂开始运动了, 开始运动后才开始正式写入数据, 能比较好的保证数据没有异常.

reset()函数用于重置机械臂, 用于连续采集/部署快速归位.