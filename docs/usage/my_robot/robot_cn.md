# 自定义机器人类示例文档

## 介绍  
`TestRobot` 是一个用户自定义的机器人控制器示例类，展示了如何集成多个机械臂控制器和视觉传感器，并实现数据采集、控制和管理的完整流程。  
该示例结合了机械臂控制、视觉传感器数据获取以及统一的数据采集接口，适合作为自定义机器人类开发的模板。

---

## 类定义
```python
class TestRobot:
```

集成机械臂控制器与视觉传感器的机器人控制类，支持控制初始化、数据采集及运动指令下发。

---

## 成员属性

| 属性名           | 类型              | 说明                                         |
|-----------------|-------------------|----------------------------------------------|
| `INFO`          | `str`             | 日志输出级别，如 `"DEBUG"`                     |
| `DoFs`          | `int`             | 机械臂自由度数量，默认为 6                     |
| `arm_controllers`| `Dict[str, Controller]` | 包含多个机械臂控制器实例，键为机械臂名称               |
| `image_sensors` | `Dict[str, Sensor]`     | 包含多个视觉传感器实例，键为传感器名称                   |
| `condition`     | `dict`            | 采集配置参数字典（如存储路径、任务名、采集频率等）        |
| `collection`    | `CollectAny`      | 数据采集管理实例，用于统一采集和存储数据                    |

---

## 函数

### **__init__(self, DoFs=6, INFO="DEBUG", start_episode=0)**  
机器人初始化，创建机械臂控制器和视觉传感器实例，初始化数据采集模块。

- 参数：  
  - `DoFs` (`int`, 可选)：机械臂自由度，默认 6, 对于自己的机器人无需定义该参数 
  - `INFO` (`str`, 可选)：日志输出等级，默认 `"DEBUG"`, 对于自己的机器人无需定义该参数   
  - `start_episode` (`int`, 可选)：数据采集起始episode编号，默认 0  
- 返回：无  

---

### **reset(self)**  
将所有机械臂控制器重置到初始关节状态（均为零向量）。

- 参数：无  
- 返回：无  

---

### **set_up(self)**  
初始化所有机械臂控制器和视觉传感器，设置采集字段。

- 参数：无  
- 返回：无  

---

### **set_collect_type(self, ARM_INFO_NAME, IMG_INFO_NAME)**  
设置机械臂控制器和视觉传感器需要采集的数据字段。

- 参数：  
  - `ARM_INFO_NAME` (`List[str]`)：机械臂采集字段列表，如 `["joint", "qpos", "gripper"]`  
  - `IMG_INFO_NAME` (`List[str]`)：视觉传感器采集字段列表，如 `["color"]`  
- 返回：无  

---

### **is_start(self) -> bool**  
判断机器人是否准备启动，示例中总是返回 `True`。

- 参数：无  
- 返回：  
  - `bool`：是否启动标志  

---

### **get(self) -> List[Dict]**  
获取当前所有机械臂控制器和视觉传感器的采集数据。

- 参数：无  
- 返回：  
  - `List[Dict]`：列表包含两个字典，第一个为所有机械臂采集数据，第二个为所有视觉传感器采集数据。  

---

### **collect(self, data)**  
调用数据采集模块收集传入数据。

- 参数：  
  - `data` (`List[Dict]`)：由 `get()` 返回的采集数据  
- 返回：无  

---

### **finish(self)**  
完成数据采集，触发数据写入存储。

- 参数：无  
- 返回：无  

---

### **set_action(self, action)**  
设置所有机械臂控制器的动作指令。

- 参数：  
  - `action`：动作指令数据  
- 返回：无  

---

### **move(self, move_data)**  
发送运动控制指令给所有机械臂控制器。

- 参数：  
  - `move_data` (`Dict[str, Dict]`)：键为机械臂名称，值为对应的控制指令字典  
- 返回：无  

---

## 使用示例
```python
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "DEBUG"  # 设置日志级别

    robot = TestRobot()
    robot.set_up()

    # 获取当前状态数据
    data = robot.get()

    # 构造运动控制指令示例（6自由度关节角）
    move_data = {
        "left_arm": {
            "joint": np.random.rand(6) * 3.1415926
        },
        "right_arm": {
            "joint": np.random.rand(6) * 3.1415926
        }
    }

    # 执行运动控制
    robot.move(move_data)
```

---

## 备注  
- `TestArmController`、`TestVisonSensor`、`CollectAny` 等均为用户自定义的模块和类，需根据具体机器人硬件和需求自行实现。  
- 本示例框架支持多机械臂、多视觉传感器协同工作，适用于复杂机器人系统集成开发。  
- 数据采集与存储功能封装于 `CollectAny`，便于统一管理训练与测试数据。  
