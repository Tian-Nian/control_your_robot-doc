# 🦾 机械臂控制器类

## 介绍  
`ArmController` 继承自 `Controller`，用于控制机械臂设备。  
实现了机械臂状态信息采集与控制指令发送，支持关节角、末端位置、夹爪、动作、速度及力的控制。

---

## 类定义
```python
class ArmController(Controller):
```

机械臂控制器，实现了核心方法 `get_information()` 和 `move_controller()`。

---

## 成员属性

| 属性名           | 类型         | 说明                         |
|-----------------|--------------|------------------------------|
| `name`          | `str`        | 控制器名称，默认 `"arm_controller"` |
| `controller`    | `Any`        | 机械臂控制器接口或硬件句柄，默认 `None` |
| `controller_type`| `str`       | 控制器类型，固定为 `"robotic_arm"`   |

---

## 函数

### **__init__(self)**  
初始化机械臂控制器实例。

- 参数：无  
- 返回：无  

---

### **get_information(self) -> Dict[str, Any]**  
根据 `self.collect_info` 获取机械臂当前状态信息。

支持字段：  
- `"joint"`：关节角度数组  
- `"qpos"`：末端位置或完整位姿  
- `"gripper"`：夹爪状态  
- `"action"`：当前执行动作  
- `"velocity"`：速度信息  
- `"force"`：力传感数据  

- 参数：无  
- 返回：  
  - `Dict[str, Any]`：对应字段的状态字典  

---

### **move_controller(self, move_data: Dict[str, Any], is_delta=False)**  
执行机械臂运动控制指令。  

- 当 `is_delta=True` 时，针对 `"joint"` 和 `"qpos"` 字段进行增量控制（基于当前状态累加）；  
- 否则所有字段均采用绝对控制。  
- 对 `"action"`、`"gripper"`、`"velocity"`、`"force"` 均使用绝对值控制。  

- 参数：  
  - `move_data` (`Dict[str, Any]`)：控制字段及对应目标值（`numpy.array` 格式）  
  - `is_delta` (`bool`)：是否增量控制，默认为 `False`  
- 返回：无  

---

### **__repr__(self) -> str**  
机械臂控制器的字符串表示。

- 参数：无  
- 返回：  
  - `str`：包含名称和内部控制器信息的描述字符串  

---

## 扩展说明

- 子类应实现以下方法以完成控制接口：  
  - `get_state()`：获取当前机械臂完整状态字典  
  - `set_joint(np.array)`：设置机械臂关节角  
  - `set_position(np.array)`：设置机械臂末端位姿  
  - `set_action(np.array)`：设置动作控制  
  - `set_gripper(np.array)`：设置夹爪开合  
  - `set_velocity(np.array)`：设置速度  
  - `set_force(np.array)`：设置力控制  

这些方法根据具体机械臂硬件或仿真环境实现。  
