# 🖐️ 手部控制器基类

## 介绍  
`DexHandController` 是 `Controller` 的子类，用于控制机器人手部（如灵巧手、夹爪）。  
该类实现了手部状态获取和运动控制的方法，支持关节控制和动作控制的增量与绝对指令。

---

## 类定义
```python
class HandController:
```

继承自 `Controller` 的机器人手部控制器，实现了 `get_information()` 和 `move()` 方法。

---

## 成员属性

| 属性名             | 类型    | 说明                             |
|--------------------|---------|----------------------------------|
| `controller_type`  | `str`   | 控制器类型，设为 `"robotic_hand"` |
| `is_set_up`        | `bool`  | 控制器是否初始化，默认为 `False` |
| `controller`       | `Any`   | 外部控制器句柄，默认为 `None`    |

---

## 函数

### **__init__(self)**
初始化手部控制器。

- 参数：无  
- 返回：无  

---

### **get_information(self) -> Dict[str, Any]**
获取当前手部状态信息。返回字段由 `self.collect_info` 决定。  
支持字段包括：

- `"joint"`：关节角度
- `"action"`：执行的动作
- `"velocity"`：速度信息
- `"force"`：力传感信息

- 参数：无  
- 返回：  
  `Dict[str, Any]`：包含当前状态的字典，键为字段名，值为对应状态值。

---

### **move(self, move_data: Dict[str, Any], is_delta: bool = False)**
发送控制指令控制手部关节或动作。

- 增量控制 (`is_delta=True`)：  
  `"joint"` 和 `"action"` 为在当前状态基础上的偏移量。

- 绝对控制 (`is_delta=False`)：  
  所有字段均表示目标状态。

支持字段包括：

- `"joint"`：关节角度
- `"action"`：控制动作（策略/高维向量）

- 参数：
  - `move_data` (`Dict[str, Any]`)：控制数据，键为字段名，值为对应的目标值（推荐使用 `numpy.array`）。
  - `is_delta` (`bool`, 可选)：是否为增量控制，默认为 `False`。

- 返回：无

---

### **__repr__(self) -> str**
打印控制器当前状态。

- 参数：无  
- 返回：  
  `str`：控制器状态描述字符串，包含内部 `controller` 的引用信息。

---

## 实现自己的手部控制器子类

参考本基类创建自己的具体实现，需根据硬件/仿真 API 补充底层函数。

---

### 必要实现

#### **get_state()**
用于返回当前手部所有状态信息，用于支持增量控制逻辑。

- 参数：无  
- 返回：  
  `Dict[str, Any]`：包含所有状态信息（例如 joint, action）

---

### 可选择实现

#### **get_joint()**
返回当前关节角度。

- 参数：无  
- 返回：  
  `np.array`：关节角度数组。

#### **get_action()**
返回当前手部动作状态（策略输出）。

- 参数：无  
- 返回：  
  `np.array`：动作向量。

#### **get_velocity()**
返回当前速度信息。

- 参数：无  
- 返回：  
  `np.array`：速度信息。

#### **get_force()**
返回当前施加的力信息。

- 参数：无  
- 返回：  
  `np.array`：力信息。

#### **set_joint(move_data: np.array)**
设置关节角度。

#### **set_action(move_data: np.array)**
设置手部的策略动作向量。
