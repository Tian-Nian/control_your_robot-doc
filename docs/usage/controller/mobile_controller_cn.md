# 🚗 移动平台控制器基类

## 介绍  
`MobileController` 是 `Controller` 的子类，用于控制移动机器人（如有轮底盘）。  
该类封装了获取移动状态信息与发送控制指令的方法，包括设置移动速度、目标位置等。

---

## 类定义
```python
class MobileController:
```

继承自 `Controller` 的移动平台控制器，实现了 `get_information()` 和 `move()` 方法。

---

## 成员属性

| 属性名             | 类型    | 说明                               |
|--------------------|---------|------------------------------------|
| `controller_type`  | `str`   | 控制器类型，设为 `"robotic_mobile"` |
| `controller`       | `Any`   | 外部控制器句柄，默认为 `None`      |

---

## 函数

### **__init__(self)**
初始化移动控制器。

- 参数：无  
- 返回：无  

---

### **move(self, move_data: Dict[str, Any])**
发送移动控制指令。此类不区分是否为增量控制。

支持字段包括：

- `"move_velocity"`：底盘移动速度
- `"move_to"`：目标位置（如地图坐标）

- 参数：
  - `move_data` (`Dict[str, Any]`)：控制数据，键为字段名，值为目标控制值。

- 返回：无

---

### **get_information(self) -> Dict[str, Any]**
获取移动机器人的当前状态。返回字段由 `self.collect_info` 决定。  
支持字段包括：

- `"rotate"`：当前朝向或角度信息
- `"move_velocity"`：当前移动速度
- `"move_to"`：当前目标位置或期望位置

- 参数：无  
- 返回：  
  `Dict[str, Any]`：包含当前状态的字典，键为字段名，值为对应状态值。

---

### **__repr__(self) -> str**
打印控制器当前状态。

- 参数：无  
- 返回：  
  `str`：控制器状态描述字符串，包含内部 `controller` 的引用信息。

---

## 实现自己的移动控制器子类

根据仿真平台或真实硬件控制接口补全下列函数。

---

### 必要实现

#### **get_state()**
用于返回当前完整状态信息（如 rotate, move_velocity 等），供其他逻辑使用。

- 参数：无  
- 返回：  
  `Dict[str, Any]`：状态信息字典

---

### 可选择实现

#### **get_rotate()**
获取当前移动平台朝向或角度信息。

- 参数：无  
- 返回：  
  `np.array`：角度/朝向向量

#### **get_move_velocity()**
获取当前底盘移动速度。

- 参数：无  
- 返回：  
  `np.array`：速度值

#### **get_move_to()**
获取当前目标移动位置。

- 参数：无  
- 返回：  
  `np.array`：目标位置

#### **set_move_velocity(move_data: np.array)**
设置底盘移动速度。

#### **set_move_to(move_data: np.array)**
设置底盘移动目标位置。
