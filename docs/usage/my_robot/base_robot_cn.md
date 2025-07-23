# 机器人系统封装基类

## 介绍
`Robot` 是用于封装多控制器与多传感器的机器人类，负责统一调度控制器与传感器的运行，并管理数据的采集、回放与控制接口。该类提供统一的调用接口，支持状态信息获取、数据记录、运动控制与回放等功能。

## 类定义
```python
class Robot:
```

## 成员属性

| 属性名        | 类型                       | 说明                                         |
|-------------|--------------------------|--------------------------------------------|
| name        | str                      | 机器人名称，默认为 "base_robot"             |
| controllers | Dict[str, Dict[str, Any]]| 控制器字典，支持多种类型与多个控制器实例     |
| sensors     | Dict[str, Dict[str, Any]]| 传感器字典，支持多种类型与多个传感器实例     |
| condition   | Dict[str, Any]           | 系统设定参数，如保存路径、频率等             |
| collection  | CollectAny               | 数据采集器，用于缓存并保存数据               |

## 函数

### **__init__(self, start_episode=0)**
初始化 `Robot` 实例。

- 参数：
  - `start_episode` (int): 起始 episode 编号，默认为 0
- 返回：无

---

### **set_up(self)**
机器人初始化接口。该方法需在子类中重写。

- 参数：无
- 返回：无（默认抛出 `NotImplementedError`）

---

### **set_collect_type(self, INFO_NAMES: Dict[str, Any])**
为控制器与传感器设置需要采集的信息字段。

- 参数：
  - `INFO_NAMES` (Dict[str, Any])：每个控制器或传感器对应的采集字段名

- 返回：无

---

### **get(self) -> List[Dict[str, Any]]**
从控制器与传感器中获取当前状态信息。

- 参数：无
- 返回：
  - `List[Dict[str, Any]]`：包含 `[controller_data, sensor_data]` 的列表

---

### **collect(self, data: List[Dict[str, Any]])**
采集数据并存入缓存。

- 参数：
  - `data` (List[Dict[str, Any]])：`[controller_data, sensor_data]` 数据对

- 返回：无

---

### **finish(self)**
将缓存中的数据写入文件。

- 参数：无
- 返回：无

---

### **move(self, move_data: Dict[str, Any], key_banned: Optional[List[str]] = None)**
控制所有控制器运动。可指定跳过部分 key。

- 参数：
  - `move_data` (Dict[str, Any])：分层控制指令（控制器类型 -> 控制器名 -> 控制内容）
  - `key_banned` (List[str], 可选)：要从 move_data 中移除的字段名

- 返回：无

---

### **is_start(self) -> bool**
判断是否应开始动作。默认实现永远返回 True，可在子类中重写。

- 参数：无
- 返回：
  - `bool`: 默认返回 True

---

### **reset(self) -> bool**
复位机器人状态。默认实现永远返回 True，可在子类中重写。

- 参数：无
- 返回：
  - `bool`: 默认返回 True

---

### **replay(self, data_path: str, key_banned: Optional[List[str]] = None)**
读取并回放一段动作数据。

- 参数：
  - `data_path` (str): `.hdf5` 数据路径
  - `key_banned` (List[str], 可选): 控制时忽略的字段名

- 返回：无

---

### **play_once(self, episode: Dict[str, Any], key_banned: Optional[List[str]] = None)**
回放单帧 episode 中的动作。

- 参数：
  - `episode` (Dict[str, Any])：单帧数据
  - `key_banned` (List[str], 可选): 控制时忽略字段

- 返回：无

---

## 工具函数

### **get_array_length(data: Dict[str, Any]) -> int**
获取最外层 `np.ndarray` 的长度。

- 参数：
  - `data` (Dict[str, Any])：嵌套字典

- 返回：
  - `int`: 数据长度

---

### **split_nested_dict(data: Dict[str, Any], idx: int) -> Dict[str, Any]**
提取嵌套数据结构中的第 idx 帧。

- 参数：
  - `data` (Dict[str, Any])：嵌套字典
  - `idx` (int)：索引帧

- 返回：
  - `Dict[str, Any]`: 第 `idx` 帧数据

---

### **dict_to_list(data: Dict[str, Any]) -> List[Dict[str, Any]]**
将嵌套字典结构转换为按帧排列的列表。

- 参数：
  - `data` (Dict[str, Any])：嵌套字典

- 返回：
  - `List[Dict[str, Any]]`: 按帧拆分的列表

---

### **remove_duplicate_keys(source_dict: dict[str, Any], keys_to_remove: list[str]) -> dict[str, Any]**
从字典中删除指定键。

- 参数：
  - `source_dict` (dict): 原始字典
  - `keys_to_remove` (list): 要删除的 key

- 返回：
  - `dict`: 删除后的新字典
