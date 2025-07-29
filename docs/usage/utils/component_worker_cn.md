# 模块：多进程同步与数据共享

---

## 类定义：DataBuffer

用于在多进程环境中共享不同组件采集的数据。

```python
class DataBuffer:
    '''
    一个用于共享存储不同组件采集的数据的信息的类
    输入:
    manager: 创建的一个独立的控制器, multiprocessing::Manager
    '''
    def __init__(self, manager):
        self.manager = manager
        self.buffer = manager.dict()

    def collect(self, name, data):
        if name not in self.buffer:
            self.buffer[name] = self.manager.list()
        self.buffer[name].append(data)

    def get(self):
        return dict(self.buffer)
```

### 成员属性

| 属性名       | 类型                    | 说明                     |
|------------|-----------------------|------------------------|
| `manager`  | `multiprocessing.Manager` | 控制共享数据结构的创建与同步 |
| `buffer`   | `Dict[str, List[Any]]`    | 存储按组件名分类的数据列表   |

### 函数一览

#### `__init__(self, manager)`
- **功能**：初始化共享数据容器。
- **参数**：
  - `manager` (`multiprocessing.Manager`): 多进程共享管理器。

#### `collect(self, name, data)`
- **功能**：收集一条数据并按组件名分类存入缓冲区。
- **参数**：
  - `name` (`str`): 组件名称。
  - `data` (`Any`): 要存储的数据。

#### `get(self)`
- **功能**：获取所有已存储的数据副本（转为常规字典）。
- **返回值**：`Dict[str, List[Any]]` 数据缓冲字典。

---

## 函数定义：ComponentWorker

组件级别的多进程同步器，用于执行独立子进程的组件采集逻辑。

```python
def ComponentWorker(component_class, component_name, component_setup_input,
                    component_collect_info, data_buffer: DataBuffer,
                    time_lock: Semaphore, start_event: Event,
                    finish_event: Event, process_name: str):
    '''
    组件级别的多进程同步器, 用于多进程数据采集, 如果希望是多进程的同步控制也可以稍微改下代码添加一个共享的信号输入
    输入:
    component_class: 你的组件类, 可以自定义的是controller / sensor, class
    component_name: 你希望组件的名称, 用于对应组件info的输出, str
    component_setup_input: 组件初始化需要设置的信息, List[Any]
    component_collect_info: 组件采集的数据种类, List[str]
    data_buffer: 初始化一个同步所有组件的内存空间, DataBuffer
    time_lock: 初始化对于当前组件的时间同步锁, 该锁需要分配给time_scheduler用于控制时间, multiprocessing::Semaphore
    start_event: 同步开始事件, 所有的组件共用一个, multiprocessing::Event
    finish_event: 同步结束事件, 所有的组件共用一个, multiprocessing::Event
    process_name:你希望当前进程叫什么, 用于对应子进程info的输出, str
    '''
```

### 参数说明

| 参数名                  | 类型                       | 说明                                                         |
|----------------------|--------------------------|------------------------------------------------------------|
| `component_class`     | `Type`                   | 自定义组件类（如 `TestArmController`, `TestVisionSensor`）               |
| `component_name`      | `str`                    | 组件名称（用于调试和分类数据）                                          |
| `component_setup_input` | `Optional[List[Any]]`    | 组件初始化参数，如为 `None` 则使用默认构造函数                                |
| `component_collect_info` | `List[str]`              | 该组件要采集的数据类型（如 `["color"]`, `["joint", "qpos"]`）           |
| `data_buffer`         | `DataBuffer`             | 数据共享对象，跨进程存储采集结果                                        |
| `time_lock`           | `Semaphore`              | 控制时间步的锁，与时间调度器配合控制采集频率                                 |
| `start_event`         | `Event`                  | 由主线程触发的全局“开始”事件                                          |
| `finish_event`        | `Event`                  | 全局“结束”事件，触发后各子进程结束任务                                       |
| `process_name`        | `str`                    | 当前进程的标识名（用于 debug 日志输出）                                 |

---

## 示例：主函数使用方式（简略）

```python
manager = Manager()
data_buffer = DataBuffer(manager)
time_lock = Semaphore(0)
start_event = Event()
finish_event = Event()

vision_proc = Process(target=ComponentWorker, args=(
    TestVisionSensor, "vision", None, ["color"],
    data_buffer, time_lock, start_event, finish_event, "vision_proc"
))
vision_proc.start()
```

---

## 实现自己的组件类（Controller 或 Sensor）

组件类需要实现以下接口：

### 必要函数

#### `set_up(*args)`
- 初始化组件，接收任意参数。
- 无返回值。

#### `set_collect_info(info: List[str])`
- 告知组件采集哪些类型的数据。

#### `get() -> Dict[str, Any]`
- 获取当前采集结果，作为一个字典返回。

---

如需整理更多类或模块，随时贴上来我继续按此格式编写。
