# 模块：机器人数据采集进程

---

## 函数定义：RobotWorker

该函数用于在独立进程中运行机器人类的采集任务，配合多进程事件和信号完成启动、执行和终止控制。

```python
def RobotWorker(robot_class, start_episode,
                time_lock: Semaphore, start_event: Event, finish_event: Event, process_name: str):
    '''
    对于实现的机器人类进行多进程数据采集, 可以对多个机器人进行.
    
    输入:
    robot_class: 机器人类, my_robot::robot_class
    start_episode: 数据采集的开始序号, 只影响保存数据的后缀组号, int
    time_lock: 初始化对于当前组件的时间同步锁, 该锁需要分配给time_scheduler用于控制时间, multiprocessing::Semaphore
    start_event: 同步开始事件, 所有的组件共用一个, multiprocessing::Event
    finish_event: 同步结束事件, 所有的组件共用一个, multiprocessing::Event
    process_name: 你希望当前进程叫什么, 用于对应子进程info的输出, str
    '''
```

### 参数说明

| 参数名         | 类型              | 说明                                                         |
|--------------|------------------|------------------------------------------------------------|
| `robot_class`   | `Type`           | 自定义的机器人类（需要实现 `set_up`, `get`, `collect`, `finish`）接口 |
| `start_episode` | `int`            | 本轮数据采集开始的序号编号，用于设置保存数据的起始编号                    |
| `time_lock`     | `Semaphore`      | 用于步进采集的时间控制信号，每次采集需调用 `acquire()`                 |
| `start_event`   | `Event`          | 全局同步开始事件，主进程触发后所有进程开始采集                          |
| `finish_event`  | `Event`          | 全局同步结束事件，触发后当前进程终止采集                               |
| `process_name`  | `str`            | 当前进程标识名，用于日志调试输出                                      |

---

## 工作流程

1. 初始化机器人类实例并调用 `set_up()`。
2. 等待 `start_event` 触发前，每 5 秒提示一次“等待开始”。
3. 一旦开始信号触发，开始循环采集：
   - 每次等待 `time_lock.acquire()`。
   - 检查 `finish_event` 是否已经设置，若是则终止。
   - 获取数据并调用机器人类的 `collect()` 方法处理。
4. 循环持续至 `finish_event` 被设置。
5. 调用机器人类的 `finish()` 完成最终写入。

---

## 机器人类要求实现接口

### 必要函数

#### `set_up()`
- 初始化机器人内部状态。
- 无返回值。

#### `get() -> Dict[str, Any]`
- 获取当前一帧的机器人状态（如传感器、位置等）。
- 返回值为字典，存储需要采集的信息。

#### `collect(data: Dict[str, Any])`
- 将 `get()` 采集的数据存储/处理。

#### `finish()`
- 所有采集完成后，执行数据保存或清理动作。

---

## 示例用法（与多进程集成）

```python
from multiprocessing import Process, Semaphore, Event, Manager

start_event = Event()
finish_event = Event()
time_lock = Semaphore(0)

robot_proc = Process(target=RobotWorker, args=(
    MyRobotClass, 0, time_lock, start_event, finish_event, "robot_proc"
))
robot_proc.start()
```

---

如需文档化更多进程函数或组件，欢迎继续贴代码。
