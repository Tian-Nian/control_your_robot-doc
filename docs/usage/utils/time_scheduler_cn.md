# 时间同步器模块

## 介绍
`TimeScheduler` 是用于多进程同步控制的时间协调器。通过调度多个 `Semaphore` 信号量，它能够以设定频率同步各子进程的执行周期，适用于数据采集、传感器同步等需要统一时序的系统场景。

此外，配合 `worker` 函数可进行多进程数据写入模拟测试，用于验证同步调度效果。

## 类定义
```python
class TimeScheduler:
```

## 成员属性

| 属性名                         | 类型              | 说明                                                                 |
|------------------------------|------------------|----------------------------------------------------------------------|
| time_freq                    | int              | 时间同步频率（单位 Hz），用于控制每秒触发周期                          |
| time_semaphores              | List[Semaphore]  | 子进程使用的信号量列表，每个子进程对应一个信号量                         |
| process_name                 | str              | 调度器名称，默认为 `"time_scheduler"`                                |
| real_time_accumulate_time_interval | multiprocessing.Value | 实际累计时间间隔，用于评估同步效率                                 |
| step                         | multiprocessing.Value | 同步轮次计数器                                                     |
| time_locker                  | multiprocessing.Process | 后台同步调度进程                                                 |

## 函数

### **__init__(self, time_semaphores: List[Semaphore], time_freq=10)**
初始化时间同步器。

- 参数：
  - `time_semaphores` (List[Semaphore])：控制子进程同步的信号量
  - `time_freq` (int)：同步频率，单位为 Hz，默认值为 10

- 返回：无

---

### **time_worker(self)**
后台同步线程的执行逻辑。按照设定频率释放所有子进程信号量，并记录实际触发间隔。

- 参数：无
- 返回：无（内部循环执行）

---

### **start(self)**
启动时间同步器（作为独立进程运行）。

- 参数：无
- 返回：无

---

### **stop(self)**
关闭并释放时间同步器进程，计算并打印实际采集频率。

- 参数：无
- 返回：无

---

## 子进程工作函数

### **worker(process_id: int, process_name: str, time_semaphore: Semaphore, result_array: Array, result_lock: Lock)**
用于测试的子进程函数。等待调度器释放信号量后执行任务，并写入共享结果数组。

- 参数：
  - `process_id` (int)：进程序号
  - `process_name` (str)：进程名称
  - `time_semaphore` (Semaphore)：进程控制信号量
  - `result_array` (multiprocessing.Array)：共享结果数组
  - `result_lock` (Lock)：写入锁

- 返回：无（无限循环）

---

## 示例入口
```python
if __name__ == "__main__":
    processes = []
    process_num = 4
    result_array = Array('d', process_num * 25)
    time_semaphores = [Semaphore(0) for _ in range(process_num)]
    result_lock = Lock()

    # 启动子进程
    for i in range(process_num):
        process = Process(target=worker, args=(i, f"process_{i}", time_semaphores[i], result_array, result_lock))
        process.start()
        processes.append(process)

    # 启动时间调度器
    time_scheduler = TimeScheduler(time_semaphores, time_freq=10)
    time_scheduler.start()

    try:
        time.sleep(5)
        print("Sample result snapshot:", result_array[:])
    except KeyboardInterrupt:
        print("Main process interrupted.")
    finally:
        time_scheduler.stop()
        for p in processes:
            p.terminate()
            p.join()
```

---

## 使用建议
如需与机器人系统集成，建议在控制器、传感器等模块的 `get()` 调用中配合信号量实现“按时间槽执行”的统一时序控制，确保同步采集与控制。

