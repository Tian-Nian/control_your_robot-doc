# 类定义文档：ROSSubscriber

---

## 类介绍

```python
class ROSSubscriber:
```

一个简洁的 ROS 订阅器类，封装了 ROS 话题订阅、回调处理及线程安全的消息缓存读取。适用于定期轮询最新消息，或响应式触发自定义回调函数。

---

## 类继承

无（基础类）

---

## 类定义代码块

```python
class ROSSubscriber:
    def __init__(self, topic_name, msg_type, call: Optional[Callable] = None):
        ...
```

---

## 成员属性说明

| 属性名        | 类型              | 说明                                             |
|-------------|-----------------|--------------------------------------------------|
| `topic_name` | `str`            | 订阅的话题名称                                       |
| `msg_type`   | `Type`           | 消息类型（如 `PoseStamped`, `Twist`, 自定义消息等）   |
| `latest_msg` | `Any`            | 最近接收到的消息（线程安全保存）                         |
| `lock`       | `threading.Lock` | 用于保护 `latest_msg` 的线程锁                         |
| `user_call`  | `Callable` or `None` | 可选的用户自定义回调函数，接收 `msg` 作为参数             |
| `subscriber` | `rospy.Subscriber` | ROS 的订阅器对象，用于接收消息                          |

---

## 函数接口

### `__init__`

```python
def __init__(self, topic_name, msg_type, call: Optional[Callable] = None):
```

**功能描述**：构造 ROS 订阅器并启动订阅。

**参数**：

| 参数名        | 类型                  | 说明                                       |
|-------------|---------------------|------------------------------------------|
| `topic_name` | `str`                | 订阅的 ROS 话题名                              |
| `msg_type`   | `Type`               | 消息类型（例如 `PoseStamped`, `Twist`）         |
| `call`       | `Callable` or `None` | （可选）接收到消息时执行的回调函数，参数为该消息对象 |

**返回值**：无

---

### `callback`

```python
def callback(self, msg):
```

**功能描述**：内部回调函数，接收消息并更新缓存；如设置用户回调函数则一并调用。

**参数**：

| 参数名 | 类型 | 说明           |
|------|------|--------------|
| `msg` | `Any` | 接收到的 ROS 消息 |

**返回值**：无

---

### `get_latest_data`

```python
def get_latest_data(self):
```

**功能描述**：返回最近收到的消息。

**参数**：无

**返回值**：

| 返回值类型 | 说明                      |
|---------|-------------------------|
| `Any`   | 最后接收的 ROS 消息，或 None（若尚未接收到） |

---

## 示例主程序

```python
if __name__ == "__main__":
    import time
    from geometry_msgs.msg import PoseStamped

    # 初始化 ROS 节点
    rospy.init_node('ros_subscriber_node', anonymous=True)

    # 示例订阅 PoseStamped 类型话题
    ros_test = ROSSubscriber('/pika_pose_l', PoseStamped)

    for i in range(100):
        print(ros_test.get_latest_data())
        time.sleep(0.1)
```

---

## 实现自己的子类（可选）

此类可直接使用，无需继承。如需拓展功能，可重写 `callback()` 方法，例如：

```python
class MySubscriber(ROSSubscriber):
    def callback(self, msg):
        super().callback(msg)  # 保留原逻辑
        print("Received message with custom logic")
```

---

## 线程安全说明

- 所有对 `latest_msg` 的访问都通过 `threading.Lock` 保护，确保在多线程中不会读取到不一致的数据。
- 可在主线程中轮询 `get_latest_data()`，同时回调函数在 ROS 回调线程中运行。

