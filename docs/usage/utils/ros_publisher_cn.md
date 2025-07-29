# 类定义文档：ROSPublisher

---

## 类介绍

```python
class ROSPublisher:
```

一个简易的 ROS 发布器类，支持单次或持续性地向指定话题发布消息。适用于控制指令（如 `Twist`）或其他周期性发布的数据。

---

## 类继承

无（基础类）

---

## 类定义代码块

```python
class ROSPublisher:
    def __init__(self, topic_name, msg_type, continuous=True):
        ...
```

---

## 成员属性说明

| 属性名           | 类型        | 说明                                       |
|----------------|-----------|------------------------------------------|
| `topic_name`     | `str`      | 发布话题名                                   |
| `msg_type`       | `Type`     | 消息类型（如 `Twist`, `PoseStamped` 等）        |
| `publisher`      | `Publisher`| ROS 的 `rospy.Publisher` 实例                 |
| `pub_msg`        | `Any`      | 当前待发布的消息实例                             |
| `shutdown_flag`  | `bool`     | 是否进入停止状态标志                              |
| `continuous`     | `bool`     | 是否持续发布消息（True 表示使用 `rospy.Timer` 定时） |

---

## 函数接口

### `__init__`

```python
def __init__(self, topic_name, msg_type, continuous=True):
```

**功能描述**：初始化 ROS 发布器。

**参数**：

| 参数名        | 类型    | 说明                           |
|-------------|-------|------------------------------|
| `topic_name` | `str`  | 发布话题名                        |
| `msg_type`   | `Type` | 消息类型（如 `Twist`）             |
| `continuous` | `bool` | 是否启用定时器周期发布（默认 `True`） |

**返回值**：无

---

### `publish`

```python
def publish(self, event=None):
```

**功能描述**：向话题发布当前 `pub_msg` 消息。

**参数**：

| 参数名   | 类型     | 说明                     |
|--------|--------|------------------------|
| `event` | `TimerEvent` | ROS Timer 自动传入，可为空 |

**返回值**：无

---

### `continuous_publish`

```python
def continuous_publish(self):
```

**功能描述**：通过 `rospy.Timer` 每 0.01 秒周期性发布消息。

**参数**：无

**返回值**：无

---

### `update_msg`

```python
def update_msg(self, msg):
```

**功能描述**：更新当前待发布的消息。

**参数**：

| 参数名 | 类型 | 说明         |
|------|----|------------|
| `msg` | `Any` | ROS 消息对象 |

**返回值**：无

---

### `stop`

```python
def stop(self):
```

**功能描述**：设置停止标志，终止后续发布。

**参数**：无

**返回值**：无

---

## 启动发布线程辅助函数

```python
def start_publishing(publisher):
    publisher.continuous_publish()
    rospy.on_shutdown(publisher.stop)
```

**说明**：为该发布器绑定 `rospy.Timer` 并在 ROS 关闭时执行 `stop()`。

---

## 示例主程序

```python
if __name__ == "__main__":
    try:
        publisher = ROSPublisher('/cmd_vel', Twist)
        rospy.init_node('ros_publisher_node', anonymous=True)

        msg = Twist()
        msg.linear.x = 0.1 
        publisher.update_msg(msg)

        pub_thread = threading.Thread(target=start_publishing, args=(publisher,))
        pub_thread.start()

        rospy.sleep(1)
        msg.linear.x = 0.0
        publisher.update_msg(msg)

        rospy.sleep(1)
        msg.linear.x = 0.1
        publisher.update_msg(msg)

        rospy.sleep(1)
        msg.linear.x = 0.0
        publisher.update_msg(msg)

        rospy.sleep(1)
        publisher.stop()
        rospy.loginfo("Shutting down ROS publisher.")

        pub_thread.join()

    except rospy.ROSInterruptException:
        pass
```

---

## 实现自己的子类（可选）

本类设计为基础类，一般无需继承。但如需扩展发布逻辑，可以：

- 重写 `publish()` 增加滤波、变换等功能
- 重写 `update_msg()` 实现多源数据合并或检查
