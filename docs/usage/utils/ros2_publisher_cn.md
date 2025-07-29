# 类定义文档：ROS2Publisher

---

## 类介绍

```python
class ROS2Publisher(Node):
```

一个基于 ROS 2 的通用发布器类，支持一次性发布和周期性持续发布。适用于实时控制命令发送，如控制机器人运动等。

---

## 类继承

| 类名         | 继承自       |
|--------------|------------|
| ROS2Publisher | `rclpy.node.Node` |

---

## 类定义代码块

```python
class ROS2Publisher(Node):
    def __init__(self, topic_name, msg_type, continuous=True):
        ...
```

---

## 成员属性说明

| 属性名           | 类型                 | 说明                                       |
|----------------|--------------------|------------------------------------------|
| `topic_name`    | `str`               | 发布的 ROS2 topic 名称                         |
| `msg_type`      | `Type`              | ROS2 消息类型，如 `Twist`、`PoseStamped` 等     |
| `continuous`    | `bool`              | 是否持续发布消息（默认为 True）                  |
| `publisher`     | `Publisher`         | ROS2 发布器对象                                 |
| `pub_msg`       | `Any`               | 当前待发布的消息（会被周期性或一次性发布）            |
| `shutdown_event`| `threading.Event`   | 控制持续发布线程是否终止                          |

---

## 函数接口

### `__init__`

```python
def __init__(self, topic_name, msg_type, continuous=True):
```

**功能描述**：初始化发布器节点，创建发布器对象。

**参数**：

| 参数名        | 类型    | 说明                                       |
|-------------|-------|------------------------------------------|
| `topic_name` | `str`  | 发布的 ROS2 话题名                              |
| `msg_type`   | `Type` | 发布的消息类型（如 `Twist`、`PoseStamped`）       |
| `continuous` | `bool` | 是否持续发布消息，默认为 True（每 10ms 发布一次） |

**返回值**：无

---

### `publish_once`

```python
def publish_once(self):
```

**功能描述**：立即将当前 `pub_msg` 发布一次。

**参数**：无  
**返回值**：无

---

### `continuous_publish`

```python
def continuous_publish(self, interval_sec=0.01):
```

**功能描述**：以固定时间间隔持续发布 `pub_msg`。

**参数**：

| 参数名          | 类型     | 说明                   |
|---------------|--------|----------------------|
| `interval_sec` | `float` | 发布时间间隔，单位为秒，默认 0.01 秒 |

**返回值**：无

---

### `update_msg`

```python
def update_msg(self, msg):
```

**功能描述**：更新待发布的消息内容。

**参数**：

| 参数名 | 类型  | 说明             |
|------|-----|----------------|
| `msg` | `Any` | ROS2 消息对象       |

**返回值**：无

---

### `stop`

```python
def stop(self):
```

**功能描述**：发送停止信号终止持续发布线程，并输出日志。

**参数**：无  
**返回值**：无

---

## 示例主程序

```python
def main():
    rclpy.init()
    pub_node = ROS2Publisher('/cmd_vel', Twist, continuous=True)

    msg = Twist()
    msg.linear.x = 0.02
    pub_node.update_msg(msg)

    pub_thread = Thread(target=pub_node.continuous_publish)
    pub_thread.start()

    try:
        rclpy.spin_once(pub_node, timeout_sec=0.1)
        time.sleep(1)

        msg.linear.x = 0.0
        pub_node.update_msg(msg)
        time.sleep(1)

        msg.linear.x = -0.02
        pub_node.update_msg(msg)
        time.sleep(1)

        msg.linear.x = 0.0
        pub_node.update_msg(msg)
        time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        pub_node.stop()
        pub_thread.join()
        pub_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 实现自己的子类（可选）

若需要在发布前添加日志、消息校验等，可继承本类并重写 `update_msg()` 或 `publish_once()` 函数：

```python
class MyPublisher(ROS2Publisher):
    def update_msg(self, msg):
        print("准备发布:", msg)
        super().update_msg(msg)
```

---

## 多线程注意事项

- 发布器默认启动线程持续发布，使用 `threading.Event` 控制退出。
- 在主线程中仍可使用 `rclpy.spin_once()` 来处理 ROS2 回调（如日志或参数服务）。
