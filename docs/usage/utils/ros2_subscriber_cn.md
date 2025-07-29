## ROS2Subscriber 类

用于封装 ROS 2 的订阅器功能，支持线程安全地获取最新消息，并可选使用自定义回调处理接收到的消息。

### 类定义
```python
class ROS2Subscriber(Node):
```

### 成员属性

| 名称         | 类型             | 描述                               |
|--------------|------------------|------------------------------------|
| topic_name   | str              | 要订阅的 topic 名称                |
| msg_type     | Type             | 订阅的 ROS2 消息类型               |
| latest_msg   | Any              | 最近接收到的消息                   |
| lock         | threading.Lock   | 保证多线程读取 `latest_msg` 安全  |
| user_call    | Optional[Callable] | 用户自定义的消息回调函数（可选） |
| subscription | Subscription     | ROS2 订阅器对象                    |

### 函数列表

#### `__init__(self, node_name: str, topic_name: str, msg_type, call: Optional[Callable] = None)`
创建一个 ROS2 订阅器。

- **参数**:
  - `node_name` (str): ROS2 节点名称。
  - `topic_name` (str): 要订阅的话题名称。
  - `msg_type` (Type): 消息类型（如 `BunkerRCState`）。
  - `call` (Callable, optional): 接收到消息时的用户回调函数。

#### `callback(self, msg)`
默认回调函数，记录最新消息，并调用用户自定义回调（若提供）。

- **参数**:
  - `msg`: 接收到的 ROS2 消息。

#### `get_latest_data(self)`
返回最近接收到的消息。

- **返回值**:
  - `latest_msg`: 上一次接收到的 ROS2 消息对象。

---

## 示例用法：监听 BunkerRCState

以下示例展示了如何创建 `ROS2Subscriber` 节点，订阅 `/bunker_rc_state` 话题，并通过用户自定义回调打印数据。

```python
import rclpy
from rclpy.node import Node
from threading import Lock
from typing import Callable, Optional
from bunker_msgs.msg import BunkerRCState  # 替换为你使用的消息类型
import time

class ROS2Subscriber(Node):
    def __init__(self, node_name: str, topic_name: str, msg_type, call: Optional[Callable] = None):
        super().__init__(node_name)
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.latest_msg = None
        self.lock = Lock()
        self.user_call = call

        self.subscription = self.create_subscription(
            msg_type,
            topic_name,
            self.callback,
            10  # QoS depth
        )

    def callback(self, msg):
        with self.lock:
            self.latest_msg = msg
            if self.user_call:
                self.user_call(msg)

    def get_latest_data(self):
        with self.lock:
            return self.latest_msg

def custom_callback(msg):
    print(f"Received: SWA={msg.swa}, SWC={msg.swc}")

def main():
    rclpy.init()

    subscriber_node = ROS2Subscriber(
        node_name='rc_state_listener',
        topic_name='/bunker_rc_state',
        msg_type=BunkerRCState,
        call=custom_callback  # 可选
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber_node, timeout_sec=0.1)
            msg = subscriber_node.get_latest_data()
            if msg:
                print(msg)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 实现自己的子类（如需扩展）

### 必须实现

- `__init__` 中提供具体的 topic 名称、消息类型。
- 若需处理消息，重写 `callback()` 或传入 `call` 回调函数。

### 可选实现

- 可以扩展 `get_latest_data()` 实现更多缓存、解析等功能。
- `call` 函数可以用于消息解码、记录、状态更新等自定义逻辑。