# 双向通信套接字类

## 介绍
`BiSocket` 是一个基于 TCP 套接字的双向通信类，支持从连接中接收消息并自动调用 `handler` 处理逻辑。可选启用“回传模式”（`send_back=True`），即在处理后将结果返回给发送方。该类适用于客户端-服务端架构中状态、控制、数据的同步交互。

## 类定义
```python
class BiSocket:
```

## 成员属性

| 属性名         | 类型               | 说明                                                             |
|--------------|------------------|------------------------------------------------------------------|
| conn         | socket.socket    | 已连接的套接字对象                                               |
| handler      | Callable         | 接收数据时回调执行的函数，输入为消息内容（一般为 Dict[Any]）         |
| send_back    | bool             | 是否开启“回传模式”，若为 True，则会将 handler 的返回值通过 send() 发回 |
| running      | threading.Event  | 控制接收线程是否继续运行的事件标志                                |
| receiver_thread | threading.Thread | 后台接收线程，负责持续监听并处理来自连接的数据                    |

## 函数

### **__init__(self, conn: socket.socket, handler, send_back=False)**
初始化一个 `BiSocket` 实例。

- 参数：
  - `conn` (socket.socket): 已建立连接的 TCP 套接字
  - `handler` (Callable): 消息处理函数，接受消息数据作为参数
  - `send_back` (bool): 是否启用处理结果的回传，默认为 False

- 返回：无

---

### **_recv_exact(self, n: int) -> Optional[bytes]**
从连接中接收精确长度的字节数据。

- 参数：
  - `n` (int): 要接收的字节数

- 返回：
  - `bytes`：接收到的数据；若连接关闭或出错则返回 None

---

### **_recv_loop(self)**
后台线程函数，持续监听消息输入并执行 handler。若 `send_back=True` 则将处理结果通过 send() 发回。

- 参数：无
- 返回：无（死循环，直到连接关闭）

---

### **send(self, data: Dict[Any])**
将数据发送给对端。使用 pickle 序列化，并发送带长度前缀的数据包。

- 参数：
  - `data` (Dict[Any])：要发送的数据字典，支持任何可序列化对象

- 返回：无（发送失败将关闭连接）

---

### **close(self)**
主动关闭连接，停止接收线程。

- 参数：无
- 返回：无

---

## 使用示例

```python
# 假设你已经在 server 端使用 socket.accept() 得到了 conn:
def my_handler(data):
    print("Received:", data)
    return {"status": "ok"}

bisocket = BiSocket(conn, handler=my_handler, send_back=True)
bisocket.send({"command": "move", "value": 1.0})
```

---

## 注意事项

- 若 `handler` 抛出异常或处理失败，系统会记录错误但不中断主循环。
- 本类设计用于长连接场景，如需频繁建立关闭连接，需增加额外状态管理。
- 若远端关闭连接或发送非法数据（反序列化失败），会自动关闭当前连接。
