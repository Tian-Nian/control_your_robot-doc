## 类说明

### class TestModel
一个用于模拟推理动作的测试模型类，包含随机生成动作、更新观测窗口和语言指令等功能。

---

### 初始化函数 `__init__(self, model_path, task_name, DoFs=6, is_dual=True, INFO="DEBUG")`
- `model_path`: 模型路径（目前未使用，仅占位）
- `task_name`: 任务名称，用于加载对应语言指令JSON文件
- `DoFs`: 机器人自由度数，默认6
- `is_dual`: 是否为双臂机器人，默认True
- `INFO`: 日志等级，默认DEBUG  
初始化时加载任务指令，设置默认图片大小224x224。

---

### `set_img_size(self, img_size)`
设置模型输入的图片尺寸，`img_size`为元组 (width, height)。

---

### `random_set_language(self)`
随机从对应任务的 JSON 指令文件中选择一条语言指令，保存到 `self.instruction`。

---

### `update_observation_window(self, img_arr, state)`
更新观测缓存窗口，`img_arr`为包含多路摄像头图片的数组列表，`state`为状态向量。  
- 会将图片从 HWC 转换为 CHW 格式。  
- 检查状态维度是否符合单臂或双臂模型的输入要求。  
- 将状态、图像和语言指令保存到 `self.observation_window`。

---

### `get_action(self)`
基于当前观测窗口模拟推理动作，返回一个动作数组。  
- horizon长度为3，动作维度依`is_dual`和`DoFs`设定。  
- 动作为随机生成的角度和夹爪开合控制。  
- 内部使用 `time.sleep` 模拟推理延迟。  
- 返回形状为 `(3, 动作维度)` 的 numpy 数组。

---

### `reset_obsrvationwindows(self)`
重置语言指令和观测窗口缓存。

---

## 示例代码

```python
if __name__ == "__main__":
    import os
    os.environ["INFO_LEVEL"] = "INFO"
    
    DoFs = 14
    model = TestModel("test", DoFs=DoFs, is_dual=True)
    
    height, width = 480, 640
    img_arr = [
        np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8),
        np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8),
        np.random.randint(0, 256, size=(height, width, 3), dtype=np.uint8)
    ]
    
    state = np.random.rand(DoFs) * 3.1515926
    model.update_observation_window(img_arr, state)
    action = model.get_action()
    model.reset_obsrvationwindows()
