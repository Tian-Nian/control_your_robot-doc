# 模型部署示例
## deploy介绍

`deploy`中提供了两种部署模式:
### 实现自己模型的部署脚本
可以参考多个robot_on_model.py脚本, 主要不同点在于模型初始化与用于将数据转化为模型推理数据的`input_transform()`函数. 

对于你的模型,应当在policy/your_model/路径下有`inference_model.py`文件, 该文件拥有你的模型的封装类, 并且拥有:
- `update_observation_window()`: 用于管理模型的输入参数, 由于部分模型需要前文信息, 因此需要用滑动窗口管理数据.
— `get_action()`: 根据当前的滑动窗口数据进行推理, 获取动作序列.
- `reset_obsrvationwindows()`: 重置滑动窗口, 用于开启新一轮推理.

如果你模型需要其他的信息, 直接在`input_transform()`中添加对应传感器的返回值, 并转化为模型

### 使用通用部署脚本

