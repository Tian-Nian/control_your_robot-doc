# 快速上手!
由于本项目实现了部分测试样例, 如机械臂测试样例, 视觉模拟样例, 完整机器人模拟样例, 因此可以在没有任何实体的情况下快速了解本项目的整体框架.
由于没涉及任何本体, 所以安装环境只需要执行:
```
 pip install -r requirements.txt
```  
本项目有特殊的调试参数, 分为:"DEBUG", "INFO", "ERROR", 如果想要完整看到数据的流程, 可以设置为"DEBUG".
```bash
export INFO_LEVEL="DEBUG"
```
或者可以在对应main函数中引入:
```python
import os
os.environ["INFO_LEVEL"] = "DEBUG" # DEBUG , INFO, ERROR
```
1. 数据采集测试
```bash
# 多进程(通过时间同步器实现更严格的等时间距采集)
python example/collect/collect_mp_robot.py
# 多进程(对每个元件单独进程采集数据)
python example/collect/collect_mp_component.py
# 单线程(会存在一些由于函数执行导致的延迟堆积)
python example/collect/collect.py
```
2. 模型部署测试
```bash
# 跑一个比较直观的部署测试代码
python example/deploy/robot_on_test.py
# 实现的通用部署脚本
bash deploy.sh
```
3. 远程部署数据传输
```bash
# 先启动服务器, 模仿推理端(允许多次连接, 监听端口)
python scripts/server.py
# 本地, 获取数据并执行指令(示例只执行了10次)
python scripts/client.py
```
4. 一些有意思的代码
```python
# 采集对应的关键点, 并且进行轨迹重演
python scripts/collect_moving_ckpt.py 
# sapien仿真, 请参考planner/README.md
```
5. 调试对应的一些代码
```bash
# 由于controller与sensor有__init__.py, 所以需要按照-m形式执行代码
python -m controller.TestArm_controller
python -m sensor.TestVision_sensor
python -m my_robot.test_robot
```