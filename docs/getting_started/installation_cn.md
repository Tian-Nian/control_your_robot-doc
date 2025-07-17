# 配置项目环境
### 基础环境
由于本项目是一个集成性项目, 因此如果不需要配置对应设备 / 训练环境, 则只需要安装基础环境即可.

``` bash
conda create -n my_robot python==3.10
conda activate my_robot
git clone git@github.com:Tian-Nian/control_your_robot.git
cd control_your_robot
pip install -r requirements.txt
```

### 指定设备/模型的额外环境
如果你需要使用到本项目为对应设备 / 模型训练所提供的集成, 则需要根据对应需求, 额外安装指定支持.
``` bash
# (可选)编译最新版lerobot
cd ..
git clone https://github.com/huggingface/lerobot.git
cd lerobot
conda install ffmpeg
pip install --no-binary=av -e .

# (可选)下载你的机械臂需要的python安装包
pip install piper_sdk
pip install Robotic_Arm

# 对于模型训练而言, RDT与openpi有自己的环境配置要求
# 请使用对应模型环境, 然后执行
cd ~/control_your_robot/
pip install -r requirements.txt

# 松灵机械臂安装SDK请参考:https://github.com/agilexrobotics
# 睿尔曼机械臂SDK请参考:https://develop.realman-robotics.com/robot/summarize/
# 大然机械臂SDK请参考:
# 所有机械臂如果涉及到原声代码编译或链接，会统一放置到./third_party/目录下
```

### 特殊编译需求环境
注意, 有些设备由于需要额外库支持, 因此需要手动编译对应项目或者下载编译后文件, 此部分在`third_party/`中有详细介绍.

**dr:** 

是大然aloha机械臂的控制底层代码, 无需编译, 使用提供的默认x64编译后文件, 如果是不同架构系统, 请与厂家联系获得支持.

**curobo:**

提供了IK / planner, 需要编译使用. 
```bash
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e . --no-build-isolation
```
**oculus_reader:**

用于控制VR遥操设备QuestVR, 需要编译使用.
``` bash
正在测试, 请等待...
```