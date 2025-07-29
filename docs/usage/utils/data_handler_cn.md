## 函数说明

### apply_local_offset_to_global_pose(T_offset, T_current)
在当前末端位姿 `T_current`（全局坐标）上，应用局部坐标系下的变换 `T_offset`。  
等价于：`T_target = T_current @ T_offset`  
返回偏移后的全局位姿矩阵 `T_target`。

---

### euler_to_matrix(euler, degrees=False)
将欧拉角和平移位置转换为4x4齐次变换矩阵。  
- 输入 `euler` 是6维数组 `[x, y, z, roll, pitch, yaw]`，旋转顺序为 'xyz'。  
- `degrees` 指明角度单位（默认弧度）。  
- 返回4x4 numpy矩阵。

---

### matrix_to_xyz_rpy(matrix)
从4x4变换矩阵中提取平移和欧拉角（RPY）  
- 输入4x4矩阵  
- 输出6维数组 `[x, y, z, roll, pitch, yaw]`，欧拉角为弧度。

---

### compute_rotate_matrix(pose)
将位姿 `[x, y, z, roll, pitch, yaw]` 转换为4x4齐次变换矩阵，欧拉角顺序为'XYZ'。  
- 输入：6维位姿  
- 输出：4x4变换矩阵

---

### compute_local_delta_pose(base_pose, target_pose)
计算基于 `base_pose` 局部坐标系下到 `target_pose` 的位姿增量。  
- 输入：两个6维位姿数组  
- 输出：6维增量位姿 `[delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw]`  
- 包括旋转和平移的局部增量。

---

### apply_local_delta_pose(base_pose, delta_pose)
将局部坐标系下的增量位姿 `delta_pose` 应用到 `base_pose`，得到全局目标位姿 `target_pose`。  
- 输入：`base_pose` 和 `delta_pose`，均为6维数组  
- 输出：6维目标位姿

---

### get_item(Dict_data: Dict, item)
从嵌套字典中获取数据。  
- `item` 可为字符串（支持点分割访问）或字符串列表（逐个拼接列）  
- 返回对应数据或拼接的数组。

---

### hdf5_groups_to_dict(hdf5_path)
读取HDF5文件中的所有group及其数据集，返回嵌套字典结构。  
- 输入HDF5文件路径  
- 输出字典：键为group名，值为该group中所有数据集的numpy数组。

---

### get_files(directory, extension)
遍历目录，返回所有匹配指定扩展名的文件路径列表。

---

### debug_print(name, info, level="INFO")
带颜色分级输出调试信息。  
- 支持级别：DEBUG, INFO, WARNING, ERROR  
- 根据环境变量 `INFO_LEVEL` 控制输出等级。

---

### is_enter_pressed()
非阻塞检测用户是否按下回车键，返回布尔值。

---
