# 设备 INS 直接建图实验

独立实验：读取设备原始 INSPVAX 轨迹，按时间插值位置和姿态，结合候选 IMU/设备轴间旋转，将 Livox 原始点直接投影到局部 ENU。没有二次 INS 滤波和位姿图优化，不依赖旧项目模块。输出是反射率几何点云，尚非经过标定验收的测绘产品。

数据：ROS1 bag `/livox/lidar`，及同录原始串口 dump 中经 CRC 验证的 INSPVAX。先在项目根目录运行 `python3 extract_inspvax.py --raw /path/ins_raw.bin --output /path/inspvax_raw.csv` 提取轨迹。轨迹 GPS 周秒以固定 GPS-UTC 差 18 秒转换，须结合采集时钟验证。候选旋转来自同一包的陀螺角增量拟合，非独立标定。LiDAR 原点与 INS 原点平移暂设零；INS 姿态的装配定义及天线杆臂未验证。按设备状态及位置跳变检测异常，并仅排除超限的插值区间，不补造轨迹。末端超出 INS 轨迹的帧不外推。

```bash
python3 map_direct.py --bag /path/input.bag --ins /path/inspvax_raw.csv --output /path/new_output
```

需要 ROS1 rosbag、NumPy、SciPy。该实现直接使用 INSPVAX 1 Hz 姿态插值进行逐点变换，不宣称高频 IMU 去畸变。请独立核查轨迹、时间、安装外参和点云几何后再用于测绘。

## 2026-11-03 板卡参数核查

板卡 FAST-LIVO2 的 `config/camera_pinhole.yaml` 给出 1440×1080、fx=1038.6749、fy=1040.0463、cx=703.0561、cy=582.8034。`config/avia.yaml` 和 `avia_gnssimu.yaml` 给出同一组 Rcl/Pcl。候选值存于 `calibration_board_20261103.json`。这只是当前板卡配置，尚未证明对应 11 月 2 日录制时的镜头、安装与图像处理；尚需检查 FAST-LIVO2 代码中的矩阵方向和实际投影。

板卡 LiDAR/内置 IMU 配置为旋转单位阵和平移 [0.04165, 0.02326, -0.0284] m；这不等同于 LiDAR 到 GNSS/INS 机体系的完整外参。项目当前使用的 LiDAR 到设备姿态系旋转由同一 bag 的角速度拟合得到，平移仍为零假设，不能称为已标定。

着色时可显式使用板卡候选相机参数：

```bash
python3 color_direct.py --bag /path/input.bag --ins /path/inspvax_raw.csv --output /path/new_output --calibration calibration_board_20261103.json
```

相机曝光时间未在目标包中记录；默认 -0.026 秒只是最近帧匹配得到的近似，不能证明曝光同步。先抽样查看投影叠加与边缘，检查坐标方向、外参、时间和机械安装，再执行全包着色；真彩覆盖率不是几何质量或着色准确度指标。

## 开源与数据

本项目自有代码以 MIT 许可证发布。ROS、NumPy、SciPy、OpenCV、three.js 等依赖各自遵循其原许可证；请在分发依赖时自行遵守。仓库不包含个人原始日志、bag、点云和实地坐标。网页使用 CDN 时需要联网。

