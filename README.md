# Eye_To_Hand_ws：双相机 与 Piper 标定

完整操作见 [piper_eye_to_hand_calibration README](src/piper_eye_to_hand_calibration/README.md)。

流程：ROS 两机内参 → 手动原点棋盘格 5＋1 双机外参 → RS1 手动 Tsai 手眼 → 外参合成。
统一 640×480@30，棋盘 11×8 内角点、25 mm，末端 ArUco ID 18、100 mm。

首次构建：

```bash
cd /home/zhengsihze/Eye_To_Hand_ws
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

仅保留标定功能与历史数据；不提供机械臂自动运动。新内参必须实际采集，不能使用原 640×360 内参代替。
`Calculation Results` 保存历史和新会话；`CLEANUP.json` 记录工作区外可恢复备份与清理清单。
