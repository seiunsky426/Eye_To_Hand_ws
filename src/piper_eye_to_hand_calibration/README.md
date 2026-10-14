# RS1／RS3 与 Piper 完整标定（ROS Noetic）

本工作区只负责标定与数据保存。顺序是 **ROS 内参 → 双机棋盘格 5＋1 → RS1 手动手眼 → 外参合成**。
全部入口位于 `piper_eye_to_hand_calibration`；不依赖 `eyetohandauto_ws` 的 Python 包，也不启动自动运动。
手眼算法是本包 OpenCV Tsai，末端位姿来自 Piper 官方驱动 `/end_pose`，不是声称存在另一套“Piper 官方标定算法”。

## 1. 固定约定与数据

| 项目 | 配置 |
|---|---|
| RS1 / RS3 | `346522071783` / `934222070377` |
| 彩色流 | 两机均为 **640×480、30 FPS**；深度、红外、点云关闭 |
| 棋盘 | **11×8 内角点（88 点）**，每格 **25 mm**；不是 11×8 个格子 |
| 双机采集 | S01～S05 拟合；S06 独立验证；每个摆放 30 对静止图像 |
| 手眼板 | `DICT_4X4_50`，ID 18，黑色编码框实测边长 **100 mm**，刚性固定在末端 |
| 手眼输入 | `/end_pose`，按 `arm_base → tool` 解释；帧名只允许空值或 `arm_base` |
| 变换记法 | `A_T_B` 将 B 中的坐标变换到 A：`p_A = A_T_B @ p_B` |

**两台相机及支架从双机外参采集开始，到手眼标定结束都不能移动。** 改动内参、图像规格或相机安装后，重新创建会话。
旧 640×360 内参不能用于本流程，不会缩放代用。默认相机参数只是请求；必须检查真实输出的尺寸和频率。

手动原点：两幅图各点击**同一个物理内角点阵顶点**。例如固定选照片中文字附近的左下内角点；
不能把“两幅画面各自最左下点”当成同一个物理点。+X 从原点沿 11 点长边，+Y 沿 8 点短边，+Z=X×Y。
两机应看到同一印刷正面。界面显示 0、10、77、87 与 +X/+Y；内角点阵跨度为 250×175 mm。
鼠标只选编号方向，实际计算使用自动检测的亚像素角点；不增加 25 mm ArUco 定向。

新数据分为四类，每次使用新时间戳目录：

```text
Calculation Results/
  intrinsics/rs1/<时间>/     # 同理 rs3：ROS 原包、图像、ost.yaml、intrinsics.yaml、manifest.yaml
  stereo/<时间>/            # 6 次摆放、失败尝试、dataset.yaml、stereo_result.yaml
  handeye/<时间>/           # 绑定信息、逐次样本快照、带时间戳的求解结果
  composed/<时间>/          # composition.yaml、rs1_extrinsics.yaml、rs3_extrinsics.yaml
```

原 `rs1/`、`rs2/`、`rs3/` 历史结果保留，旧内参在 `legacy_intrinsics/`，不自动作为新流程输入。

## 2. 构建与终端准备

依赖已有 ROS Noetic、RealSense ROS 驱动、`camera_calibration`、OpenCV（含 ArUco）、NumPy、PyYAML。
采集界面需要桌面显示。缺少 ROS 标定工具时，由用户安装 `ros-noetic-camera-calibration`。

构建终端：

```bash
cd /home/zhengsihze/Eye_To_Hand_ws
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rospack find piper_eye_to_hand_calibration
```

随后每个操作终端先执行：

```bash
cd /home/zhengsihze/Eye_To_Hand_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
```

终端 A 保持一个 ROS master（已有可用 master 时复用）：

```bash
roscore
```

终端 B 启动相机。先退出占用这两台设备的旧相机进程，随后执行：

```bash
roslaunch piper_eye_to_hand_calibration cameras.launch mode:=both
```

单机启动分别为 `mode:=rs1` 或 `mode:=rs3`；不要与 `both` 同时启动。这个入口只启动相机及其内部 TF。

终端 C 检查，每条 `hz` 看完按 Ctrl+C 再运行下一条：

```bash
rosparam get /rs1/realsense2_camera/serial_no
rosparam get /rs3/realsense2_camera/serial_no
rostopic echo -n 1 /rs1/color/camera_info
rostopic echo -n 1 /rs3/color/camera_info
rostopic hz /rs1/color/image_raw
rostopic hz /rs3/color/image_raw
```

必须有新鲜图像、正确序列号、640×480 尺寸及对应的 `rsN_color_optical_frame`，彩色流约 30 Hz。
只有节点名或序列号参数不代表相机已经就绪。两机独立曝光，不声明硬件同步。

## 3. 用 ROS 自带工具分别标定内参

**保留终端 C 作为控制终端**：下面导入的路径保存在它的变量中，双机与合成步骤继续使用这个终端。
终端 D 先标定 RS1：

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 11x8 --square 0.025 --no-service-check \
  --camera_name rs1_346522071783_640x480 \
  image:=/rs1/color/image_raw camera:=/rs1
```

手持棋盘覆盖不同画面位置、距离和倾斜角度；保持棋盘平整、角点清晰。等待 ROS 工具覆盖指标满足条件，
点击 **CALIBRATE**，完成后点 **SAVE**。使用保存文件，不依赖 COMMIT 或驱动 `set_camera_info` 服务。
ROS 默认写入 `/tmp/calibrationdata.tar.gz`，下一次 SAVE 会覆盖它。

**先在终端 C 导入 RS1，再开始 RS3 标定**：

```bash
RS1_INTRINSICS=$(rosrun piper_eye_to_hand_calibration import_intrinsics.py --camera rs1)
printf 'RS1 内参：%s\n' "$RS1_INTRINSICS"
test -f "$RS1_INTRINSICS"
```

成功返回 `Calculation Results/intrinsics/rs1/<时间>/intrinsics.yaml` 的绝对路径。
脚本复制原始 tar.gz、提取原图及 ost 文件、保存 SHA-256 清单；不覆盖既有目录。相机名必须与命令一致，
它用于防止把上一次相机的归档导入另一台，但不能替代现场序列号核对。

退出终端 D 的 RS1 标定窗口，然后标定 RS3：

```bash
rosrun camera_calibration cameracalibrator.py \
  --size 11x8 --square 0.025 --no-service-check \
  --camera_name rs3_934222070377_640x480 \
  image:=/rs3/color/image_raw camera:=/rs3
```

同样 CALIBRATE → SAVE。终端 C 立即导入：

```bash
RS3_INTRINSICS=$(rosrun piper_eye_to_hand_calibration import_intrinsics.py --camera rs3)
printf 'RS3 内参：%s\n' "$RS3_INTRINSICS"
test -f "$RS3_INTRINSICS"
```

退出 ROS 内参窗口，保留相机进程。两阶段均使用文件 K/D；驱动 CameraInfo 用于规格、时间、帧名及变化检查，
不会覆盖文件内参。历史归档可用 `--archive /绝对路径/calibrationdata.tar.gz` 导入，`--output` 指定尚不存在的新目录。

## 4. 双机棋盘格：5 个拟合＋1 个独立验证

终端 C（沿用上面的内参变量）：

```bash
STEREO_DIR="$PWD/Calculation Results/stereo/$(date -u +%Y%m%dT%H%M%S.%NZ)"
roslaunch piper_eye_to_hand_calibration stereo_calibration.launch \
  rs1_intrinsics:="$RS1_INTRINSICS" \
  rs3_intrinsics:="$RS3_INTRINSICS" \
  output_directory:="$STEREO_DIR"
```

界面左 RS1、右 RS3；以原图像素显示，点击坐标不会进入拟合。

1. 让两机完整看到棋盘，保持静止，在两幅图分别点击同一个物理原点。
2. 核对 0 点、+X 和 +Y；按 **空格**开始保存 30 对图像，采集期间保持静止。
3. 保存后移动棋盘，重新点击两个原点。S01～S05 应覆盖不同位置、距离和适度倾斜；不要只在一个平面位置微动。
4. S06 换一个未使用位置，只用于独立验证。第六次完成后自动求解并显示 PASS／FAIL；按 **Q**退出界面。

**R** 清除当前选择；正在采集时按 R 或退出，会保留该次已保存图像与失败原因，不把它算成成功摆放。
丢角点、棋盘移动、相机断流、时间过期、重复帧、帧名/内参变化、写盘失败均中止本次采集并清空方向选择。
RGB/CameraInfo 时间跨度最多 80 ms；采集只用于静止棋盘。单次 30 对采集最多等待 45 秒。
原点选择可检查镜像式方向冲突；选错同一对角方向仍可能无法仅靠单帧发现，必须核对物理原点与拟合/验证结果。

输出：原始 PNG、逐帧时间戳/角点/内参、选择记录、失败原因、`dataset.yaml` 与 `stereo_result.yaml`。
固定 K/D 求解，仅 S01～S05 的每组角点中位数进入拟合，S06 不参与拟合。

```text
rs1_optical_T_rs3_optical：RS3 optical → RS1 optical
rs3_optical_T_rs1_optical：上式的逆
```

PASS 要求 S06 双向跨相机重投影 RMS ≤0.8 px，RGB/PnP 角点差异中位数 ≤5 mm、P95 ≤8 mm。
这不是深度测量误差，也不是全工作空间的绝对精度验收。失败仍保存全部结果，不能进入正式手眼绑定/合成；
不自动删异常数据、不放宽阈值。重新采集使用新目录。

回到终端 C：

```bash
STEREO_RESULT="$STEREO_DIR/stereo_result.yaml"
cat "$STEREO_RESULT"
```

如果换了控制终端，用以下输入恢复路径：

```bash
read -r -p '粘贴 RS1 intrinsics.yaml 的绝对路径：' RS1_INTRINSICS
read -r -p '粘贴 stereo_result.yaml 的绝对路径：' STEREO_RESULT
```

## 5. RS1 与 Piper 手动手眼

移走自由摆放的棋盘，保持相机支架不动，在末端刚性固定 ID 18、100 mm 黑框 ArUco。
标定包不启动/使能/控制机器人。官方驱动是“关闭自动使能的反馈来源”；`auto_enable=false` **不保证 CAN 零发送**，
驱动仍具有控制接口并可能执行 SDK 初始化。避免其他程序向其控制接口发送命令。

终端 E 由操作者配置 CAN 并启动反馈（这些命令涉及实际设备，不属于离线验证）：

```bash
cd /home/zhengsihze/piper_ws/src/piper_libs
bash can_activate.sh can0 1000000
source /opt/ros/noetic/setup.bash
source /home/zhengsihze/piper_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
rosrun piper piper_ctrl_single_node.py \
  _can_port:=can0 _auto_enable:=false _gripper_exist:=true
```

不要启动默认自动使能 launch，不与其他 Piper 控制节点混跑。机械臂由操作者按已有手动/示教流程摆姿态。

终端 C 先确认末端反馈持续更新，再 Ctrl+C 退出频率观察，启动标定：

```bash
rostopic hz /end_pose
```

```bash
roslaunch piper_eye_to_hand_calibration calibrate_camera.launch \
  intrinsics_file:="$RS1_INTRINSICS" stereo_result:="$STEREO_RESULT"
```

节点检查双机报告通过、RS1 序列号、640×480 规格和内参哈希一致。启动后产生新的 `handeye/<时间>/session.yaml`。
终端 F 可查看检测：

```bash
rqt_image_view /rs1/eye_to_hand/debug_image
```

终端 G 开启手动 CLI：

```bash
rosrun piper_eye_to_hand_calibration calibration_cli.py _camera:=rs1
```

每次换姿态后静止至少 1 秒，确认 ID 18 清晰、检测图像绿色 OK，再短按一次 Enter。
采集至少 15 个姿态，建议 20～30 个，覆盖不同位置与多轴旋转。没有自动 20 点运动。

| 按键 | 行为 |
|---|---|
| Enter | 保存一个同步样本；成功显示 `captured sample N` |
| d / r | 删除最后一个当前样本 / 清空当前样本；已归档快照保留 |
| q | Tsai 求解，PASS 或 FAIL 都保存新结果；只有 PASS 时 CLI 退出 |
| c | 不求解退出 CLI；已保存样本快照保留 |

原有静止、重复位姿与样本数检查保留。PASS 门槛为安装变换 `tool_T_marker` 的内部一致性：
平移 RMS ≤10 mm、旋转 RMS ≤1.5°，不等于独立物理精度通过。
结果文件完整路径显示在 CLI；在终端 C 按 Ctrl+C 退出标定节点后继续下一步。不要关闭相机或改变支架。

## 6. 显式合成与查看

终端 C 输入刚才 CLI 给出的手眼结果路径：

```bash
read -r -p '粘贴本次 PASS 手眼 YAML 的绝对路径：' HANDEYE_RESULT
COMPOSED_DIR="$PWD/Calculation Results/composed/$(date -u +%Y%m%dT%H%M%S.%NZ)"
rosrun piper_eye_to_hand_calibration compose_results.py \
  --stereo "$STEREO_RESULT" --handeye "$HANDEYE_RESULT" --output "$COMPOSED_DIR"
cat "$COMPOSED_DIR/composition.yaml"
cat "$COMPOSED_DIR/rs1_extrinsics.yaml"
cat "$COMPOSED_DIR/rs3_extrinsics.yaml"
```

计算关系：

```text
arm_base_T_rs3_optical = arm_base_T_rs1_optical @ rs1_optical_T_rs3_optical
arm_base_T_rsN_link   = arm_base_T_rsN_optical @ inverse(rsN_link_T_optical)
```

拒绝条件包括任一阶段未通过、来源报告哈希不符、RS1 内参不一致、基座或相机帧名不匹配、link/optical 关系变化。
输出同时包含 optical 正逆矩阵、camera_link 外参、两阶段来源哈希及质量报告。
`physical_validation: PENDING` 始终保留；求解和合成均不自动发布 TF。

需要明确加载结果时才运行：

```bash
roslaunch piper_eye_to_hand_calibration publish_results.launch directory:="$COMPOSED_DIR"
```

另一个终端查看：

```bash
rosrun tf tf_echo arm_base rs1_color_optical_frame
rosrun tf tf_echo arm_base rs3_color_optical_frame
```

发布器会验证 PASS、两机身份、帧名和当前 RealSense 内部 TF。它发布 `arm_base → rsN_link`，驱动提供
`rsN_link → rsN_color_optical_frame`，避免重复父节点。保留单机显式入口 `publish_camera_result.launch`。
显式发布成功只表示 TF 已加载，正式定位还需用未参与标定的已知物理点/姿态验证。

## 7. 常见拒绝与恢复

| 提示 | 处理 |
|---|---|
| `camera_name must be ...` | 核对 ROS 标定时的 camera_name 与实际相机，确认没有导入另一台的默认临时归档 |
| `real 640x480 calibration required` | 用正确模式重新采集，不能缩放旧内参 |
| `88 inner corners not detected` / cells too small | 完整露出棋盘、调整距离/曝光/倾角，不改变板参数 |
| `board moved` / `preview expired` | 保持静止并重新选择两幅图的原点 |
| `duplicate placement` | 换一个明显不同位置或姿态，不能靠重复帧凑样本 |
| `stereo validation failed` | 查看 S06 双向误差，检查原点、内参、板尺寸和相机安装；新目录重采 |
| `no synchronized Piper/marker observation` | 检查 `/end_pose`、marker_pose 和新鲜时间戳 |
| `Piper is moving` / `near-duplicate pose` | 静止后再采，或更换明显不同姿态 |
| `intrinsics/stereo result changed` | 恢复原文件或创建新绑定会话，不原地替换会话输入 |

## 8. 软件验证与清理备份

```bash
cd /home/zhengsihze/Eye_To_Hand_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_piper_eye_to_hand_calibration
catkin_test_results build/test_results/piper_eye_to_hand_calibration
```

测试使用临时目录与合成图像/位姿，覆盖内参归档、棋盘排序、5＋1隔离、30 对图像存储、过期/移动拒绝、
Tsai 回归及矩阵合成。软件通过不代表相机实测、双视图 GUI 人工验收或机械臂物理标定已完成。

`CLEANUP.json` 记录清理范围。移出的规划包、运行材料及改动前原件已存入工作区外的时间戳备份，
包含 `before_cleanup.tar.gz`、`manifest.json` 和 `SHA256SUMS`。先校验再解压到**新的恢复目录**，不要覆盖当前工作区。
参考工作区 `eyetohandauto_ws` 和 `eyetohandauto_cleanup_backups` 未修改。
