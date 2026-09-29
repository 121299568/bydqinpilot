# 秦 PLUS DM-i 全功能控制 — 路测日志核对与排查指南

> 适用文件：`controlsd.py`（基于 `controlsd.py1` 全功能版 + 秦专属/设备定制补回）  
> 功能范围：横向扭矩控制、弯道减速、雷达+视觉静态前车识别（SS）、E2E 红绿灯停起步（TL）、df_tune 跟车距离。  
> 用途：上车试跑时，确认每个功能“真的在按预期工作”，并在异常时快速定位。

---

## 0. 先看车口是否真的接管（最优先）

启动后第一件事——确认设备认为这是“秦”并进入控制，而不是掉进只读：

- **正常接管**日志（在 `__init__` 命中秦车口时打印）：
  ```
  Qin PLUS DM-i detected: carName='byd', carFingerprint='<具体指纹>', using dedicated full-feature control file
  ```
- **只读告警**（车口不匹配才会触发，千万别上车控制）：
  ```
  Qin-only mode: car '<实际carName>' != 'byd', running read-only
  ```
  若出现这条，说明 `self.CP.carName` 不是 `byd`，秦专属门控把所有控制砍了。核对 `carFingerprint`，必要时把 `QIN_PLUS_DMI_CAR_NAME` 改成真实值。

> UI 表现对应：只读时方向盘/油门完全不接管（类似仅行车记录仪）。HUD 定速不出现或不可设。

---

## 1. 横向（扭矩参我弟数覆盖 + 右转修正）

秦车口下，`state_control` 每帧用车口文件 `CP.lateralTuning.torque` 的值覆盖实时学习（`is_qin_dmi` 分支）：

- 想临时确认参数是否按“文件值”生效：在 `controlsd.py` 秦分支（`if self.is_qin_dmi:`）里临时加一行 `cloudlog.info(f"qin torque: f={qin_lat_accel_factor} off={qin_lat_accel_offset} fr={qin_friction}")`。
- **右转压线修正**：`latcontrol_torque.update_live_torque_params` 内对 `latAccelOffset` 固定叠加 `+0.05 m/s²`。右转仍切内 → 回到 `latcontrol_torque.py` 把 `+0.05` 调到 `+0.06/+0.07`；左转偏外 → 降到 `+0.03/+0.02`。
- 若 UI 开着 `CustomTorqueLateral`：`latcontrol_torque.update_live_tune` 每 250 帧会用 UI 的 `TorqueMaxLatAccel / TorqueFriction` 覆盖一次（单帧闪切）。**建议测试期在 UI 关掉该开关**，让文件值全程生效。

**怎么看**：路测中过同一个右弯，对比改前/改后是否还压线；转向是否偏轻/偏重（latAccelFactor 影响增益）。

---

## 2. 弯道减速（Curve）

关键内部变量（在 `self._curve_*` / `self._scene_flags['curve']`）：

| 变量                       | 含义                 |
| ------------------------ | ------------------ |
| `_curve_active`          | 弯道减速进行中            |
| `_curve_severity`        | 严重程度 `0~1`（曲率越大越大） |
| `_curve_cooldown_timer`  | 出弯冷却倒计时 (s)        |
| `_v_cruise_before_curve` | 进弯前保存的定速，出弯恢复      |
| `_scene_flags['curve']`  | 本帧是否处于弯道场景         |

**生效判据**：

- 车速 `> CV_MIN_KPH(15km/h)`；模型前瞻 `CV_LA_MIN(20m) ~ CV_LA_MAX(150m)` 内有曲率 `>= CV_TH(0.003)`。
- 目标速度：`max(vCruise - severity*CV_DROP(35km/h), CV_TARGET(20km/h))`，按 `CV_DEC(1.5 m/s²)` 平滑降到安全速度。
- 出弯后按 `CV_REC(0.1km/h/帧)` 缓慢恢复。

**路测核对**：

- 进弯时 HUD 定速应明显掉（严重时最多 -35km/h），方向盘横向修正平稳；出弯自动回升。
- `experimentalMode` 在弯道为 True。
- 临时加日志：`cloudlog.info(f"CV act={self._curve_active} sev={self._curve_severity:.3f} v={self.v_cruise_helper.v_cruise_kph:.1f}")`。
- 若弯道完全不减速：先确认设备模型是否输出 `position.x` 与 `orientationRate.z`；再检查车速是否低于 15km/h、或曲率是否低于 `0.003`。

---

## 3. 静态前车识别（SS：停着的车 / 高速接近）

双源融合：雷达 `radarState.leadOne` + 视觉 `model_v2.leadsV3[0]`。  
关键内部变量：`_static_lead_active`、`_static_lead_dRel`、`_static_lead_vRel`、`_scene_flags['stopped_lead']`、`_scene_flags['high_speed_approach']`。

| 常量                        | 值   | 含义              |
| ------------------------- | --- | --------------- |
| `SS_RADAR_STOP_SPEED_KPH` | 1.0 | 雷达前车静止判定速度      |
| `SS_RADAR_MAX_DIST`       | 150 | 静止前车最大识别距离      |
| `SS_RADAR_REL_DIST`       | 3.0 | 雷达前车“贴近”判定距离    |
| `SS_HIGH_RELSPEED_KPH`    | 10  | 雷达高速接近相对车速阈值    |
| `SS_STOP_CONFIRM_TIME`    | 0.5 | 静止前车确认时间        |
| `SS_LOW_KPH`              | 3.0 | 极低速接近静止前车车速阈值   |
| `VL_PROB`                 | 0.6 | 视觉前车置信度阈值       |
| `VL_STOP_KPH`             | 0.5 | 视觉前车静止判定速度      |
| `HSA_REL_SPEED_KPH`       | 35  | 高速接近相对车速阈值（视觉源） |

**生效判据**：

- 雷达前车静止（`vLead < 1km/h` 且 `0<dRel<150m`）→ 确认有静态前车，触发实验模式、强制跟停。
- 雷达相对车速 `>10km/h` 或视觉相对车速 `>35km/h`（且距离 15~80m）→ 高速接近，触发实验模式跟车。
- 视觉前车 `prob>0.6` 且速度 `<0.5km/h`（`<30m`）→ 停着的车；或相对车速 `>30km/h` → 危险接近。

**路测核对**：

- 跟在一辆停着的车后面：车应稳稳跟停，不穿透、不突然加速；停稳后 `vCruise≈0` 或保持。
- 高速接近慢车/停车排队：提前减速，不临近才急刹。
- 临时加日志：`cloudlog.info(f"SS act={self._static_lead_active} d={self._static_lead_dRel:.1f} vr={self._static_lead_vRel:.1f} stop={self._scene_flags.get('stopped_lead')} hsa={self._scene_flags.get('high_speed_approach')}")`。

---

## 4. E2E 红绿灯停起步（TL）

原理：E2E 纵向模型内置停止线/信号灯分支，但只在实验模式（blended/e2e）下生效。`controlsd.py` 在实际车速 **5~70km/h** 窗口内强制 `experimental_mode = True` 并让 OP 纵向接管：模型看到红灯/停止线自行刹停，变绿灯后 E2E 轨迹恢复 → `cruiseControl.resume` 自动起步。**不依赖** `hybrid_modeld` 的 `trafficLightState` 字段。

| 常量                     | 值    | 含义                             |
| ---------------------- | ---- | ------------------------------ |
| `TL_CONTROL_ENABLE`    | True | E2E 红绿灯总开关（False 彻底关闭）         |
| `TL_LOW_SPEED_MIN_KPH` | 5.0  | 低于此车速不强制（停车/蠕行，避免误触发）          |
| `TL_LOW_SPEED_MAX_KPH` | 70.0 | 低于此车速强制实验模式 + OP 纵向接管（市区红绿灯窗口） |

**生效判据**：

- 前置条件：`TL_CONTROL_ENABLE=True` 且 `CP.openpilotLongitudinalControl=True`（启动日志 `[CAP]` 可确认）。
- 激活中（`self.enabled`）且实际车速进入 5~70km/h → `_scene_flags['traffic_light']=True`、`experimental_mode=True`、OP 纵向接管（带 0.3s/1.5s 双向防抖）。

**路测核对**：

- 市区 30~60km/h 无前车接近红灯：车应在停止线前平稳刹停，不需要人工干预。
- 停稳后变绿灯：E2E 轨迹恢复，自动起步跟随（若前车未走，由 SS/跟车逻辑兜底）。
- `[DBG]` 日志里 `TL act=True` 且 `exp=True long=True` 即链路正常。
- 完全不动作：先查 `[CAP]` 的 `openpilotLongitudinalControl` 是否为 True；再确认车速确实在 5~70km/h 窗口内（>70km/h 不强制，蠕行 <5km/h 也不强制）。

---

## 5. 设备定制 / 安全项

- **QIN_ONLY 只读**：非 `byd` 车口自动只读（panda `noOutput`）。刷到别的车也不会乱控——但这也意味着车口名必须正确（见第 0 节）。
- **VAG 时间炸弹绕过**：仅对大众系有意义，BYD 用不到；开关 `dp_vag_timebomb_bypass`，常量 `DP_VAG_TIMEBOMB_BYPASS_*`（帧）。
- **NO_IR**：无红外摄像头设备关 `dp_device_no_ir_ctrl`，关闭驾驶员监控（避免误判 distracted）。开着时 `driverMonitoringState` 不订阅、不进入 `forceDecel`。
- **温控**：`dp_device_disable_temp_check` 可关温度检查；默认 `thermalStatus>=red` 报 `overheat`、`freeSpacePercent<7%` 报 `outOfSpace`、`memoryUsagePercent>90%` 报 `lowMemory`。
- **df_tune 跟车距离**：由 `longitudinal_mpc_lib/long_mpc.py` 与 `legacy_longitudinal_mpc_lib/long_mpc.py` 的 `get_dynamic_follow` 控制（受 `dp_long_use_df_tune` 门控），不在 `controlsd.py` 内；改完这里要同步到两条 MPC 链路。

---

## 6. 快速定位清单（出问题先看这几条日志）

> 最省事：先把 `controlsd.py` 顶部的 `DEBUG_FEATURE_LOG = False` 改成 `True`，路测日志里 grep `[DBG]` 即可一次性看到弯道/SS/TL 三类信号（见第 7 节）。

1. **完全不接管 / 只读** → 搜 `Qin-only mode`（车口名错）或 `running read-only`。
2. **弯道不减速度** → 看 `[DBG]` 里的 `CV act=`：是否一直 `act=False`？查车速是否 <15km/h、模型是否输出 `position.x`/`orientationRate.z`。
3. **静态前车穿透 / 不跟停** → 看 `[DBG]` 里的 `SS act=`：雷达 `leadOne.status` 是否为 False（无雷达或雷达未识别）。
4. **红灯不刹车 / 不起步** → 看 `[DBG]` 里的 `TL act=` 与 `exp=`/`long=`：`TL act=False` 说明不在 5~70km/h 窗口或总开关关闭；`exp=False` 说明实验模式没强制上；`long=False` 说明 OP 纵向没接管（查 `[CAP]` 能力日志）。
5. **横向偏轻/偏重、右转压线** → 见第 1 节调 `latAccelFactor` / `+0.05` 偏移。
6. **参数被 UI 闪切** → 关掉 UI `CustomTorqueLateral`。

---

## 7. 调试日志怎么加（统一做法）

### 方式一（推荐）：内置 `DEBUG_FEATURE_LOG` 开关

`controlsd.py` 顶部有模块级常量 `DEBUG_FEATURE_LOG = False`。改成 `True` 后，`state_control` 每 ~0.5s（每 50 帧）自动打印一行，覆盖四大功能的现场信号，**无需改任何功能代码**：

```
[DBG] CV act=弯道中 sev=曲率严重程度 v=当前定速 | SS act=有静态前车 d=距离 vr=相对速度 | TL act=红绿灯窗口 exp=实验模式 long=OP纵向
```

- `CV sev`：弯道曲率严重程度 `0~1`（越大弯越急）
- `SS act/d/vr`：雷达+视觉静态前车识别（是否确认、距离、相对速度）
- `TL act`：E2E 红绿灯窗口是否激活（5~70km/h 强制实验模式）

路测时用 `journalctl -u controlsd -f` 或抓 `rlog` 回放 `grep "[DBG]"`。验证完把开关改回 `False` 避免刷屏。

### 方式二（定向深挖）：手动加 `cloudlog.info(...)`

需要更细的现场值时，在对应功能分支里加一行 `cloudlog.info(...)`（例如弯道分支加 `cloudlog.info(f"CV act={self._curve_active} sev={self._curve_severity:.3f}")`）。验证完记得删掉，避免刷屏。
