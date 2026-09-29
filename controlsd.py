# [MOD 2026-09-18 23:29]
# 作者：温存°sunshine
# 秦PLUS DM-i 全功能控制
# 已实现：
#   1) 前方静态车辆识别（雷达 leadOne + 视觉 leadsV3 双源融合）
#   2) 弯道减速（模型曲率前瞻预判）
#   3) 高速跟车 / 前车起步跟随 / 积极加速
#   4) E2E 红绿灯停起步（5~70km/h 强制实验模式，模型停止线/信号灯分支）
# =======================
# 多源感知（视觉 / 雷达 / 模型）
# =======================

import os, math, time
import cereal.messaging as messaging
from typing import SupportsFloat
from cereal import car, log
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL, DT_MDL
from openpilot.common.profiler import Profiler
from openpilot.common.params import Params, put_nonblocking, put_bool_nonblocking
from cereal.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.conversions import Conversions as CV
from panda import ALTERNATIVE_EXPERIENCE
from openpilot.system.swaglog import cloudlog
from openpilot.system.version import get_short_branch
from openpilot.system.hardware import TICI
from openpilot.selfdrive.boardd.boardd import can_list_to_can_capnp
from openpilot.selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from openpilot.selfdrive.controls.lib.lateral_planner import CAMERA_OFFSET
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import VCruiseHelper, get_lag_adjusted_curvature, CONTROL_N, V_CRUISE_INITIAL, V_CRUISE_INITIAL_EXPERIMENTAL_MODE
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.longcontrol_tuner import LongControlTuner
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from openpilot.selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.events import Events, ET
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.hybrid_modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.dynamic_endtoend_controller import DynamicEndtoEndController


# ==============================================================================
# 可调参数集中配置区（所有可调数值统一放在此处，方便调试）
# ==============================================================================

# ===== 基础控制 =====
SOFT_T = 3               # 软禁用超时时间（秒）
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS    # 车道偏离预警最低触发速度（m/s）
LDW_TH = 0.1        # 车道偏离概率阈值（0~1）
STR_T = 0.5           # 方向盘放松确认时间（秒）
LC_TOL = 0.8           # 车道居中容差（米），超出则判定偏离中心

# ===== ACC/OP切换速度阈值（km/h） =====
ACC_ON_KPH = 1.5      # 切回原车ACC的速度阈值（低于此速度用ACC）
ACC_OFF_KPH = 8.5     # 切到OP纵向的速度阈值（低于此速度用OP）

# ===== 雷达/视觉静态前车识别（SS）=====
SS_RADAR_STOP_SPEED_KPH = 1.0     # 雷达前车静止判定速度 (km/h)
SS_RADAR_MAX_DIST = 150.0         # 雷达静止前车最大识别距离 (m)
SS_RADAR_REL_DIST = 3.0           # 雷达前车贴近判定距离 (m)
SS_HIGH_RELSPEED_KPH = 10.0       # 高速接近相对车速阈值 (km/h)
SS_STOP_CONFIRM_TIME = 0.5        # 静止前车确认时间 (s)
SS_LOW_KPH = 3.0                  # 极低速接近静止前车车速阈值 (km/h)

# ===== 起步/跟车辅助常量 =====
LEAD_START_MOVE_SPEED = 0.1       # 前车视为"已移动"的最低速度 (m/s)
LEAD_START_ACCEL_TIMER = 2.0      # 起步积极加速持续时长 (s)
# [CRASH-FIX 2026-09-27] 跟车距离默认档位：aggressive(最近档)。
# 此前 _cycle_long_personality/__init__ 引用未定义的 DEFAULT_LONG_PERSONALITY，
# 按距离按钮即 NameError -> controlsd 崩溃红屏"Controls Unresponsive"。
DEFAULT_LONG_PERSONALITY = int(log.LongitudinalPersonality.aggressive)

# ===== 弯道检测与预判减速 =====
CV_MIN_KPH = 15.0             # 检测最低速度（km/h）
CV_LA_MIN = 20.0          # 预瞄起始距离（米）
CV_LA_MAX = 150.0         # 预瞄最远距离（米）
CV_TH = 0.003                # 触发曲率阈值
CV_CD = 2.0                 # 退出冷却时间（秒）
CV_SEV_MAX = 1.0               # 最大严重程度（0~1）
CV_SEV_DIV = 0.007         # 严重程度归一化除数
CV_DROP = 35.0            # 最大降速幅度（km/h）
CV_TARGET = 20.0            # 弯道最低目标速度（km/h）
CV_DEC = 1.5                  # 减速度（m/s²）
CV_STEP = 8.0                # 单帧最大减速（m/s）
# 弯道退出后的自适应回速（2026-09-26 优化）：
# 基础 0.2 km/h/帧(=20km/h/s)起步，每多 1km/h gap 多 3% 步长，上限 0.5 km/h/帧(=50km/h/s)。
# 比旧固定 0.1 km/h/帧 (=10km/h/s) 恢复快 2~5 倍：60km/h gap 从 6s 缩到约 1.2s 回到 v_cruise_before_curve。
CV_REC_MIN  = 0.2           # 基础恢复步长 (km/h/帧)
CV_REC_GAIN = 0.03          # 恢复步长随 gap 增长的系数 (km/h 每 km/h gap)
CV_REC_MAX  = 0.5           # 恢复步长上限 (km/h/帧)，保护油门平滑

# ===== 视觉前车检测 =====
VL_PROB = 0.6          # 视觉前车置信度阈值（0~1），越高越严格
VL_STOP_KPH = 0.5       # 前车静止判定速度（km/h）
VL_STOP_DIST_M = 30.0   # 视觉静止前车最大判定距离（m），30m 外依赖雷达/MPC
VL_STOP_CONFIRM_T = 0.15  # 视觉静止确认时间(s)：连续满足才点亮，防模型单帧噪声误触发
VL_DANGER_KPH = 30.0  # 危险接近/相对速度阈值（km/h），超过触发跟车+实验模式
# 视觉“疑似静止前车”软兜底：置信度在 (VL_PROB_SUSPECT, VL_PROB] 的近距离慢/停物体，
# 模型不确定但仍极可能是静止前车 -> 也按静态前车刹车（覆盖 prob 0.35~0.6 的边缘漏检）。
# 距离/速度都收得很紧，避免模型噪声造成的幽灵刹。
VL_PROB_SUSPECT = 0.35   # 疑似静止前车最低置信度
VL_SUSPECT_DIST_M = 20.0  # 仅生效于近距离 (m)
VL_SUSPECT_STOP_KPH = 2.0  # 速度低于此 (km/h) 视为停着

# ===== 雷达侧疑似静态前车软兜底（视觉 VL_PROB_SUSPECT 的对称补盲，2026-09-26 加） =====
# 视觉 leadsV3 低置信度(prob<0.35) 或完全无识别时，夜间跟尾灯丢失、暴雨、强逆光等
# 场景会出现"前车就在那儿但视觉不报"的盲区。本段复用雷达 leadOne 信号软点亮疑似静态前车：
# vLead≈0 + 距离近 + 连续满足 RS_RADAR_SUSPECT_CONFIRM_T 秒 -> 软点亮 static lead 档刹车。
# 比下方 SS 块更宽松（vLead 阈值 2 vs 1, 距离收紧到 20m, 加 0.3s 防抖窗口防雷达单帧噪声幽灵刹）。
RS_RADAR_SUSPECT_V_KPH = 2.0      # 雷达 vLead 阈值 (km/h)：低于此视为疑似静止
RS_RADAR_SUSPECT_DIST_M = 20.0    # 仅近距离生效 (m)：超过不软点亮，避免远处误触发
RS_RADAR_SUSPECT_CONFIRM_T = 0.30 # 连续满足时间 (s)：防雷达单帧噪声幽灵刹

# ===== 高速接近前车检测 =====
HSA_EGO_MIN_KPH = 50.0       # 自车最小速度阈值（km/h）
HSA_REL_SPEED_KPH = 35.0     # 相对速度阈值（km/h），超过此值认为高速接近
HSA_DIST_MAX_M = 80.0        # 最大检测距离（米）
HSA_DIST_MIN_M = 15.0        # 最小检测距离（米）
HSA_CLOSE_DIST_M = 25.0      # 近距离高速接近阈值(m)：HSA 档(-1.0)在此距离内兜不住慢车，升 stopped_lead 档强刹
HSA_CLOSE_VLEAD_KPH = 30.0   # 视觉侧近距离升档的前车速度上限(km/h)：前车明显慢于自车才升档，防正常跟车误触发
HSA_CLOSE_VLEAD_KPH_HI = 50.0   # 升档宽上限(km/h)：rel>35 高速接近时前车速度 < 此值也升档，补 30~50km/h 中速前车盲区

# ===== 跟车状态防抖 =====
LEAD_DB_T = 0.5           # 前车状态切换防抖时间（秒）

# ===== ACC/OP 纵向接管防抖（防夜间视觉抖动导致跟车频繁点刹） =====
# 原车 ACC 与 OP 纵向来回切换时，每次切换都会重置纵向 PID，体感就是"点刹"。
# 改为双向防抖：场景需持续 ACC_TAKEOVER_IN_T 才从原车 ACC 切 OP 纵向，
# 场景消失需持续 ACC_TAKEOVER_OUT_T 才交回原车 ACC。
ACC_TAKEOVER_IN_T = 0.3   # 切入 OP 纵向前，前车/弯道场景需持续的时间 (s)
ACC_TAKEOVER_OUT_T = 1.5  # 交回原车 ACC 前，场景需消失的时间 (s)

# ===== E2E 红绿灯识别（低速强制实验模式窗口，按实际车速判定） =====
# 原理：E2E 纵向模型内置停止线/信号灯分支，但仅在实验模式(blended/e2e)下生效。
# 市区 5~70km/h 窗口内强制实验模式并让 OP 纵向接管：模型看到红灯/停止线即自行刹停，
# 变绿灯后 E2E 轨迹恢复 -> cruiseControl.resume 自动起步。
# 不依赖 hybrid_modeld 的 trafficLightState 字段（多数 fork 无此字段，会静默降级）；
# 写法对齐 controlsd.py11 已验证可用的机制。
TL_CONTROL_ENABLE = False       # 精调0923晚: 用户反馈斑马线误停车+前车不减速，彻底关闭E2E红绿灯
TL_LOW_SPEED_MIN_KPH = 5.0      # 低于此车速不强制（停车/蠕行，避免误触发）
TL_LOW_SPEED_MAX_KPH = 70.0     # 低于此车速强制实验模式 + OP 纵向接管（秦PLUS市区红绿灯窗口）

# ===== 警报开关 =====
STEER_SATURATED_ALERT_MODE = 2  # 1=开 | 2=关（秦PLUS大弯易误报"请接管/转弯超出转向极限"，默认关闭）

# ===== 加速度限制（场景组合，单位m/s²） =====
AC_SL_MAX = 0.0              # 静止前车：最大加速度
AC_SL_MIN = -4.5             # 静止前车：最小加速度（最大减速度），比 -3.0 更跟手、停车距离更短
AC_HSA_MAX = 0.0       # 高速接近：最大加速度
AC_HSA_MIN = -3.0      # 高速接近：最小加速度（处理1：-1.0 太弱，rel>35 接近 30~50km/h 中速前车刹不住，提至 -3.0）
AC_CV_MAX_B = 0.3                # 弯道：最大加速度基础值
AC_CV_MSF = 1.7     # 弯道：最大加速度严重程度衰减因子
AC_CV_MIN_B = -1.5               # 弯道：最小加速度基础值
AC_CV_MS2 = 0.5     # 弯道：最小加速度严重程度增强因子
AC_NORM_MAX = 0.5                    # 正常行驶：最大加速度
AC_NORM_MIN = -4.0                   # 正常行驶：最小加速度（方案B：-3.0→-4.0，正常跟车急刹能力增强）

# ===== 纵向加速度变化率限幅（消除"场景跳变导致的突然重刹"）=====
# 高速接近时 HSA 档 a_min=-1.0(轻刹)，慢慢靠近后 HSA 条件掉出 -> 退回 normal 档 a_min=-3.0，
# 一帧内从 -1.0 蹦到 -3.0，体感即"突然重刹"。加变化率限幅让 accel 平滑过渡：仅限制变化速率，
# 不抬高安全下限——MPC 仍需 -3.0 时约 1~1.5s 内到达；停前车档 a_min=-4.5、紧急安全网 a_min=-7.0，
# 真实紧急刹车走 stockAeb 安全路径不受影响。
ACCEL_SLEW_RATE = 3.5                # 纵向加速度最大变化率 (m/s^2 per s)，越大越跟手、越小越顺滑（方案C：2.0→3.5）

# ===== 紧急静态前车安全网（最后兜底，临界即强刹） =====
# 当已确认“静态前车”且距离/碰撞时间(TTC)进入临界区，强制最大减速度并绕过变化率限幅，
# 立即重刹——即使 MPC 偏保守或识别偏晚也能把车停住。这是冗余兜底，不替代正常 SS 跟停。
EMERGENCY_STATIC_BRAKE = True    # 总开关；出现幽灵刹(护栏)时关 False
EMERGENCY_TTC_S = 1.6            # 碰撞时间阈值(s)：TTC<此值判定临界
EMERGENCY_DIST_NEAR_M = 15.0     # 近距离无条件强刹距离(m)：15m 内静态前车直接 -7.0（原 45m 过宽，正常跟停会过早重刹）
AC_EMERGENCY_MIN = -7.0          # 紧急最小加速度(最大减速度, m/s^2)，接近轮胎/ABS 极限(~-8)

# ===== 前车急刹联动制动（方案A：雷达前车急刹 -> 立即重刹） =====
# 原安全网仅认静态前车(_static_lead_active，几乎停住)，急刹中的前车不会触发，只能等它减速到
# 接近静止才兜底，往往为时已晚。此处直接读雷达 lead.aLeadK：前车正在以超过阈值的减速度急刹、
# 且距离 / TTC 进入临界时，直接走紧急档 -7.0 并跳过变化率限幅立即重刹。
LEAD_EBRAKE_ENABLE = True        # 总开关；出现误触发(前车正常刹车但被判定急刹)时关 False
LEAD_EBRAKE_ACCEL_TH = -3.0      # 前车加速度阈值(m/s^2)：aLeadK<此值视为“前车正在急刹”
LEAD_EBRAKE_DIST_MAX_M = 60.0    # 触发最大距离(m)：仅在此距离内联动，防止远处误触发
LEAD_EBRAKE_TTC_S = 3.0          # 碰撞时间阈值(s)：TTC<此值且前车急刹才联动，低速跟停不误伤

# ===== LDW车道偏离 =====
LDW_LANE_OFFSET = 1.10                     # 车道偏离判定偏移量（米），越大越不敏感

# ===== 秦 PLUS DM-i 专属配置（在“全功能”基础上叠加，确保仅本车口生效） =====
# BYD 车口 carName 为 "byd"(radard 按 selfdrive.car.<carName> 导入车口目录,
# legacy_lateral_planner 的 STEER_RATE_COST 亦以 "byd" 为键)。
QIN_PLUS_DMI_CAR_NAME = "byd"
QIN_ONLY = True                            # True:非本车口车型时强制只读(只记录不控制),实现"专用"
# 秦 PLUS DM-i 专属扭矩参数:None = 直接采用车口控制文件 CP.lateralTuning.torque 的值
# (每帧覆盖实时学习,避免学习值漂移);填具体数值则以该数值强制覆盖。
QIN_TORQUE_LAT_ACCEL_FACTOR = None
QIN_TORQUE_LAT_ACCEL_OFFSET = None   # 右转压线的 +0.05 latAccelOffset 修正在 latcontrol_torque 内叠加
QIN_TORQUE_FRICTION_COEFF = None

# ===== 设备级定制（从原控制栈保留） =====
# NO_IR:无红外摄像头设备关闭驾驶员监控
NO_IR_CTRL = Params().get_bool("dp_device_no_ir_ctrl")
# VAG(大众集团)车型转向"时间炸弹"绕过:计数阈值(帧)
DP_VAG_TIMEBOMB_BYPASS_WARNING = 34000  # 开始预警
DP_VAG_TIMEBOMB_BYPASS_START = 345000   # 开始关闭转向/油门
DP_VAG_TIMEBOMB_BYPASS_END = 348000     # 计数归零复位


REPLAY = "REPLAY" in os.environ
DEBUG_FEATURE_LOG = False  # 设为 True 后,每 ~0.5s 在日志打印弯道/静态前车关键状态(路测排查用);验证完改回 False
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"loggerd", "encoderd", "statsd", "mapd", "gpxd"}
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
ThermalStatus = log.DeviceState.ThermalStatus
Desire = log.LateralPlan.Desire
LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)
CONTROL_N_T_IDX=ModelConstants.T_IDXS[:CONTROL_N]

def get_accel_from_plan(CP, speeds, accels):
    if len(speeds) == CONTROL_N and len(accels) == CONTROL_N:
        v_target_now = interp(DT_MDL, CONTROL_N_T_IDX, speeds)
        a_target_now = interp(DT_MDL, CONTROL_N_T_IDX, accels)
        delay = (CP.longitudinalActuatorDelayLowerBound + CP.longitudinalActuatorDelayUpperBound) * 0.5
        v_target = interp(delay + DT_MDL, CONTROL_N_T_IDX, speeds)
        a_target = 2 * (v_target - v_target_now) / delay - a_target_now
    else:
        v_target = 0.0
        v_target_now = 0.0
        a_target = 0.0
    return a_target
class Controls:
    # 原车四格距离按钮 -> LongitudinalPersonality 循环切换常量（提升为类属性，避免每次调用重建）
    _PERSONALITY_ORDER = [log.LongitudinalPersonality.aggressive,
                          log.LongitudinalPersonality.standard,
                          log.LongitudinalPersonality.relaxed]
    _PERSONALITY_ORDER_INT = [int(m) for m in _PERSONALITY_ORDER]
    _PERSONALITY_LABELS = {log.LongitudinalPersonality.aggressive: "近 (aggressive)",
                           log.LongitudinalPersonality.standard: "中 (standard)",
                           log.LongitudinalPersonality.relaxed: "远 (relaxed)"}

    def _init_state_attrs(self):
        """统一初始化所有控制逻辑所需的状态属性（避免动态初始化和 hasattr 检查）"""

        # ========== 自适应跟车加速度控制相关 ==========
        self._lead_status = False
        self._lead_status_timer = 0
        # ========== 前车状态跟踪相关 ==========
        self._stopped_lead_detected = False
        # ========== 雷达/视觉静态前车识别（SS） ==========
        self._static_lead_active = False           # 当前是否确认有"静态前车"
        self._static_lead_dRel = 0.0               # 静态前车最近距离
        self._static_lead_vRel = 0.0               # 静态前车相对速度
        self._emergency_static = False            # 紧急静态前车安全网触发标志
        self._vision_stop_confirm = 0.0           # 视觉静止确认计时（连续满足才点亮，防单帧噪声）
        # ========== 雷达疑似静态前车软兜底相关（2026-09-26 加） ==========
        self._radar_suspect_confirm = 0.0         # 雷达疑似静态前车：连续确认计时 (s)，达到阈值才软点亮
        self._radar_suspect_active = False        # 本帧是否已通过雷达软兜底点亮 static_lead（防下方 SS 块 elif/else 分支误清）
        # ========== 车道变换辅助相关 ==========
        self._dp_lat_lane_change_assist_disabled_active = False
        # ========== ACC/OP切换状态相关 ==========
        self._last_use_stock_acc = False
        self._op_long_takeover = False            # OP 纵向是否已从原车 ACC 接管（带双向防抖）
        self._op_long_takeover_timer = 0.0        # 接管切换持续计时
        self._last_cmd_accel = 0.0                # 上帧实际下发加速度（用于变化率限幅）
        # ========== 场景检测标志位 ==========
        self._scene_flags = {
            'traffic_light': False,        # 红绿灯场景（E2E 低速强制实验模式窗口）
            'curve': False,                # 弯道场景
            'stopped_lead': False,         # 静止前车场景
            'high_speed_approach': False,  # 高速接近场景
            'emergency_static': False,     # 紧急静态前车安全网场景
        }
        self._tl_engaged = False         # E2E 红绿灯锁存：窗口内进入后保持 blended 到停稳/绿灯起步
        self._curve_severity = 0.0
        self._v_cruise_before_curve = None
        self._curve_active = False
        self._curve_cooldown_timer = 0.0
        # ========== 原车方向盘四格跟车距离按钮 -> LongitudinalPersonality 切换 ==========
        self._stock_dist_btn_types = set()   # 端口转发的"距离按钮"ButtonType 候选集合（__init__ 中填充）
        self._dist_btn_seen = set()          # 发现模式：累计实际出现的按钮类型名
        self._dist_btn_last_log = 0          # 发现模式日志节流计时(frame)
        self._dist_btn_last_cycle = 0        # 切换防抖：上次循环切换的 frame
        self._dist_btn_toast_until = 0       # HUD 提示持续到的 frame
        self._dist_btn_toast_text = ""       # HUD 提示文本

    def _cycle_long_personality(self):
        """原车四格距离按钮按下：循环切换 LongitudinalPersonality（aggressive->standard->relaxed）。"""
        raw = self.params.get("LongitudinalPersonality")
        try:
            cur = int(raw.decode()) if raw is not None else DEFAULT_LONG_PERSONALITY
        except (ValueError, AttributeError):
            cur = DEFAULT_LONG_PERSONALITY
        try:
            idx = self._PERSONALITY_ORDER_INT.index(cur)
        except ValueError:
            idx = 0
        nxt = self._PERSONALITY_ORDER[(idx + 1) % len(self._PERSONALITY_ORDER)]
        self.params.put("LongitudinalPersonality", str(int(nxt)).encode())
        self._dist_btn_toast_text = self._PERSONALITY_LABELS.get(nxt, str(int(nxt)))
        self._dist_btn_toast_until = self.sm.frame + int(2.0 / DT_CTRL)  # 约 2s HUD 提示
        self._dist_btn_last_cycle = self.sm.frame
        cloudlog.info(f"[DIST_BTN] LongitudinalPersonality -> {self._dist_btn_toast_text}")

    def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
        config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)
        self.dp_gps_ok_once = False
        self.branch = get_short_branch("")
        self.pm = pm
        if self.pm is None:
            self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState','carControl', 'carEvents', 'carParams', 'controlsStateExt'])
        can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 20
        self.can_sock = messaging.sub_sock('can', timeout=can_timeout)
        self.log_sock = messaging.sub_sock('androidLog')
        self.params = Params()
        # 跟车距离调近：LongitudinalPersonality 未设置时默认写入 aggressive（最近档）。
        # 用户已显式设置过（含原车按钮循环切换的档位）则保持原值，尊重用户选择。
        try:
            if self.params.get("LongitudinalPersonality") is None:
                self.params.put("LongitudinalPersonality", str(int(DEFAULT_LONG_PERSONALITY)).encode())
                cloudlog.info(f"[DIST] LongitudinalPersonality 未设置, 默认写入 {DEFAULT_LONG_PERSONALITY}")
        except Exception:
            pass  # UnknownKeyName 等异常忽略，回退到 _cycle_long_personality 默认值
        self.dp_no_gps_ctrl = self.params.get_bool("dp_no_gps_ctrl")
        self.dp_no_fan_ctrl = self.params.get_bool("dp_no_fan_ctrl")
        self.dp_0813 = self.params.get_bool("dp_0813")
        # 原车方向盘四格跟车距离按钮 -> 切换 LongitudinalPersonality（默认开启；参数未注册时按 True 处理）
        # 注意：设备端 Params 为严格白名单（params_pyx.pyx check_key），未注册 key 的 get() 会抛
        # UnknownKeyName 导致 controlsd 启动即崩，必须捕获后按“未设置”处理。
        try:
            _dist_btn_raw = self.params.get("dp_long_use_stock_acc_distance_button")
        except Exception:
            _dist_btn_raw = None  # UnknownKeyName：设备未注册该 key → 视为未设置（默认开启）
        self.dp_long_use_stock_acc_distance_button = _dist_btn_raw is None or _dist_btn_raw == b"1"
        # 候选原车距离按钮类型名（覆盖各 fork 端口可能的命名）。端口未转发已知类型则进入发现模式。
        self._stock_dist_btn_types = set()
        for _n in ("gapAdjust", "distance", "accDistance", "gapButton", "stockAccDistance",
        "distButton", "accelCruiseAlt", "decelCruiseAlt", "followDistance"):
            _bt = getattr(ButtonType, _n, None)
            if _bt is not None:
                self._stock_dist_btn_types.add(_bt)
        if self.dp_long_use_stock_acc_distance_button:
            cloudlog.info(f"[DIST_BTN] enabled; candidate button types={[t.name for t in self._stock_dist_btn_types] or 'NONE (discovery mode)'}")
        self._dp_alka = self.params.get_bool("dp_alka")
        self._dp_alka_active = True
        self._dp_alka_trigger_count = 0
        self._dp_alka_btn_block_frame = 0
        self._dp_lat_lane_change_assist_disabled = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) == 0
        self._dp_lat_lane_change_assist_disabled_active = False
        self.torqued_override = self.params.get_bool("CustomTorqueLateral")
        # 设备级定制属性
        self.dp_device_disable_temp_check = self.params.get_bool("dp_device_disable_temp_check")  # 关闭温度检查
        self._dp_vag_timebomb_bypass_counter = 0                             # VAG 时间炸弹绕过计数
        self._dp_vag_timebomb_bypass = self.params.get_bool("dp_vag_timebomb_bypass")            # VAG 绕过开关
        # 摄像头数据包列表:关 IR 时只看路摄像头,否则同时看驾驶员摄像头
        if NO_IR_CTRL:
            self.camera_packets = ["roadCameraState"]
        else:
            self.camera_packets = ["roadCameraState", "driverCameraState"]
        self.sm = sm
        if self.sm is None:
            ignore = ['testJoystick']
            if SIMULATION:
                ignore += ['driverCameraState', 'managerState']
            if NO_IR_CTRL:
                ignore += ['driverCameraState', 'driverMonitoringState']
            self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'radarState', 'liveCalibration', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman', 'managerState', 'liveParameters', 'liveTorqueParameters', 'driverMonitoringState', 'testJoystick'] + self.camera_packets, ignore_alive=ignore, ignore_avg_freq=['testJoystick'])
        if CI is None:
            get_one_can(self.can_sock)

            num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
            experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")# and not self.dp_0813 # and not is_release_branch()
            self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
        else:
            self.CI, self.CP = CI, CI.CP

            # 一次性能力诊断:确认 OP 纵向 / 实验性纵向能力
        cloudlog.info(f"[CAP] openpilotLongitudinalControl={self.CP.openpilotLongitudinalControl} "
                  f"experimentalLongitudinalAvailable={self.CP.experimentalLongitudinalAvailable} "
                  f"ExperimentalLongitudinalEnabled={self.params.get_bool('ExperimentalLongitudinalEnabled')} "
                  f"ExperimentalMode={self.params.get_bool('ExperimentalMode')} "
        f"dp_0813={self.dp_0813} radarUnavailable={self.CP.radarUnavailable}")

        # 秦 PLUS DM-i 专用判断:后续分支据此应用专属默认值/只读限制
        self.is_qin_dmi = (self.CP.carName == QIN_PLUS_DMI_CAR_NAME)
        # 秦 PLUS DM-i:强制使用专属扭矩参数(覆盖实时学习/UI 自定义)
        if self.is_qin_dmi:
            self.torqued_override = True
            cloudlog.info(f"Qin PLUS DM-i detected: carName='{self.CP.carName}', carFingerprint='{self.CP.carFingerprint}', using dedicated full-feature control file")

        self.joystick_mode = self.params.get_bool("JoystickDebugMode") or self.CP.notCar

        self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
        self.CP.alternativeExperience = 0
        if not self.disengage_on_accelerator:
            self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS

        if self._dp_alka:
            self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALKA

        self.is_metric = self.params.get_bool("IsMetric")
        self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
        openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
        passive = self.params.get_bool("Passive") or not openpilot_enabled_toggle

        car_recognized = self.CP.carName != 'mock'

        controller_available = self.CI.CC is not None and not passive and not self.CP.dashcamOnly
        self.read_only = not car_recognized or not controller_available or self.CP.dashcamOnly
        # 秦 PLUS DM-i 专用(精简):非本车口进入只读模式,不输出任何控制指令
        if QIN_ONLY and not self.is_qin_dmi:
            self.read_only = True
            cloudlog.warning(f"Qin-only mode: car '{self.CP.carName}' != '{QIN_PLUS_DMI_CAR_NAME}', running read-only")
        if self.read_only:
            safety_config = car.CarParams.SafetyConfig.new_message()
            safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
            self.CP.safetyConfigs = [safety_config]

        prev_cp = self.params.get("CarParamsPersistent")
        if prev_cp is not None:
            self.params.put("CarParamsPrevRoute", prev_cp)

        cp_bytes = self.CP.to_bytes()
        self.params.put("CarParams", cp_bytes)
        put_nonblocking("CarParamsCache", cp_bytes)
        put_nonblocking("CarParamsPersistent", cp_bytes)

        if not self.CP.experimentalLongitudinalAvailable:# or is_release_branch():
            self.params.remove("ExperimentalLongitudinalEnabled")
        if not self.CP.openpilotLongitudinalControl:
            self.params.remove("ExperimentalMode")

        self.CC = car.CarControl.new_message()
        self.CS_prev = car.CarState.new_message()
        self.AM = AlertManager()
        self.events = Events()

        if self.CP.useLongitudinalTuner:
            self.LoC = LongControlTuner(self.CP)
        else:
            self.LoC = LongControl(self.CP)
        self.VM = VehicleModel(self.CP)
        self.LaC: LatControl
        if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            self.LaC = LatControlAngle(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'pid':
            self.LaC = LatControlPID(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'indi':
            self.LaC = LatControlINDI(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'lqr':
            self.LaC = LatControlLQR(self.CP, self.CI)
        elif self.CP.lateralTuning.which() == 'torque':
            self.LaC = LatControlTorque(self.CP, self.CI)
        self.DH = DesireHelper()
        self.dual_e2e_controller = DynamicEndtoEndController()
        self.dual_e2e_controller.set_enabled(True)

        self.initialized = False
        self.state = State.disabled
        self.enabled = False
        self.active = False
        self.soft_disable_timer = 0
        self.mismatch_counter = 0
        self.cruise_mismatch_counter = 0
        self.can_rcv_timeout_counter = 0     
        self.can_rcv_cum_timeout_counter = 0  
        self.last_blinker_frame = 0
        self.last_steering_pressed_frame = 0
        self.distance_traveled = 0
        self.last_functional_fan_frame = 0
        self.events_prev = []
        self.current_alert_types = [ET.PERMANENT]
        self.logged_comm_issue = None
        self.not_running_prev = None
        self.last_actuators = car.CarControl.Actuators.new_message()
        self.steer_limited = False
        self.desired_curvature = 0.0
        self.desired_curvature_rate = 0.0
        self.experimental_mode = False
        self.v_cruise_helper = VCruiseHelper(self.CP)
        self.recalibrating_seen = False
        self.sm['liveParameters'].valid = True
        self.can_log_mono_time = 0
        self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

        # 统一初始化所有状态属性（遵循统一初始化原则）
        self._init_state_attrs()

        if not car_recognized:
            self.events.add(EventName.carUnrecognized, static=True)
            if len(self.CP.carFw) > 0:
                set_offroad_alert("Offroad_CarUnrecognized", True)
            else:
                set_offroad_alert("Offroad_NoFirmware", True)
        elif self.read_only:
            self.events.add(EventName.dashcamMode, static=True)
        elif self.joystick_mode:
            self.events.add(EventName.joystickDebug, static=True)
            self.startup_event = None
            #帧处理100hz
        self.rk = Ratekeeper(100, print_delay_threshold=None)
        self.prof = Profiler(False)
    

    def set_initial_state(self):
        if REPLAY:
            controls_state = Params().get("ReplayControlsState")
            if controls_state is not None:
                controls_state = log.ControlsState.from_bytes(controls_state)
                self.v_cruise_helper.v_cruise_kph = controls_state.vCruise
            if any(ps.controlsAllowed for ps in self.sm['pandaStates']):
                self.state = State.enabled

    def _set_vision_static_lead(self, dist_m, v_lead_kph, v_ego_kph):
        """统一点亮 stopped_lead 与 static_lead 相关状态（视觉/雷达软兜底共用）。"""
        # 视觉确认的静止前车同样点亮 _static_lead_active，让紧急安全网也能覆盖纯视觉场景
        self._stopped_lead_detected = True
        self._scene_flags['stopped_lead'] = True
        self._static_lead_active = True
        self._static_lead_dRel = dist_m
        self._static_lead_vRel = (v_lead_kph - v_ego_kph) * CV.KPH_TO_MS

    def _detect_curve_ahead(self, model_v2, v_ego):
        """检测前方弯道，返回 (detected, severity, distance)。"""
        if not hasattr(model_v2, 'position') or v_ego < 0.1:
            return False, 0.0, float('inf')
        max_curv = 0.0
        min_dist = float('inf')
        for i, x in enumerate(model_v2.position.x):
            if CV_LA_MIN <= x <= CV_LA_MAX and i < len(model_v2.orientationRate.z):
                curv = abs(model_v2.orientationRate.z[i] / v_ego)
                if curv >= CV_TH:
                    max_curv = max(max_curv, curv)
                    min_dist = min(min_dist, x)
        if max_curv >= CV_TH:
            severity = min((max_curv - CV_TH) / CV_SEV_DIV, CV_SEV_MAX)
            return True, severity, min_dist
        return False, 0.0, float('inf')

    def update_events(self, CS):
        self.events.clear()
        if self.startup_event is not None:
            self.events.add(self.startup_event)
            self.startup_event = None
        if not self.initialized:
            self.events.add(EventName.controlsInitializing)
            return
        if self.read_only:
            return
        # ========== 设备温度/磁盘/内存 相关事件 =========
        if not self.dp_device_disable_temp_check and self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
            self.events.add(EventName.overheat)  # 过热
        if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
            self.events.add(EventName.outOfSpace)  # 存储空间 <7%
        if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
            self.events.add(EventName.lowMemory)  # 内存 >90%
        # ========== ALKA 功能逻辑 ==========
        if self._dp_alka and CS.brakePressed:
            if self.CP.pcmCruise and CS.cruiseState.available != self.CS_prev.cruiseState.available:
                self._dp_alka_trigger_count += 1
            if self._dp_alka_trigger_count == 2:
                self._dp_alka_active = not self._dp_alka_active
            if self.sm.frame % 50 == 0:
                self._dp_alka_trigger_count = 0
            if not self.CP.pcmCruise and self._dp_alka_btn_block_frame < self.sm.frame:
                if any(be.type in (ButtonType.decelCruise, ButtonType.setCruise) for be in CS.buttonEvents):
                    self._dp_alka_active = not self._dp_alka_active
                    self._dp_alka_btn_block_frame = self.sm.frame + 100

                    # ========== 原车方向盘四格跟车距离按钮 -> 切换 LongitudinalPersonality ==========
        if self.dp_long_use_stock_acc_distance_button and self.CP.openpilotLongitudinalControl:
            if self._stock_dist_btn_types:
                if any(be.type in self._stock_dist_btn_types and be.pressed for be in CS.buttonEvents):
                    if self.sm.frame - self._dist_btn_last_cycle > 30:  # 0.3s 防抖,避免单次长按重复触发
                        self._cycle_long_personality()
            else:
                # 发现模式：端口未转发已知类型名,累计记录实际出现的按钮类型名,便于确认信号
                if CS.buttonEvents:
                    for be in CS.buttonEvents:
                        self._dist_btn_seen.add(be.type.name)
                    if self.sm.frame - self._dist_btn_last_log >= 200:  # 约 2s 节流
                        self._dist_btn_last_log = self.sm.frame
                        cloudlog.info(f"[DIST_BTN][DISCOVERY] seen button types so far: {sorted(self._dist_btn_seen)}")

        resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
        if not self.CP.pcmCruise and not self.v_cruise_helper.v_cruise_initialized and resume_pressed:
            self.events.add(EventName.resumeBlocked)
        if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
      (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
        (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
            self.events.add(EventName.pedalPressed)
        if CS.brakePressed and CS.standstill:
            self.events.add(EventName.preEnableStandstill)
        if CS.gasPressed:
            self.events.add(EventName.gasPressedOverride)
        if CS.canValid:
            self.events.add_from_msg(CS.events)
            # 驾驶员监控事件(非仅行车记录仪且未关 IR)
        if not self.CP.notCar and not NO_IR_CTRL:
            self.events.add_from_msg(self.sm['driverMonitoringState'].events)
        if not self.dp_no_fan_ctrl and self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
            if self.sm['peripheralState'].fanSpeedRpm == 0 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
                if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
                    self.events.add(EventName.fanMalfunction)
            else:
                self.last_functional_fan_frame = self.sm.frame

        cal_status = self.sm['liveCalibration'].calStatus
        if cal_status != log.LiveCalibrationData.Status.calibrated:
            if cal_status == log.LiveCalibrationData.Status.uncalibrated:
                self.events.add(EventName.calibrationIncomplete)
            elif cal_status == log.LiveCalibrationData.Status.recalibrating:
                if not self.recalibrating_seen:
                    set_offroad_alert("Offroad_Recalibration", True)
                self.recalibrating_seen = True
                self.events.add(EventName.calibrationRecalibrating)
            else:
                self.events.add(EventName.calibrationInvalid)

        if self.sm['lateralPlan'].laneChangeState == LaneChangeState.preLaneChange:
            direction = self.sm['lateralPlan'].laneChangeDirection
            if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
               (CS.rightBlindspot and direction == LaneChangeDirection.right):
                self.events.add(EventName.laneChangeBlocked)
            else:
                if direction == LaneChangeDirection.left:
                    self.events.add(EventName.preLaneChangeLeft)
                else:
                    self.events.add(EventName.preLaneChangeRight)
        elif self.sm['lateralPlan'].laneChangeState == LaneChangeState.laneChangeStarting:
            self.events.add(EventName.laneChange)
        elif self.sm['lateralPlan'].laneChangeState == LaneChangeState.laneChangeFinishing:
            self.events.add(EventName.laneChange)

        for i, pandaState in enumerate(self.sm['pandaStates']):
            if i < len(self.CP.safetyConfigs):
                safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                pandaState.alternativeExperience != self.CP.alternativeExperience
            else:
                safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

            if safety_mismatch or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
                self.events.add(EventName.controlsMismatch)

            if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
                self.events.add(EventName.relayMalfunction)
        num_events = len(self.events)
        not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
        if self.sm.rcv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
            self.events.add(EventName.processNotRunning)
            if not_running != self.not_running_prev:
                cloudlog.event("process_not_running", not_running=not_running, error=True)
            self.not_running_prev = not_running
    
        if not self.sm.valid['pandaStates']:
            self.events.add(EventName.usbError)
        if CS.canTimeout:
            self.events.add(EventName.canBusMissing)
        elif not CS.canValid:
            self.events.add(EventName.canError)
        can_rcv_timeout = self.can_rcv_timeout_counter >= 5
        has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
        no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
        if (not self.sm.all_checks() or can_rcv_timeout) and no_system_errors:
            if not self.sm.all_alive():
                self.events.add(EventName.commIssue)
            elif not self.sm.all_freq_ok():
                self.events.add(EventName.commIssueAvgFreq)
            else: 
                self.events.add(EventName.commIssue)

            logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
        'can_rcv_timeout': can_rcv_timeout,
            }
            if logs != self.logged_comm_issue:
                cloudlog.event("commIssue", error=True, **logs)
                self.logged_comm_issue = logs
        else:
            self.logged_comm_issue = None

        if not self.sm['liveParameters'].valid and not TESTING_CLOSET and (not SIMULATION or REPLAY):
            self.events.add(EventName.vehicleModelInvalid)
        if not self.sm['lateralPlan'].mpcSolutionValid:
            self.events.add(EventName.plannerError)
        if not (self.sm['liveParameters'].sensorValid or self.sm['liveLocationKalman'].sensorsOK) and not NOSENSOR:
            if self.sm.frame > 5 / DT_CTRL: 
                self.events.add(EventName.sensorDataInvalid)
        if not self.sm['liveLocationKalman'].posenetOK:
            self.events.add(EventName.posenetInvalid)
        if not self.sm['liveLocationKalman'].deviceStable:
            self.events.add(EventName.deviceFalling)

        if not REPLAY:
            cruise_mismatch = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
            self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
            if self.cruise_mismatch_counter > int(6. / DT_CTRL):
                self.events.add(EventName.cruiseMismatch)

        stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
        model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
        planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
        if planner_fcw or model_fcw:
            self.events.add(EventName.fcw)

        for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
            try:
                msg = m.androidLog.message
                if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
                    csid = msg.split("CSID:")[-1].split(" ")[0]
                    evt = CSID_MAP.get(csid, None)
                    if evt is not None:
                        self.events.add(evt)
            except UnicodeDecodeError:
                pass

                # TODO: fix simulator
        if not SIMULATION or REPLAY:
            if not NOSENSOR and not self.dp_no_gps_ctrl:
            # rick - assuming gps never ok before and it's ok once, meaning the gps is functioning
                if not self.dp_gps_ok_once and self.sm['liveLocationKalman'].gpsOK:
                    self.dp_gps_ok_once = True
                if self.dp_gps_ok_once and not self.sm['liveLocationKalman'].gpsOK and self.sm['liveLocationKalman'].inputsOK and (self.distance_traveled > 1500):
                # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
                    self.events.add(EventName.noGps)
                    if self.distance_traveled > 2000:
                        self.dp_no_gps_ctrl = True
                if self.sm['liveLocationKalman'].gpsOK:
                    self.distance_traveled = 0

            if self.sm['modelV2'].frameDropPerc > 20:
                self.events.add(EventName.modeldLagging)
            if self.sm['liveLocationKalman'].excessiveResets:
                self.events.add(EventName.localizerMalfunction)
    def data_sample(self):
        can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
        CS = self.CI.update(self.CC, can_strs)
        if len(can_strs) and REPLAY:
            self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

        self.sm.update(0)

        if not self.initialized:
            all_valid = CS.canValid and self.sm.all_checks()
            timed_out = self.sm.frame * DT_CTRL > (6. if REPLAY else 3.5)
            if all_valid or timed_out or (SIMULATION and not REPLAY):
                available_streams = VisionIpcClient.available_streams("camerad", block=False)
                if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
                    self.sm.ignore_alive.append('roadCameraState')
                if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
                    self.sm.ignore_alive.append('wideRoadCameraState')

                if not self.read_only:
                    self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])

                self.initialized = True
                self.set_initial_state()
                put_bool_nonblocking("ControlsReady", True)

        if not can_strs:
            self.can_rcv_timeout_counter += 1
            self.can_rcv_cum_timeout_counter += 1
        else:
            self.can_rcv_timeout_counter = 0
        if not self.enabled:
            self.mismatch_counter = 0

        if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
        if ps.safetyModel not in IGNORED_SAFETY_MODES):
            self.mismatch_counter += 1

        self.distance_traveled += CS.vEgo * DT_CTRL

        return CS

    def state_transition(self, CS):
        self.v_cruise_helper.update_v_cruise(CS, self.enabled, self.is_metric)
        self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

        self.current_alert_types = [ET.PERMANENT]

        if self.state != State.disabled:
            if self.events.contains(ET.USER_DISABLE):
                self.state = State.disabled
                self.current_alert_types.append(ET.USER_DISABLE)

            elif self.events.contains(ET.IMMEDIATE_DISABLE):
                self.state = State.disabled
                self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

            else:
                if self.state == State.enabled:
                    if self.events.contains(ET.SOFT_DISABLE):
                        self.state = State.softDisabling
                        self.soft_disable_timer = int(SOFT_T / DT_CTRL)
                        self.current_alert_types.append(ET.SOFT_DISABLE)

                    elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
                        self.state = State.overriding
                        self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

                elif self.state == State.softDisabling:
                    if not self.events.contains(ET.SOFT_DISABLE):
                        self.state = State.enabled

                    elif self.soft_disable_timer > 0:
                        self.current_alert_types.append(ET.SOFT_DISABLE)

                    elif self.soft_disable_timer <= 0:
                        self.state = State.disabled

                elif self.state == State.preEnabled:
                    if not self.events.contains(ET.PRE_ENABLE):
                        self.state = State.enabled
                    else:
                        self.current_alert_types.append(ET.PRE_ENABLE)

                elif self.state == State.overriding:
                    if self.events.contains(ET.SOFT_DISABLE):
                        self.state = State.softDisabling
                        self.soft_disable_timer = int(SOFT_T / DT_CTRL)
                        self.current_alert_types.append(ET.SOFT_DISABLE)
                    elif not (self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL)):
                        self.state = State.enabled
                    else:
                        self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

        elif self.state == State.disabled:
            if self.events.contains(ET.ENABLE):
                if self.events.contains(ET.NO_ENTRY):
                    self.current_alert_types.append(ET.NO_ENTRY)

                else:
                    if self.events.contains(ET.PRE_ENABLE):
                        self.state = State.preEnabled
                    elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
                        self.state = State.overriding
                    else:
                        self.state = State.enabled
                    self.current_alert_types.append(ET.ENABLE)
                    self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode)
                    # PCM模式（原车ACC）下 initialize_v_cruise 直接return，需手动设置初始速度
                    if self.CP.pcmCruise:
                        initial_speed = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if self.experimental_mode else V_CRUISE_INITIAL
                        self.v_cruise_helper.v_cruise_kph = initial_speed
                        self.v_cruise_helper.v_cruise_cluster_kph = initial_speed

        self.enabled = self.state in ENABLED_STATES
        self.active = self.state in ACTIVE_STATES

        if self.active or (self._dp_alka and self._dp_alka_active):
            self.current_alert_types.append(ET.WARNING)

    def _determine_accel_limits(self):
    # 根据场景标志位组合确定加速度限制
        flags = self._scene_flags
    
        if flags['emergency_static']:
            return AC_SL_MAX, AC_EMERGENCY_MIN     # 紧急静态前车：强制最大减速度
        elif flags['stopped_lead']:
            # 前车已开始移动(>LEAD_START_MOVE_SPEED)时不再锁死加速上限：
            # 红绿灯前车起步蠕行(约1km/h)时若仍按静止前车给 a_max=0，自车会被钳死只能怠速蠕行
            lead = self.sm['radarState'].leadOne
            lead_started = lead is not None and lead.status and lead.vLead > LEAD_START_MOVE_SPEED
            if lead_started:
                return AC_NORM_MAX, AC_NORM_MIN
            return AC_SL_MAX, AC_SL_MIN
        elif flags['high_speed_approach']:
            return AC_HSA_MAX, AC_HSA_MIN
        elif flags['curve']:
            severity = getattr(self, '_curve_severity', 0.0)
            a_max = AC_CV_MAX_B * (1.0 - severity * AC_CV_MSF)
            a_min = AC_CV_MIN_B - severity * AC_CV_MS2
            return a_max, a_min
        else:
            return AC_NORM_MAX, AC_NORM_MIN
  
    def state_control(self, CS):
    # ========== 重置场景标志位和加速度限制 ==========
        for key in self._scene_flags:
            self._scene_flags[key] = False
        self._static_lead_active = False      # 每帧重置，由 SS 检测块重新置位，避免陈旧状态误触发
        self._curve_severity = 0.0
        lp = self.sm['liveParameters']
        x = max(lp.stiffnessFactor, 0.1)
        sr = max(lp.steerRatio, 0.1)
        self.VM.update_params(x, sr)
        v_ego_kph = CS.vEgo * CV.MS_TO_KPH

        if self.CP.lateralTuning.which() == 'torque':
            torque_params = self.sm['liveTorqueParameters']
            if self.is_qin_dmi:
            # 秦 PLUS DM-i:全用车口控制文件 CP.lateralTuning.torque 的扭矩参数,
            # 每帧覆盖实时学习(常量填了具体数值时则以常量为准)
                tp = self.CP.lateralTuning.torque
                qin_lat_accel_factor = QIN_TORQUE_LAT_ACCEL_FACTOR if QIN_TORQUE_LAT_ACCEL_FACTOR is not None else tp.latAccelFactor
                qin_lat_accel_offset = QIN_TORQUE_LAT_ACCEL_OFFSET if QIN_TORQUE_LAT_ACCEL_OFFSET is not None else tp.latAccelOffset
                qin_friction = QIN_TORQUE_FRICTION_COEFF if QIN_TORQUE_FRICTION_COEFF is not None else tp.friction
                self.LaC.update_live_torque_params(qin_lat_accel_factor, qin_lat_accel_offset, qin_friction)
            elif self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams and not self.torqued_override:
                self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                torque_params.frictionCoefficientFiltered)

        lat_plan = self.sm['lateralPlan']
        long_plan = self.sm['longitudinalPlan']
        model_v2 = self.sm['modelV2']
        lead_one = self.sm['radarState'].leadOne

        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        actuators = CC.actuators
        actuators.longControlState = self.LoC.long_control_state
        standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
        CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                       (not standstill or self.joystick_mode)
        CC.longActive = self.enabled and not self.events.contains(ET.OVERRIDE_LONGITUDINAL) and self.CP.openpilotLongitudinalControl

        if self.enabled:
            if self._last_use_stock_acc:
                use_stock_acc = ACC_ON_KPH < v_ego_kph
            else:
                use_stock_acc = ACC_OFF_KPH < v_ego_kph

                # ========== E2E 红绿灯：低速强制实验模式窗口（按实际车速判定） ==========
                # 5~70km/h 窗口内强制实验模式，让 E2E 模型的停止线/信号灯识别分支生效
                # （对齐 controlsd.py11 已验证写法，不依赖 trafficLightState 字段）。
                # 须放在弯道/视觉/雷达分支之前，保证 traffic_light 场景参与后续
                # other_scene_active 与接管防抖判定，experimental_mode 不会被清除。
                # 锁存(_tl_engaged)：一旦在窗口内进入过，停稳/低速(<70km/h)期间保持 engaged，
                # 使 blended 模式持续到车停稳乃至绿灯起步。原因：long_mpc 在 acc 模式会把模型
                # E2E 预测清零(只跟 vCruise/前车)，若停稳后(<5km/h)释放实验模式，车会按 vCruise
                # 重新起步、冲过红灯；blended 才跟随模型轨迹，红灯保持停、绿灯随模型预测自动起步。
                # 与 step() 一致：雷达缺失 + dp_0813 模型时原作者刻意关闭实验模式，TL 不应重新打开
            tl_window = TL_CONTROL_ENABLE and self.CP.openpilotLongitudinalControl and \
                  (not (self.CP.radarUnavailable and self.dp_0813)) and \
            TL_LOW_SPEED_MIN_KPH <= v_ego_kph < TL_LOW_SPEED_MAX_KPH
            if tl_window:
                self._tl_engaged = True
            elif v_ego_kph >= TL_LOW_SPEED_MAX_KPH:
                self._tl_engaged = False
            tl_active = tl_window or (TL_CONTROL_ENABLE and self._tl_engaged and \
            (CS.standstill or v_ego_kph < TL_LOW_SPEED_MAX_KPH))
            if tl_active:
                self._scene_flags['traffic_light'] = True
                self.experimental_mode = True

                # ========== 弯道预判减速（简化版）==========
                # 重要：弯道只通过降低 v_cruise 让传统 MPC(acc) 在弯道减速，
                # 不再强制 experimental_mode(blended/E2E)。否则弯道会被 E2E 误判成停止线，
                # 从低速一路刹停且不再起步（用户反馈的"转弯后慢慢降到停、不走"现象）。
                # 每帧先清 curve 标志，仅在本帧确实检测到弯道时才置 True。
            if hasattr(model_v2, 'orientationRate') and len(model_v2.orientationRate.z) > 0 and v_ego_kph > CV_MIN_KPH:
                self._scene_flags['curve'] = False
                curve_detected, curve_severity, curve_distance = self._detect_curve_ahead(model_v2, CS.vEgo)
                if curve_detected:
                    self._curve_cooldown_timer = CV_CD
                    self._curve_severity = curve_severity
                    self._scene_flags['curve'] = True
                    self.experimental_mode = False
                    if not self._curve_active:
                        self._v_cruise_before_curve = self.v_cruise_helper.v_cruise_kph
                        self._curve_active = True
                    target_speed_kph = max(self.v_cruise_helper.v_cruise_kph - self._curve_severity * CV_DROP, CV_TARGET)
                    target_speed_ms = target_speed_kph * CV.KPH_TO_MS
                    if CS.vEgo > target_speed_ms and curve_distance > 0:
                        safe_speed_ms = min(CS.vEgo, (target_speed_ms ** 2 + 2.0 * CV_DEC * curve_distance) ** 0.5)
                        safe_speed_ms = max(CS.vEgo - CV_STEP * DT_CTRL, safe_speed_ms)
                        safe_speed_kph = safe_speed_ms * CV.MS_TO_KPH
                        if self.v_cruise_helper.v_cruise_kph > safe_speed_kph:
                            self.v_cruise_helper.v_cruise_kph = safe_speed_kph
                elif self._curve_active:
                    self._curve_cooldown_timer -= DT_CTRL
                    if self._curve_cooldown_timer <= 0:
                        self._curve_active = False
                        self._curve_severity = 0.0
                        self._scene_flags['curve'] = False
                        if self._v_cruise_before_curve is not None:
                            current_v = self.v_cruise_helper.v_cruise_kph
                            if current_v < self._v_cruise_before_curve:
                                # 弯道退出后自适应回速（2026-09-26 改）：
                                # 基础 20km/h/s 起步，每多 1km/h gap 多 3% 步长，上限 50km/h/s。
                                # 60km/h gap 从旧 6s 缩到约 1.2s 全程回到 v_cruise_before_curve，
                                # 既不会一帧突跳又比旧固定 10km/h/s 流畅许多。
                                gap = self._v_cruise_before_curve - current_v
                                step = min(CV_REC_MIN + gap * CV_REC_GAIN, CV_REC_MAX)
                                self.v_cruise_helper.v_cruise_kph = min(current_v + step, self._v_cruise_before_curve)
                            else:
                                self._v_cruise_before_curve = None
                    else:
                        self._scene_flags['curve'] = True
                        self.experimental_mode = False

                        # ========== 纯视觉前车检测（带完整安全逻辑）==========
            current_lead_status = False
            other_scene_active = self._scene_flags.get('traffic_light', False) or self._scene_flags.get('curve', False)
            if len(model_v2.leadsV3) > 0:
                lead_v3 = model_v2.leadsV3[0]
                if len(lead_v3.v) > 0 and len(lead_v3.x) > 0:
                    v_vision_lead_kph = lead_v3.v[0] * CV.MS_TO_KPH
                    dist_m = lead_v3.x[0]
                    rel_speed = v_ego_kph - v_vision_lead_kph

                    # --- 高置信度前车 (prob > VL_PROB) ---
                    if lead_v3.prob > VL_PROB:
                        is_stopped = v_vision_lead_kph < VL_STOP_KPH and dist_m < VL_STOP_DIST_M
                        is_close = dist_m < (3.0 + v_ego_kph * 0.4)
                        is_dangerous = rel_speed > VL_DANGER_KPH
                        if is_stopped or (is_close or is_dangerous):
                            current_lead_status = True
                            self.experimental_mode = True
                            if is_stopped:
                                # 静止确认：连续 VL_STOP_CONFIRM_T 秒才点亮，防模型单帧噪声误触发；
                                # 雷达 is_radar_stopped 并行兜底，0.15s 延迟不伤急刹及时性。
                                self._vision_stop_confirm += DT_CTRL
                                if self._vision_stop_confirm >= VL_STOP_CONFIRM_T:
                                    self._set_vision_static_lead(dist_m, v_vision_lead_kph, v_ego_kph)
                            else:
                                self._vision_stop_confirm = 0.0
                                if not is_close:
                                    self._stopped_lead_detected = False
                                # P0：近距离 + 高速接近(rel>35) + 前车明显慢行 -> 升 stopped_lead 档强刹（HSA 兜不住慢速前车）
                                # [C2-OSC-FIX] 加 rel_speed>HSA_REL_SPEED_KPH 与雷达侧 P0 对齐：原逻辑只要求
                                # "近距离+前车绝对速度<30"，市区低速跟慢车(自车35跟前车28)会逐帧误触发
                                # stopped_lead 强刹档，叠加 MPC 振荡形成一脚油门一脚刹车循环
                                if is_close and dist_m < HSA_CLOSE_DIST_M and v_vision_lead_kph < HSA_CLOSE_VLEAD_KPH \
                                        and rel_speed > HSA_REL_SPEED_KPH:
                                    self._set_vision_static_lead(dist_m, v_vision_lead_kph, v_ego_kph)
                                    use_stock_acc = False
                                # P0b：近距离 + 高速接近(rel>35) + 前车中速(30~50km/h) -> 同样升档强刹，
                                # 补 HSA 档(-1.0)兜不住的中速前车盲区。要求 rel 明确高速接近才升档，
                                # 正常跟车接近(rel 小)不升档，防误重刹。
                                elif is_close and dist_m < HSA_CLOSE_DIST_M and rel_speed > HSA_REL_SPEED_KPH \
                                        and v_vision_lead_kph < HSA_CLOSE_VLEAD_KPH_HI:
                                    self._set_vision_static_lead(dist_m, v_vision_lead_kph, v_ego_kph)
                                    use_stock_acc = False
                                # P3：危险接近(相对速度大) -> 立即切 OP 纵向，不等接管防抖窗口
                                elif is_dangerous:
                                    use_stock_acc = False
                        else:
                            # 非静止且非危险/非近距离：重置静止确认计时，防间断信号累积提前触发
                            self._vision_stop_confirm = 0.0

                        hsa_triggered = (v_ego_kph > HSA_EGO_MIN_KPH and
                            rel_speed > HSA_REL_SPEED_KPH and
                        HSA_DIST_MIN_M <= dist_m <= HSA_DIST_MAX_M)
                        if hsa_triggered:
                            self._scene_flags['high_speed_approach'] = True
                            self.experimental_mode = True
                            current_lead_status = True
                        elif not other_scene_active and not self._op_long_takeover:
                            self.experimental_mode = False
                            if not is_stopped:
                                self._stopped_lead_detected = False

                                # --- 视觉“疑似静止前车”软兜底：置信度 0.35~0.6、近距离、几乎停着 -> 按静态前车刹车 ---
                                # 覆盖模型不确定(prob 0.35~0.6)但仍极可能是静止前车的边缘漏检；距离/速度收紧防幽灵刹。
                    suspected_stopped = (VL_PROB_SUSPECT < lead_v3.prob <= VL_PROB) and \
                    dist_m < VL_SUSPECT_DIST_M and v_vision_lead_kph < VL_SUSPECT_STOP_KPH and \
                    v_vision_lead_kph <= VL_STOP_KPH
                    if suspected_stopped:
                        current_lead_status = True
                        self.experimental_mode = True
                        self._set_vision_static_lead(dist_m, v_vision_lead_kph, v_ego_kph)
                        use_stock_acc = False
            elif not other_scene_active and not self._op_long_takeover:
                self.experimental_mode = False
                self._stopped_lead_detected = False

                # ========== 雷达侧疑似静态前车软兜底（视觉低置信/跟丢场景补盲，2026-09-26 加）==========
                # 视觉 leadsV3 prob<0.35 或完全无识别时，夜间跟尾灯丢失、雨雾、强逆光等场景
                # 会出"前车就在那儿但视觉不报"的盲区。本段复用雷达 leadOne：
                # vLead≈0 + 距离近 + 连续满足 RS_RADAR_SUSPECT_CONFIRM_T 秒 -> 软点亮 static lead。
                # 一旦点亮，后续 SS 块的 elif/else 分支不再清 _static_lead_active（防 SS 块误清）。
                self._radar_suspect_confirm = 0.0
                self._radar_suspect_active = False
                if lead_one is not None and lead_one.status and not self._scene_flags.get('stopped_lead', False):
                    rs_v_lead_kph = lead_one.vLead * CV.MS_TO_KPH
                    rs_d_rel = lead_one.dRel
                    if rs_v_lead_kph < RS_RADAR_SUSPECT_V_KPH and 0.0 < rs_d_rel < RS_RADAR_SUSPECT_DIST_M:
                        self._radar_suspect_confirm += DT_CTRL
                        if self._radar_suspect_confirm >= RS_RADAR_SUSPECT_CONFIRM_T:
                            # 复用 _set_vision_static_lead 统一设置 stopped_lead + static_lead 相关 5 个字段
                            self._radar_suspect_active = True
                            self._set_vision_static_lead(rs_d_rel, rs_v_lead_kph, v_ego_kph)
                            self.experimental_mode = True
                            current_lead_status = True
                            use_stock_acc = False
                    else:
                        self._radar_suspect_confirm = 0.0

                # ========== 雷达静态前车识别（SS：停着的车 / 高速接近） ==========
                # 雷达 leadOne 提供绝对/相对速度，对静止障碍物比纯视觉更可靠。
                # vision_static 记录视觉块本帧是否已确认静态前车，避免雷达(运动 lead)把该标志误清掉。
            vision_static = self._static_lead_active
            if lead_one is not None and lead_one.status:
                ss_drel = lead_one.dRel
                ss_vlead = lead_one.vLead
                # 前车已开始移动(>LEAD_START_MOVE_SPEED)不再判静止：红绿灯前车起步时 1km/h 蠕行
                # 会被误判静止，导致自车被 stopped_lead 钳 0 无法起步（用户反馈"前车走很远才慢慢动"）
                lead_started = ss_vlead > LEAD_START_MOVE_SPEED
                is_radar_stopped = not lead_started and ss_vlead < SS_RADAR_STOP_SPEED_KPH / CV.MS_TO_KPH and 0.0 < ss_drel < SS_RADAR_MAX_DIST
                rel_speed = v_ego_kph - ss_vlead * CV.MS_TO_KPH
                is_radar_hsa = v_ego_kph > HSA_EGO_MIN_KPH and rel_speed > SS_HIGH_RELSPEED_KPH and \
                               SS_RADAR_REL_DIST < ss_drel < SS_RADAR_MAX_DIST
                if is_radar_stopped:
                    self._static_lead_active = True
                    self._static_lead_dRel = ss_drel
                    self._static_lead_vRel = ss_vlead - CS.vEgo
                    self._stopped_lead_detected = True
                    self._scene_flags['stopped_lead'] = True
                    self.experimental_mode = True
                    current_lead_status = True
                    use_stock_acc = False
                elif is_radar_hsa:
                    if not vision_static and not self._radar_suspect_active:
                        self._static_lead_active = False
                    self._scene_flags['high_speed_approach'] = True
                    self.experimental_mode = True
                    current_lead_status = True
                    # P0：近距离高速接近慢/静前车 -> HSA 档(-1.0)太弱，直接升 stopped_lead 档强刹(-4.5)，
                    # 紧急安全网随后按 a_req/ttc 决定是否进一步 -7.0。要求前车绝对速度也慢(<30km/h)，
                    # 与视觉侧对齐，防止"rel 刚过 10 的正常跟车接近"被误判升档重刹。
                    if ss_drel < HSA_CLOSE_DIST_M and ss_vlead * CV.MS_TO_KPH < HSA_CLOSE_VLEAD_KPH:
                        self._static_lead_active = True
                        self._static_lead_dRel = ss_drel
                        self._static_lead_vRel = ss_vlead - CS.vEgo
                        self._stopped_lead_detected = True
                        self._scene_flags['stopped_lead'] = True
                        use_stock_acc = False
                    # P0b：近距离 + 高速接近(rel>35) + 前车中速(30~50km/h) -> 同样升档强刹，
                    # 与视觉侧 P0b 对齐补盲区；rel 需明确高速接近才升档，防正常跟车接近误触发。
                    elif ss_drel < HSA_CLOSE_DIST_M and rel_speed > HSA_REL_SPEED_KPH \
                            and ss_vlead * CV.MS_TO_KPH < HSA_CLOSE_VLEAD_KPH_HI:
                        self._static_lead_active = True
                        self._static_lead_dRel = ss_drel
                        self._static_lead_vRel = ss_vlead - CS.vEgo
                        self._stopped_lead_detected = True
                        self._scene_flags['stopped_lead'] = True
                        use_stock_acc = False
                else:
                    if not vision_static and not self._radar_suspect_active:
                        self._static_lead_active = False
            elif not self._scene_flags.get('traffic_light', False) and not self._scene_flags.get('curve', False):
                if not vision_static and not self._radar_suspect_active:
                    self._static_lead_active = False

                    # ========== 紧急静态前车安全网（最后兜底，临界即强刹） ==========
                    # 已确认静态前车(_static_lead_active) 且距离/碰撞时间进入临界 -> 强制最大减速度。
                    # 即使 MPC 偏保守或识别偏晚，也能把车在极限内停住。不替代正常 SS 跟停逻辑。
            self._emergency_static = False
            # ========== 方案A：前车急刹联动制动 ==========
            # 雷达前车正在急刹(aLeadK 超过阈值) 且距离/TTC 进入临界 -> 直接走紧急档强刹。
            # 与下方静态安全网互不干扰：急刹中的前车 aLeadK 大负但尚未静止，原安全网不触发，
            # 本联动补上这个缺口；两者都触发时也只会置同一个 _emergency_static 标志。
            if LEAD_EBRAKE_ENABLE and lead_one is not None and lead_one.status:
                if lead_one.aLeadK < LEAD_EBRAKE_ACCEL_TH and 0.0 < lead_one.dRel < LEAD_EBRAKE_DIST_MAX_M:
                    d_rel_eb = max(lead_one.dRel, 0.1)
                    ttc_eb = d_rel_eb / max(CS.vEgo, 0.1)   # 碰撞时间(s)，用自车速度估算
                    if ttc_eb < LEAD_EBRAKE_TTC_S:
                        self._emergency_static = True
                        self._scene_flags['emergency_static'] = True
                        self.experimental_mode = True
                        current_lead_status = True
                        use_stock_acc = False
            if EMERGENCY_STATIC_BRAKE and self._static_lead_active:
                d_rel = max(self._static_lead_dRel, 0.1)
                ttc = d_rel / max(CS.vEgo, 0.1)              # 碰撞时间(s)
                a_req = CS.vEgo ** 2 / (2.0 * d_rel)         # 在 d_rel 内停住所需减速度 (m/s^2)
                # 近距离无条件 / TTC 极小 / 或正常 -4.5 已不足以在当前距离停住 -> 触发最大减速度强刹
                if d_rel < EMERGENCY_DIST_NEAR_M or ttc < EMERGENCY_TTC_S or a_req > abs(AC_SL_MIN):
                    self._emergency_static = True
                    self._scene_flags['emergency_static'] = True
                    self.experimental_mode = True
                    current_lead_status = True
                    use_stock_acc = False

                    # ========== 前车状态防抖 ==========
            if current_lead_status != self._lead_status:
                self._lead_status_timer += DT_CTRL
                if self._lead_status_timer >= LEAD_DB_T:
                    self._lead_status = current_lead_status
                    self._lead_status_timer = 0

                    # ========== ACC/OP 纵向接管双向防抖（核心防点刹逻辑） ==========
                    # 场景瞬时成立/消失不再立刻切换纵向控制器，避免 stock ACC <-> OP 纵向
                    # 高频换手（每次换手都重置 PID，表现为频繁点刹）
            takeover_req = current_lead_status or self._scene_flags['curve'] or self._scene_flags['traffic_light']
            if takeover_req != self._op_long_takeover:
                self._op_long_takeover_timer += DT_CTRL
                need = ACC_TAKEOVER_IN_T if takeover_req else ACC_TAKEOVER_OUT_T
                if self._op_long_takeover_timer >= need:
                    self._op_long_takeover = takeover_req
                    self._op_long_takeover_timer = 0.0
            else:
                self._op_long_takeover_timer = 0.0
                # 安全优先：确认静态前车/紧急时立即切 OP 纵向，跳过切入防抖延迟（0.3s@100km/h≈8m），
                # 避免 stock ACC 接管期间错过最佳刹车时机。交回仍走 OUT 防抖，避免抖动。
            if (self._emergency_static or self._scene_flags['stopped_lead']) and not self._op_long_takeover:
                self._op_long_takeover = True
                self._op_long_takeover_timer = 0.0
            if self._op_long_takeover:
                use_stock_acc = False
            self._last_use_stock_acc = use_stock_acc
            # self.CP.openpilotLongitudinalControl: #车辆是否支持openpilot纵向控制
        if self.CP.openpilotLongitudinalControl:
            CC.longActive = CC.longActive and not self._last_use_stock_acc

            # rick - alka
        if (self._dp_alka and self._dp_alka_active) and not standstill and CS.cruiseState.available:
            if self.sm['liveCalibration'].calStatus != log.LiveCalibrationData.Status.calibrated:
                pass
            elif CS.steerFaultTemporary or CS.steerFaultPermanent:
                pass
            elif CS.gearShifter == car.CarState.GearShifter.reverse:
                pass
            else:
                CC.latActive = True

                # rick - assist-less lane change
        if self._dp_lat_lane_change_assist_disabled:
        # de-activate
            if not CS.leftBlinker and not CS.rightBlinker:
                self._dp_lat_lane_change_assist_disabled_active = False

                # activate
            if not self._dp_lat_lane_change_assist_disabled_active and CS.steeringPressed and ((CS.steeringTorque > 0 and CS.leftBlinker) or (CS.steeringTorque < 0 and CS.rightBlinker)):
                self._dp_lat_lane_change_assist_disabled_active = True

            if self._dp_lat_lane_change_assist_disabled_active:
                self.events.add(EventName.laneChange)
                CC.latActive = False

                # rick - VAG 转向时间炸弹绕过:连续激活一段时间后预警并关闭转向/油门
        if self._dp_vag_timebomb_bypass:
            if not CC.latActive:
                self._dp_vag_timebomb_bypass_counter = 0
            else:
                self._dp_vag_timebomb_bypass_counter += 1

                # 开始预警
                if DP_VAG_TIMEBOMB_BYPASS_WARNING <= self._dp_vag_timebomb_bypass_counter < DP_VAG_TIMEBOMB_BYPASS_START:
                    self.events.add(EventName.steerTimeLimit)

                    # 关闭转向 + 油门
                if self._dp_vag_timebomb_bypass_counter >= DP_VAG_TIMEBOMB_BYPASS_START:
                    self.events.add(EventName.ldw)
                    CC.latActive = False
                    CC.longActive = False

                    # 计数到上限后归零(循环)
                if self._dp_vag_timebomb_bypass_counter >= DP_VAG_TIMEBOMB_BYPASS_END:
                    self._dp_vag_timebomb_bypass_counter = 0

        if CS.leftBlinker or CS.rightBlinker:
            self.last_blinker_frame = self.sm.frame

            # State specific actions
        if not CC.latActive:
            self.LaC.reset()
        if not CC.longActive:
            self.LoC.reset(v_pid=CS.vEgo)
            self._last_cmd_accel = 0.0

        if not self.joystick_mode:
        # accel PID loop
            pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
            t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL
            actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)

            # ========== 最终加速度限制（根据场景组合应用限制，带平滑减速） ==========
            a_max, a_min = self._determine_accel_limits()
            actuators.accel = min(actuators.accel, a_max)
            actuators.accel = max(actuators.accel, a_min)

            # 前车已开始移动(>LEAD_START_MOVE_SPEED)时不钳 0：红绿灯前车起步蠕行时
            # 若 stopped_lead 仍挂起（视觉/雷达状态滞后），继续钳 0 会导致自车锁死无法起步
            lead_started = lead_one is not None and lead_one.status and lead_one.vLead > LEAD_START_MOVE_SPEED
            if CS.vEgo < 0.4 and self._stopped_lead_detected and not lead_started:
                actuators.accel = min(actuators.accel, 0.0)

                # ========== 纵向加速度变化率限幅（平滑场景跳变，消除突然重刹）==========
                # 场景分档（HSA -1.0 / 普通 -3.0 / 停前车 -3.0）会瞬间改变 a_min，这里限制每帧 accel 变化量，
                # 过渡平滑但不削弱最终减速度能力。紧急静态前车安全网时跳过限幅，立即重刹。
            if self._emergency_static:
                self._last_cmd_accel = actuators.accel        # 不平滑，立即落到底
            else:
                max_step = ACCEL_SLEW_RATE * DT_CTRL
                da = actuators.accel - self._last_cmd_accel
                if da > max_step:
                    actuators.accel = self._last_cmd_accel + max_step
                elif da < -max_step:
                    actuators.accel = self._last_cmd_accel - max_step
                self._last_cmd_accel = actuators.accel

                # Steering PID loop (转向 PID 控制)
            self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,lat_plan.psis,lat_plan.curvatures,lat_plan.curvatureRates)
            lat_tuning = self.CP.lateralTuning.which()
            if lat_tuning == 'torque':
                actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,self.last_actuators, self.steer_limited, self.desired_curvature,self.desired_curvature_rate, self.sm['liveLocationKalman'], model_data=model_v2)
            else:
                actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,self.last_actuators, self.steer_limited, self.desired_curvature,self.desired_curvature_rate, self.sm['liveLocationKalman'])
            actuators.curvature = self.desired_curvature

        if CS.steeringPressed:
            self.last_steering_pressed_frame = self.sm.frame
        recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0
        # 如果饱和度计数达到限制，发送'需要转向'警报
        if lac_log.active and not recent_steer_pressed and not self.CP.notCar:
            if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
                undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 1.2
                turning = abs(lac_log.desiredLateralAccel) > 2.0
                good_speed = CS.vEgo > 5
                max_torque = abs(self.last_actuators.steer) > 1.99
                if undershooting and turning and good_speed and max_torque and STEER_SATURATED_ALERT_MODE == 1:
                    lac_log.active and self.events.add(EventName.steerSaturated)

        for p in ACTUATOR_FIELDS:
            attr = getattr(actuators, p)
            if not isinstance(attr, SupportsFloat):
                continue
            if not math.isfinite(attr):
                cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
                setattr(actuators, p, 0.0)
     
                # ========== 路测调试日志(DEBUG_FEATURE_LOG 开启时,每 ~0.5s 打印一次) ==========
        if DEBUG_FEATURE_LOG and self.sm.frame % 50 == 0:
            cloudlog.info(
        f"[DBG] CV act={self._curve_active} sev={self._curve_severity:.3f} v={self.v_cruise_helper.v_cruise_kph:.1f} | "
        f"SS act={self._static_lead_active} d={self._static_lead_dRel:.1f} vr={self._static_lead_vRel:.1f} | "
        f"EMG act={self._emergency_static} | "
        f"TL act={self._scene_flags.get('traffic_light', False)} | "
        f"en={self.enabled} exp={self.experimental_mode} long={CC.longActive} "
            f"opLong={self.CP.openpilotLongitudinalControl}")

        return CC, lac_log

    def publish_logs(self, CS, start_time, CC, lac_log):
        """向车辆发送执行器和 HUD 命令，发送控制状态和 MPC 日志记录"""
        orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
        if len(orientation_value) > 2:
            CC.orientationNED = orientation_value
        angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
        if len(angular_rate_value) > 2:
            CC.angularVelocity = angular_rate_value
        CC.cruiseControl.override = self.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
        CC.cruiseControl.cancel = CS.cruiseState.enabled and (not self.enabled or not self.CP.pcmCruise)
        if self.joystick_mode and self.sm.rcv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
            CC.cruiseControl.cancel = True
        speeds = list(self.sm['longitudinalPlan'].speeds)
        accels = self.sm['longitudinalPlan'].accels
        # 起步预测
        if len(speeds):
            CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1 

        hudControl = CC.hudControl  #获取 HUD 对象
        hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_cluster_kph * CV.KPH_TO_MS)
        hudControl.speedVisible = self.enabled  #速度可见性
        hudControl.lanesVisible = self.enabled  #车道线可见性
        hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead  #前车可见性
        hudControl.rightLaneVisible = True  #右车道线可见性
        hudControl.leftLaneVisible = True  #左车道线可见性

        recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
        ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker and not CC.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

        model_v2 = self.sm['modelV2']
        desire_prediction = model_v2.meta.desirePrediction
        if len(desire_prediction) and ldw_allowed:
            right_lane_visible = model_v2.laneLineProbs[2] > 0.5
            left_lane_visible = model_v2.laneLineProbs[1] > 0.5
            l_lane_change_prob = desire_prediction[Desire.laneChangeLeft]
            r_lane_change_prob = desire_prediction[Desire.laneChangeRight]
            lane_lines = model_v2.laneLines
            l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(LDW_LANE_OFFSET + CAMERA_OFFSET))
            r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (LDW_LANE_OFFSET - CAMERA_OFFSET))
            hudControl.leftLaneDepart = bool(l_lane_change_prob > LDW_TH and l_lane_close)
            hudControl.rightLaneDepart = bool(r_lane_change_prob > LDW_TH and r_lane_close)
            # 处理报警
        if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
            self.events.add(EventName.ldw)

        clear_event_types = set()
        if ET.WARNING not in self.current_alert_types:
            clear_event_types.add(ET.WARNING)
        if self.enabled:
            clear_event_types.add(ET.NO_ENTRY)
        alerts = self.events.create_alerts(self.current_alert_types, [self.CP, CS, self.sm, self.is_metric, self.soft_disable_timer])
        self.AM.add_many(self.sm.frame, alerts)
        current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
        if current_alert:
            hudControl.visualAlert = current_alert.visual_alert
        if not self.read_only and self.initialized:
        # send car controls over can
            now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
            self.last_actuators, can_sends = self.CI.apply(CC, now_nanos)
            self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))
            CC.actuatorsOutput = self.last_actuators
            if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
                self.steer_limited = abs(CC.actuators.steeringAngleDeg - CC.actuatorsOutput.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
            else:
                self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2
                # 处理报警
        force_decel = (not NO_IR_CTRL and self.sm['driverMonitoringState'].awarenessStatus < 0.) or (self.state == State.softDisabling)
        lp = self.sm['liveParameters']
        steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
        curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)
        # controlsState
        dat = messaging.new_message('controlsState')
        dat.valid = CS.canValid
        # 设置车辆控制状态的警报信息、警报信息复制/传递
        controlsState = dat.controlsState
        if current_alert:
            controlsState.alertText1 = current_alert.alert_text_1
            controlsState.alertText2 = current_alert.alert_text_2
            controlsState.alertSize = current_alert.alert_size
            controlsState.alertStatus = current_alert.alert_status
            controlsState.alertBlinkingRate = current_alert.alert_rate
            controlsState.alertType = current_alert.alert_type
            controlsState.alertSound = current_alert.audible_alert
            # 原车四格距离按钮切换后的短暂 HUD 提示（无其它告警时显示,避免覆盖关键告警）
        if not current_alert and self.sm.frame < self._dist_btn_toast_until:
            controlsState.alertText1 = f"跟车距离: {self._dist_btn_toast_text}"
            controlsState.alertText2 = ""
            controlsState.alertSize = 1
        controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
        controlsState.lateralPlanMonoTime = self.sm.logMonoTime['lateralPlan']
        controlsState.enabled = self.enabled
        controlsState.active = self.active
        controlsState.curvature = curvature
        controlsState.desiredCurvature = self.desired_curvature
        controlsState.state = self.state
        controlsState.engageable = not self.events.contains(ET.NO_ENTRY)
        controlsState.longControlState = self.LoC.long_control_state
        controlsState.vPid = float(self.LoC.v_pid)
        controlsState.vCruise = float(self.v_cruise_helper.v_cruise_kph)
        controlsState.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)
        controlsState.upAccelCmd = float(self.LoC.pid.p)
        controlsState.uiAccelCmd = float(self.LoC.pid.i)
        controlsState.ufAccelCmd = float(self.LoC.pid.f)
        a_target = get_accel_from_plan(self.CP, speeds, accels)
        controlsState.aTarget = a_target
        controlsState.cumLagMs = -self.rk.remaining * 1000.
        controlsState.startMonoTime = int(start_time * 1e9)
        controlsState.forceDecel = bool(force_decel)
        controlsState.canErrorCounter = self.can_rcv_cum_timeout_counter
        controlsState.experimentalMode = self.experimental_mode
        # 处理横向控制状态并发布多个车辆消息
        lat_tuning = self.CP.lateralTuning.which()
        if self.joystick_mode:
            controlsState.lateralControlState.debugState = lac_log
        elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            controlsState.lateralControlState.angleState = lac_log
        elif lat_tuning == 'pid':
            controlsState.lateralControlState.pidState = lac_log
        elif lat_tuning == 'torque':
            controlsState.lateralControlState.torqueState = lac_log
        elif lat_tuning == 'indi':
            controlsState.lateralControlState.indiState = lac_log
        elif lat_tuning == 'lqr':
            controlsState.lateralControlState.lqrState = lac_log
        self.pm.send('controlsState', dat)
        dat = messaging.new_message('controlsStateExt')
        dat.valid = CS.canValid
        controlsStateExt = dat.controlsStateExt
        controlsStateExt.alkaActive = self._dp_alka_active
        controlsStateExt.alkaEnabled = self._dp_alka
        self.pm.send('controlsStateExt', dat)
        car_events = self.events.to_msg()
        cs_send = messaging.new_message('carState')
        cs_send.valid = CS.canValid
        cs_send.carState = CS
        cs_send.carState.events = car_events
        # 按不同频率发送车辆相关的消息
        self.pm.send('carState', cs_send)
        if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
            ce_send = messaging.new_message('carEvents', len(self.events))
            ce_send.carEvents = car_events
            self.pm.send('carEvents', ce_send)
        self.events_prev = self.events.names.copy()
        if (self.sm.frame % int(50. / DT_CTRL) == 0):
            cp_send = messaging.new_message('carParams')
            cp_send.carParams = self.CP
            self.pm.send('carParams', cp_send)
        cc_send = messaging.new_message('carControl')
        cc_send.valid = CS.canValid
        cc_send.carControl = CC
        self.pm.send('carControl', cc_send)
        self.CC = CC
        # 实时控制系统主循环
    def step(self):
        start_time = time.monotonic()
        self.prof.checkpoint("Ratekeeper", ignore=True)
        self.is_metric = self.params.get_bool("IsMetric")
        self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
        # rick - we should disable experimental mode on radarless car w/ 0.8.13 model
        if self.CP.radarUnavailable and self.dp_0813:
            self.experimental_mode = False
        CS = self.data_sample()
        cloudlog.timestamp("Data sampled")
        self.prof.checkpoint("Sample")

        self.update_events(CS)
        cloudlog.timestamp("Events updated")
        if not self.read_only and self.initialized:
            self.state_transition(CS)
            self.prof.checkpoint("State transition")
            # Compute actuators (runs PID loops and lateral MPC)
        CC, lac_log = self.state_control(CS)
        self.prof.checkpoint("State Control")
        # Publish data
        self.publish_logs(CS, start_time, CC, lac_log)
        self.prof.checkpoint("Sent")
        self.CS_prev = CS
        # 实时控制线程模式
    def controlsd_thread(self):
        while True:
            self.step()
            self.rk.monitor_time()
            self.prof.display()
      
    def is_relaxed_personality(self):
        """获取当前是否为 relaxed（远档）模式。注：原名 is_comfort_mode 中"舒适"与 relaxed/远档反直觉。"""
        personality_param = Params().get("LongitudinalPersonality")
        return personality_param is not None and int(personality_param) == log.LongitudinalPersonality.relaxed
      
def main(sm=None, pm=None, logcan=None):
    controls = Controls(sm, pm, logcan)
    controls.controlsd_thread()
if __name__ == "__main__":
    main()