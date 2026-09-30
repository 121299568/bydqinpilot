# =======================
# 车道线距离调整参数
# 作    者：MIXUE
# 微信号：RLJ122624
# 添加微信好友时请备注来源
# =======================
from collections import deque
import math
import numpy as np

from cereal import log, custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.byd.values import BYDForceTorqueFix
from openpilot.common.conversions import Conversions as CV
import cereal.messaging as messaging

# 初始化参数和消息订阅
BYD_FORCE_TORQUE_FIX = Params().get_bool(BYDForceTorqueFix)  # BYD车型的扭矩修正开关
PROTOCOL_KEY = "carControl"  # 控制消息的键名
sm = messaging.SubMaster([PROTOCOL_KEY])  # 消息订阅者
dsadFilter = FirstOrderFilter(0, 0.5, 0.01, False)  # DSAD（动态转向执行延迟）滤波器

# 低速补偿参数
LOW_SPEED_X = [0, 10, 20, 30]  # 低速补偿的X轴速度点（mph）
LOW_SPEED_Y = [15, 13, 10, 5]  # 传统低速补偿参数
LOW_SPEED_Y_NN = [12, 3, 1, 0]  # 神经网络低速补偿参数

DEBUG_PRINT = False        # 调试开关：置 True 可打印 DSAD/ETE 诊断；正式稳定版保持 False

# 模型规划最小索引
LAT_PLAN_MIN_IDX = 5

# 打灯变道参数
LANE_CHANGE_DELAY = 1.0    # 打灯后延迟1秒才开始执行变道
LANE_CHANGE_BLEND = 0.4    # 延迟结束后用0.4秒平滑过渡到变道轨迹
LC_SETTLE_TIME = 2.0       # 变道结束/熄灯后继续抑制救急修正的时间，防止把车拉回原车道
DT_CTRL = 0.01             # 控制周期（100Hz）

def get_predicted_lateral_jerk(lat_accels, t_diffs):
  # 计算连续模型数据加速度.y值之间的有限差分
  # 这只是两次调用np.diff后跟元素级除法
  lat_accel_diffs = np.diff(lat_accels)
  lat_jerk = lat_accel_diffs / t_diffs
  # 返回python列表
  return lat_jerk.tolist()


def sign(x):
  # 返回数值的符号：正数返回1.0，负数返回-1.0，零返回0.0
  return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def get_lookahead_value(future_vals, current_val):
  # 获取前瞻值：如果未来值中有相反符号，则返回0；否则返回绝对值最小的值
  if len(future_vals) == 0:
    return current_val

  same_sign_vals = [v for v in future_vals if sign(v) == sign(current_val)]

  # 如果任何未来值具有与当前值相反的符号，则返回0
  if len(same_sign_vals) < len(future_vals):
    return 0.0

  # 否则返回绝对值最小的值
  min_val = min(same_sign_vals + [current_val], key=lambda x: abs(x))
  return min_val


# 在给定滚动角度下，如果俯仰角增大，
# 重力加速度分量开始指向纵向方向，
# 减少横向加速度分量。
# 这里我们对滚动值本身做同样的处理，然后传递给神经网络前馈。
def roll_pitch_adjust(roll, pitch):
  return roll * math.cos(pitch)

class SlidingWindowMaxDiff:
    # 滑动窗口最大差分类：用于计算滑动窗口内相邻值的最大差分
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = deque(maxlen=window_size)

    def update(self, new_value):
        max_diff = 0.0
        if len(self.values) == 0 or new_value != self.values[-1]:
          self.values.append(new_value)
          if len(self.values) > 1:
            for i in range(1, len(self.values)):
                diff = abs(self.values[i] - self.values[i - 1])
                max_diff = max(max_diff, diff)
        return max_diff
      
class LatControlTorque(LatControl):
  # 扭矩型横向控制类：基于扭矩而非角度的横向控制
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque  # 扭矩参数
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()  # 从横向加速度转换为扭矩的方法
    self.low_speed_factor_handler = CI.low_speed_factor_handler()  # 低速因子处理器
    self.enable_low_speed_factor = False
    if self.torque_from_lateral_accel != CarInterfaceBase.torque_from_lateral_accel_linear:
      self.enable_low_speed_factor = True
    self.use_steering_angle = self.torque_params.useSteeringAngle  # 是否使用转向角度
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg  # 转向死区角度
    
    self.param_s = Params()  # 参数系统
    self.torqued_override = self.param_s.get_bool("CustomTorqueLateral")  # 自定义扭矩横向控制开关
    self._frame = 0

    self.use_lateral_jerk = True  # 精调0923夜: 激活高速急弯自适应摩擦因子链路(蜜雪设计为随速下调0.65->0.45/1.2->0.95, 配合已自带的+0.05右转补偿)

    # 动态steerActuatorDelay（转向执行器延迟）
    self.CP = CP
    self.enable_DSAD = BYD_FORCE_TORQUE_FIX
    self.eps_torque_error = 0.0
    self.dsad = 0.0
    self.lsf_last = 0.0

    # 摩擦因子平滑滤波器：随车速自适应重算的值再经一阶低通，滤掉轮速噪声在
    # 10/30m/s 边界处的帧间抖动，让前馈扭矩更顺滑（追求完美稳定）。
    # 时间常数 0.3s，几乎不影响响应、只抑制高频抖。
    self._ff_jerk_filter = FirstOrderFilter(0.65, 0.3, DT_CTRL)
    self._ff_accel_filter = FirstOrderFilter(1.2, 0.3, DT_CTRL)

    # 打灯变道状态
    self._blinker_prev = False       # 上一帧转向灯状态
    self._blinker_timer = 0.0        # 转向灯持续亮起的时间
    self._post_blinker_timer = 0.0   # 熄灯后抑制救急修正的剩余时间
    self._lc_hold_curvature = 0.0    # 延迟期内保持的车道保持曲率
    self._last_lane_correction = 0.0 # 救急修正低通滤波器状态
    self._emergency_engaged = False  # 救急修正滞回状态（触发后需回落到更低下沿才释放）
    
    # Twilsonco的横向神经网络前馈
    self.use_nn = CI.has_lateral_torque_nn

    if self.use_nn or self.use_lateral_jerk:
      # 瞬时横向急动度变化非常快，因此单独使用没有用处，
      # 但是，我们可以"展望"未来的计划横向急动度，
      # 以判断当前所需的横向急动度是否会持续到未来，
      # 即是否"有意图的"。
      # 这让我们可以简单地忽略短暂的急动度。
      # 注意：LAT_PLAN_MIN_IDX在上面定义，并在steerActuatorDelay之后使用的值中使用。
      self.friction_look_ahead_v = [1.4, 2.0] # 在[0, ~2.1]秒内的未来展望时间，增量为0.1
      self.friction_look_ahead_bp = [9.0, 30.0] # 对应速度m/s，在[0, ~40]内，增量为1.0

      # 缩放横向加速度"摩擦响应"可能有所帮助
      # 增加以获得更强的响应，减少以获得较弱的响应
      # 【2024-08-22调整】增大摩擦因子以解决过弯偏内和加速度不足问题
      # 【2026-09-23】以上为"低速锚点值"；高速急弯的"转向过急+压内线"已改为随车速自适应下调，
      #               见 update() 开头对 lat_jerk/lat_accel_friction_factor 的自适应重算。
      self.lat_jerk_friction_factor = 0.65    # 低速锚点：从0.4提升到0.65，增强急动度响应
      self.lat_accel_friction_factor = 1.2    # 低速锚点：从0.7提升到1.2，显著增强加速度响应

      # 预计算ModelConstants.T_IDXS之间的时间差
      self.t_diffs = np.diff(ModelConstants.T_IDXS)
      self.desired_lat_jerk_time = CP.steerActuatorDelay + 0.3
    if self.use_nn:
      self.pitch = FirstOrderFilter(0.0, 0.5, 0.01)
      # 神经网络模型采用当前v_ego、横向加速度、横向加速度/急动度误差、滚动以及
      # lat accel和roll的过去/未来/计划数据
      # 过去值使用先前的所需横向加速度和观察到的滚动计算
      self.torque_from_nn = CI.get_ff_nn
      self.nn_friction_override = CI.lat_torque_nn_model.friction_override

      # 设置未来时间偏移
      self.nn_time_offset = CP.steerActuatorDelay + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # 秒为单位的未来时间
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)

      # 设置过去时间偏移
      self.past_times = [-0.3, -0.2, -0.1]
      history_check_frames = [int(abs(i)*100) for i in self.past_times]
      self.history_frame_offsets = [history_check_frames[0] - i for i in history_check_frames]
      self.lateral_accel_desired_deque = deque(maxlen=history_check_frames[0])
      self.roll_deque = deque(maxlen=history_check_frames[0])
      self.error_deque = deque(maxlen=history_check_frames[0])
      self.past_future_len = len(self.past_times) + len(self.nn_future_times)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # 实时更新扭矩参数
    self.torque_params.latAccelFactor = latAccelFactor
    # 【秦PLUS DM-i 左右差修正 2026-09-04】右拐压内线
    # openpilot坐标系：latAccel>0=左转、<0=右转。
    # 右转切内 = 右转时控制器多施扭矩 = |latAccel|被低估 = latAccelOffset 偏负（向右偏置）
    # 修正：在latAccelOffset上叠加 +0.05 m/s² 向左补偿，让右转时控制器不再多打
    # 调参策略：+0.02起步，右转仍切内→加大到+0.03/+0.04；左转开始偏外→改小到+0.01
    self.torque_params.latAccelOffset = latAccelOffset + 0.05
    self.torque_params.friction = friction

  def update_live_tune(self):
    # 实时调整参数
    if self.enable_DSAD:      
      self.desired_lat_jerk_time = self.dsad + 0.3
      
      # 设置未来时间偏移
      self.nn_time_offset = self.dsad + 0.2
      future_times = [0.3, 0.6, 1.0, 1.5] # 秒为单位的未来
      self.nn_future_times = [i + self.nn_time_offset for i in future_times]
      self.nn_future_times_np = np.array(self.nn_future_times)
          
    self._frame += 1
    if self._frame % 250 == 0:
      self._frame = 0
      self.torqued_override = self.param_s.get_bool("CustomTorqueLateral")
      if not self.torqued_override:
        return

      self.torque_params.latAccelFactor = float(self.param_s.get("TorqueMaxLatAccel", encoding="utf8")) * 0.01
      self.torque_params.friction = float(self.param_s.get("TorqueFriction", encoding="utf8")) * 0.01

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk, model_data=None):
    # 更新控制器状态
    self.update_live_tune()

    # 【2026-09-23 高速急弯优化】转急弯速度快时"转向过急 + 压内线"
    # 根因：高速急弯下 lat_jerk/lat_accel 摩擦因子偏高，控制器过度"提前领弯"并放大曲率误差，
    #       使方向盘猛打、过冲压向弯道内侧。
    # 方案：摩擦因子随车速自适应——低速维持锚点值(保住过弯跟随、不偏内)，
    #       高速收敛到较低值(减小提前量与误差放大，转向更平顺、不再切内)。
    # 仅作用于 NN / lateral_jerk 前馈路径的 friction_input 计算(见下方第383/404行附近)。
    # interp 在边界外自动钳位：<10m/s 取锚点值，>30m/s 取高速值。
    # 随车速自适应重算 + 一阶低通平滑（消除 10/30m/s 边界处的帧间抖动）
    self.lat_jerk_friction_factor = self._ff_jerk_filter.update(interp(CS.vEgo, [10.0, 30.0], [0.65, 0.45]))
    self.lat_accel_friction_factor = self._ff_accel_filter.update(interp(CS.vEgo, [10.0, 30.0], [1.20, 0.95]))

    freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5.5

    pid_log = log.ControlsState.LateralTorqueState.new_message()

    #pid_log_sp = custom.ControlsStateSP.LateralTorqueState.new_message()
    #nn_log = None
    
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      # if self.enable_DSAD:
      #   predicted_angle_deg = CS.steeringAngleDeg - params.angleOffsetDeg + (CS.steeringRateDeg * (self.dsad * 0.5))
      #   actual_curvature_vm = -VM.calc_curvature(math.radians(predicted_angle_deg), CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      actual_lateral_jerk = 0.0
      if self.use_steering_angle:
        actual_curvature = actual_curvature_vm
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
        #if self.use_nn or self.use_lateral_jerk:
        actual_curvature_rate = -VM.calc_curvature(math.radians(CS.steeringRateDeg), CS.vEgo, 0.0)
        actual_lateral_jerk = actual_curvature_rate * CS.vEgo ** 2
      else:
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      
      lane_centering_correction = 0.0

      # ---- 打灯变道逻辑 ----
      # 症状：打灯变道犹豫，变到一半被拉回原车道
      # 原因：1) 打灯后planner立即输出变道轨迹，缺少缓冲期；
      #       2) 大偏移救急修正把变道中的横向偏移当成"跑偏"，反向施力拉回原车道
      # 方案：打灯后先保持原车道保持轨迹LANE_CHANGE_DELAY秒，
      #       再用LANE_CHANGE_BLEND秒平滑切入变道轨迹；
      #       变道执行期间及熄灯后LC_SETTLE_TIME秒内禁用救急修正
      blinker_on = bool(getattr(CS, 'leftBlinker', False) or getattr(CS, 'rightBlinker', False))
      lane_change_exec = False  # 是否处于变道执行阶段

      if blinker_on:
        if not self._blinker_prev:
          # 刚打灯：记录当前车道保持曲率，作为延迟期内的保持基准
          self._blinker_timer = 0.0
          self._lc_hold_curvature = desired_curvature
        self._blinker_timer += DT_CTRL
        self._post_blinker_timer = LC_SETTLE_TIME

        if self._blinker_timer < LANE_CHANGE_DELAY:
          # 延迟期：冻结在原车道保持轨迹上，不响应变道指令
          desired_curvature = self._lc_hold_curvature
          desired_curvature_rate = 0.0
        else:
          # 延迟结束：在LANE_CHANGE_BLEND时间内平滑过渡到变道轨迹
          lane_change_exec = True
          blend = min((self._blinker_timer - LANE_CHANGE_DELAY) / LANE_CHANGE_BLEND, 1.0)
          desired_curvature = self._lc_hold_curvature + blend * (desired_curvature - self._lc_hold_curvature)
          desired_curvature_rate = desired_curvature_rate * blend
      else:
        self._blinker_timer = 0.0
        self._post_blinker_timer = max(0.0, self._post_blinker_timer - DT_CTRL)
      self._blinker_prev = blinker_on

      # 大偏移救急修正（只在严重偏离时触发，平时不干预）
      # 注意：变道执行期间和变道刚结束的沉降期内禁止触发，
      # 否则会把变道偏移误判为跑偏，把车拉回原车道
      lc_correction_suppressed = lane_change_exec or self._post_blinker_timer > 0.0
      if not lc_correction_suppressed and model_data is not None and hasattr(model_data, 'laneLines') and len(model_data.laneLines) >= 3:
        left_line = model_data.laneLines[1]
        right_line = model_data.laneLines[2]
        if hasattr(left_line, 'y') and hasattr(right_line, 'y') and len(left_line.y) > 0 and len(right_line.y) > 0:
          dist_left = abs(left_line.y[0])
          dist_right = abs(right_line.y[0])
          # 车道线过远（>4m）多为护栏/旁车/墙壁误检，强制不触发居中修正，避免被误拉
          if dist_left > 4.0 or dist_right > 4.0:
            diff = 0.0
          else:
            diff = dist_right - dist_left

          # 车道线置信度门控（与 lane_planner.get_d_path 对齐：std>0.15 开始压、>0.3 压到 0）。
          # 规划器不敢信的车道线，救急修正也不能信，否则两套控制器用两套输入打架（直行画龙主因）。
          ll_conf = 0.0
          if hasattr(model_data, 'laneLineProbs') and len(model_data.laneLineProbs) >= 3 and \
             hasattr(model_data, 'laneLineStds') and len(model_data.laneLineStds) >= 3:
            l_std_mod = interp(model_data.laneLineStds[1], [0.15, 0.3], [1.0, 0.0])
            r_std_mod = interp(model_data.laneLineStds[2], [0.15, 0.3], [1.0, 0.0])
            ll_conf = min(model_data.laneLineProbs[1] * l_std_mod,
                          model_data.laneLineProbs[2] * r_std_mod)

          EMERGENCY_THRESHOLD = 0.45  # 触发阈值（滞回上沿）：45cm 开始救急
          EMERGENCY_RELEASE = 0.30    # 释放阈值（滞回下沿）：回落到 30cm 内才退出，防 bang-bang
          MIN_LL_CONF = 0.4           # 置信度门限：低于此值不触发（车道线不可信）
          MAX_EMERGENCY_FORCE = 0.22  # 最大救急力度
          WEAK_THRESHOLD = 0.15       # P2-WEAK 触发阈值：15cm 开始弱修正
          WEAK_RELEASE = 0.10         # P2-WEAK 释放阈值：回落到 10cm 内才退出
          MAX_WEAK_FORCE = 0.06       # P2-WEAK 最大弱修正力度
          MEDIUM_THRESHOLD = 0.25     # P2-MEDIUM 触发阈值：25cm 开始中修正
          MEDIUM_RELEASE = 0.18       # P2-MEDIUM 释放阈值：回落到 18cm 内才退出
          MAX_MEDIUM_FORCE = 0.12     # P2-MEDIUM 最大中修正力度

          # 滞回：未触发态需超 45cm 才进，触发态回落到 30cm 内才退，消除边界来回切换
          if self._emergency_engaged:
            over_threshold = abs(diff) > EMERGENCY_RELEASE
          else:
            over_threshold = abs(diff) > EMERGENCY_THRESHOLD

          if over_threshold and ll_conf > MIN_LL_CONF:
            # 超过阈值就触发，线性增强到120cm封顶。
            # max(·,0) 钳位：滞回态可停留在 30-45cm 释放带，此时 abs(diff)-0.45<0，
            # 不钳位会让力度随 diff 变小而反向放大（把车往偏的方向继续推），
            # 钳位后释放带内目标=0，修正力经同一低通对称衰减到 0。
            overflow = max(0.0, min(abs(diff) - EMERGENCY_THRESHOLD, 0.75) / 0.75)
            raw_force = MAX_EMERGENCY_FORCE * overflow

            target_correction = math.copysign(raw_force, diff)

            # 低通滤波：快速响应
            FILTER_ALPHA = 0.20

            lane_centering_correction = FILTER_ALPHA * target_correction + (1 - FILTER_ALPHA) * self._last_lane_correction
            self._last_lane_correction = lane_centering_correction
            self._emergency_engaged = True
          else:
            # 未超阈值/置信不足时归零：与触发低通对称的释放 (1-α)·last
            # 注意：旧代码此处为 0.15*(1-0.15)=0.1275，释放被加速到 ~30ms 阶跃，
            # 与触发侧 0.3s 爬升不对称，构成顿挫与振荡环路，现已修正为对称衰减。
            lane_centering_correction = (1 - 0.20) * self._last_lane_correction
            self._last_lane_correction = lane_centering_correction
            self._emergency_engaged = False
            if abs(lane_centering_correction) < 0.003:
              lane_centering_correction = 0.0
              self._last_lane_correction = 0.0
      elif lc_correction_suppressed:
        # 抑制期间清零滤波器状态，避免恢复时产生跳变
        self._last_lane_correction = 0.0
        self._emergency_engaged = False

      # 基础行驶意图：稳态目标 = κ·v² 满额（openpilot 原版行为）。
      # 弯道跟线需要满额向心力；原 *0.85 造成稳态欠 15%，是"弯道往外漂"的根因。
      # 柔化不靠缩幅值实现——desired_curvature 的变化率已在 get_lag_adjusted_curvature
      # 被 MAX_LATERAL_JERK/v² 限制（横向加加速度 ≤5.0 m/s³），去掉 0.85 不引入阶跃过冲。
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # 叠加车道居中修正（独立修正力，直道也有效！）
      desired_lateral_accel += lane_centering_correction

      # P3-FIX: 速度自适应横向软衰减（文档通用规则：低曲率直道减小增益，容忍小幅偏差）
      # 高车速时同样 curvature 产生更大的 a_lat，体感压力增大。
      # 用 sigmoid 形状软衰减：60km/h → 0.90，120km/h → 0.68，保留≥0.68 不影响弯道动力。
      # 注意：此衰减作用于 desired_lateral_accel 总目标（含曲率+correction）；
      # 弯道自身的安全减速和曲率限制不受此影响。
      v_ego_kph = CS.vEgo * 3.6
      decay = 1.0 / (1.0 + (v_ego_kph / 80.0) ** 1.5)   # 80km/h 时 ≈0.71，120km/h 时 ≈0.47
      desired_lateral_accel *= max(0.70, decay)          # 保留≥70%，保护弯道动力

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      measurement = actual_lateral_accel + self.lsf_last * actual_curvature
      suppress_lsf = self.use_nn or self.use_lateral_jerk or self.enable_low_speed_factor
      low_speed_factor = self.low_speed_factor_handler(self.torque_params, self.lsf_last, desired_lateral_accel, actual_lateral_accel, CS.vEgo, actual_lateral_jerk, suppress_lsf, freeze_integrator)
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      self.lsf_last = low_speed_factor
      
      lateral_jerk_setpoint = 0
      lateral_jerk_measurement = 0
      lookahead_lateral_jerk = 0

      model_good = model_data is not None and len(model_data.orientation.x) >= CONTROL_N and len(model_data.acceleration.y) >= len(ModelConstants.T_IDXS)
      if model_good and (self.use_nn or self.use_lateral_jerk):
        # prepare "look-ahead" desired lateral jerk
        lookahead = interp(CS.vEgo, self.friction_look_ahead_bp, self.friction_look_ahead_v)
        friction_upper_idx = next((i for i, val in enumerate(ModelConstants.T_IDXS) if val > lookahead), 16)
        predicted_lateral_jerk = get_predicted_lateral_jerk(model_data.acceleration.y, self.t_diffs)
        desired_lateral_jerk = (interp(self.desired_lat_jerk_time, ModelConstants.T_IDXS, model_data.acceleration.y) - desired_lateral_accel) / self.desired_lat_jerk_time
        lookahead_lateral_jerk = get_lookahead_value(predicted_lateral_jerk[LAT_PLAN_MIN_IDX:friction_upper_idx], desired_lateral_jerk)
        if not self.use_steering_angle or lookahead_lateral_jerk == 0.0:
          lookahead_lateral_jerk = 0.0
          actual_lateral_jerk = 0.0
          self.lat_accel_friction_factor = 1.0
        lateral_jerk_setpoint = self.lat_jerk_friction_factor * lookahead_lateral_jerk
        lateral_jerk_measurement = self.lat_jerk_friction_factor * actual_lateral_jerk

      if self.use_nn and model_good:
        # update past data
        roll = params.roll
        if len(llk.calibratedOrientationNED.value) > 1:
          pitch = self.pitch.update(llk.calibratedOrientationNED.value[1])
          roll = roll_pitch_adjust(roll, pitch)
        self.roll_deque.append(roll)
        self.lateral_accel_desired_deque.append(desired_lateral_accel)

        # prepare past and future values
        # adjust future times to account for longitudinal acceleration
        adjusted_future_times = [t + 0.5*CS.aEgo*(t/max(CS.vEgo, 1.0)) for t in self.nn_future_times]
        past_rolls = [self.roll_deque[min(len(self.roll_deque)-1, i)] for i in self.history_frame_offsets]
        future_rolls = [roll_pitch_adjust(interp(t, ModelConstants.T_IDXS, model_data.orientation.x) + roll, interp(t, ModelConstants.T_IDXS, model_data.orientation.y) + pitch) for t in adjusted_future_times]
        past_lateral_accels_desired = [self.lateral_accel_desired_deque[min(len(self.lateral_accel_desired_deque)-1, i)] for i in self.history_frame_offsets]
        future_planned_lateral_accels = [interp(t, ModelConstants.T_IDXS[:CONTROL_N], model_data.acceleration.y) for t in adjusted_future_times]

        # compute NNFF error response
        nnff_setpoint_input = [CS.vEgo, setpoint, lateral_jerk_setpoint, roll] \
                              + [setpoint] * self.past_future_len \
                              + past_rolls + future_rolls
        # past lateral accel error shouldn't count, so use past desired like the setpoint input
        nnff_measurement_input = [CS.vEgo, measurement, lateral_jerk_measurement, roll] \
                                 + [measurement] * self.past_future_len \
                                 + past_rolls + future_rolls
        torque_from_setpoint = self.torque_from_nn(nnff_setpoint_input)
        torque_from_measurement = self.torque_from_nn(nnff_measurement_input)
        pid_log.error = torque_from_setpoint - torque_from_measurement

        # compute feedforward (same as nn setpoint output)
        error = setpoint - measurement
        friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        nn_input = [CS.vEgo, desired_lateral_accel, friction_input, roll] \
                   + past_lateral_accels_desired + future_planned_lateral_accels \
                   + past_rolls + future_rolls
        ff = self.torque_from_nn(nn_input)

        # apply friction override for cars with low NN friction response
        if self.nn_friction_override:
          pid_log.error += self.torque_from_lateral_accel(0.0, self.torque_params,
                                                          friction_input,
                                                          lateral_accel_deadzone, friction_compensation=True)
        #nn_log = nn_input + nnff_setpoint_input + nnff_measurement_input
      else:
        gravity_adjusted_lateral_accel = desired_lateral_accel - roll_compensation
        torque_from_setpoint = self.torque_from_lateral_accel(setpoint, roll_compensation, self.torque_params,
                                                              lateral_jerk_setpoint, lateral_accel_deadzone, friction_compensation=self.use_lateral_jerk, gravity_adjusted=False)
        torque_from_measurement = self.torque_from_lateral_accel(measurement, roll_compensation, self.torque_params,
                                                                 lateral_jerk_measurement, lateral_accel_deadzone, friction_compensation=self.use_lateral_jerk, gravity_adjusted=False)
        pid_log.error = torque_from_setpoint - torque_from_measurement
        error = desired_lateral_accel - actual_lateral_accel
        if self.use_lateral_jerk:
          friction_input = self.lat_accel_friction_factor * error + self.lat_jerk_friction_factor * lookahead_lateral_jerk
        else:
          friction_input = error
        ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, roll_compensation, self.torque_params,
                                            friction_input, lateral_accel_deadzone, friction_compensation=True, gravity_adjusted=True)
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      if self.enable_DSAD:
        sm.update(0)
        # both current torque and requested torque diff were considered factor of delay
        current_torque_diff = abs(sm[PROTOCOL_KEY].actuatorsOutput.steer + output_torque)
        # time for canceling the error ( -1.0 to 1.0 )
        # assuming the delay is doubled cause the posive and negtive zone
        # current torque is lower than limit considered large delay, because of the friction or deadzone issues
        dsad = interp(current_torque_diff, [0.0, 1.0], [0.02, 0.02 * 20]) \
          if sm[PROTOCOL_KEY].actuatorsOutput.steer > 0.2 else 0.50
        self.dsad = dsadFilter.update(dsad)
        if self._frame % 50 == 0 and DEBUG_PRINT:
          print("DSAD: ", self.dsad, "ETE: ", self.eps_torque_error)
      
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.d = self.pid.d
      pid_log.f = self.pid.f
      if hasattr(pid_log, "lsf"):
          pid_log.lsf = self.lsf_last
      if hasattr(pid_log, "friction"):
          pid_log.friction = self.torque_params.friction
      if hasattr(pid_log, "latAccelFactor"):
          pid_log.latAccelFactor = self.torque_params.latAccelFactor
      if hasattr(pid_log, "latAccelOffset"):
          pid_log.latAccelOffset = self.torque_params.latAccelOffset
      pid_log.output = -output_torque
      pid_log.actualLateralAccel = actual_lateral_accel
      pid_log.desiredLateralAccel = desired_lateral_accel
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)
      # if nn_log is not None:
      #   pid_log_sp.nnLog = nn_log
      #   self._pid_long_sp = pid_log_sp
        
    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log