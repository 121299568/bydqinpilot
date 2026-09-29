import time
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.numpy_fast import interp
from openpilot.system.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from openpilot.selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import N as LAT_MPC_N
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log
from openpilot.selfdrive.hardware import EON

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
from openpilot.common.conversions import Conversions as CV

TRAJECTORY_SIZE = 33
if EON:
  CAMERA_OFFSET = -0.06
else:
  CAMERA_OFFSET = 0.04

PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.05
STEERING_RATE_COST = 800.0

# Experimental planner scheduling. Straight-road damping reduces weaving and
# abrupt lane changes, while confirmed curves receive more path authority and
# less steering-rate suppression.
# 【2026-09-23 修改】原设计"弯道降低转向速率代价(660<800)以换取路径权威"，
# 对秦PLUS 反而导致：急弯+高速时 steering_rate_cost 恒为最激进的 660 → 方向盘猛打(转得很急)。
# 改为弯道转向速率代价高于直道，主动抑制急弯变向速率；配合 latcontrol_torque 的高速摩擦因子自适应，
# 一并解决"转急弯速度快→转得很急 + 压内线"。路径权威仍靠 CURVE_PATH_COST 保证。
STRAIGHT_LATERAL_JERK_COST = 0.07
CURVE_LATERAL_JERK_COST = 0.07        # 原0.05：弯道加加速度代价提到与直道一致，抑制急弯突变
CURVE_PATH_COST = 1.15
CURVE_STEERING_RATE_COST = 850.0      # 原660：提到高于直道800，弯道主动抑制转向速率(不再比直道更敢打)
LANE_CHANGE_LATERAL_JERK_COST = 0.08
LANE_CHANGE_STEERING_RATE_COST = 900.0
CURVE_YAW_RATE = 0.12
CURVE_LOOKAHEAD_POINTS = 6

# 【超车远离偏移 / traffic-aware clearance】2026-09-29
# 旁边车道有车（并行 / 超车 / 被超）时，期望路径向远侧鼓出一个平滑偏移，
# 超完自然回正。注入点在两种路径模式合并之后、MPC 之前，下游 MPC 与
# latcontrol 完全无感知。坐标系约定：y 正 = 左，旁车在左 (y>0) → 偏移<0 → 往右挪。
#
# 运动学处理（三点）：
#  1. 鼓包中心用相对距离 lead_x，旁车一相对运动鼓包就跟着扫，前后窗口双向开放；
#  2. 速度前瞻：用 vRel 预测 OVERTAKE_LOOKAHEAD_T 秒后的相对距离，补偿 0.5s 低通的建立滞后；
#  3. 切入衰减：横向无可靠速度源（雷达只有 yRel、视觉只有纵向 velocity、帧间差分噪声大），
#     改用"下限降到 0.6m + prox 分段插值"，让旁车切入过程中偏移平滑衰减到零，无阶跃。
OVERTAKE_ENABLE = True            # 总开关；关掉后路径行为与改动前完全一致
OVERTAKE_MIN_SPEED = 10.0         # m/s；低于此速不生效（城市近距离跟车不瞎躲）
OVERTAKE_MAX_OFFSET = 0.30        # m；保守起步，路测确认后可放大
OVERTAKE_MARGIN = 0.50            # m；距远侧车道线的最小余量（护栏硬限制）
OVERTAKE_WINDOW_FRONT = 30.0      # m；前方窗口
OVERTAKE_WINDOW_REAR = 10.0       # m；后方窗口（被超同样生效）
OVERTAKE_Y_DETECT = 0.6           # m；|y| 检测下限，低于此认定同车道
OVERTAKE_Y_FULL = 1.4             # m；|y| 达到此值视为完整"旁边车"，prox=1.0
OVERTAKE_Y_MAX = 4.5              # m；|y| 超过此值视为隔离带 / 对向，不躲
OVERTAKE_HALF_WIDTH = 10.0        # m；鼓包纵向半宽（升余弦剖面）
OVERTAKE_TAU = 0.5                # s；一阶低通时间常数，抗目标抖动
OVERTAKE_LOOKAHEAD_T = 0.5        # s；速度前瞻时长，与低通滞后对齐
OVERTAKE_SCORE_MIN = 0.5          # 视觉置信度门控
OVERTAKE_CONFIRM_FRAMES = 3       # 连续命中帧数才出偏移（@20Hz ≈ 0.15s）
OVERTAKE_MAX_LAT_ACCEL = 2.0      # m/s²；过弯横向加速度超过此值则关闭
OVERTAKE_VREL_CLAMP = 20.0        # m/s；相对速度离群值钳制


class LateralPlanner:
  def __init__(self, CP, debug=False):
    self.DH = DesireHelper()

    self.params = Params()
    self._dp_lat_lane_priority_mode = self.params.get_bool("dp_lat_lane_priority_mode")
    self._dp_lat_lane_priority_mode_active = False
    self._dp_lat_lane_priority_mode_active_prev = False
    self.LP = LanePlanner()
    self._d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self._dp_lat_lane_priority_mode_speed_based = int(self.params.get("dp_lat_lane_priority_mode_speed_based", encoding="utf-8")) if self._dp_lat_lane_priority_mode else 0
    self.param_read_counter = 0
    self._dp_lat_lane_change_assist_speed = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) * CV.MPH_TO_MS
    try:
      self._dp_path_offset = float(self.params.get("dp_path_offset", encoding="utf-8")) * 0.01
    except (TypeError, ValueError):
      self._dp_path_offset = 0.0

    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0

    self.debug_mode = debug

    # 超车远离偏移：低通后的偏移量、鼓包中心、连续帧计数、日志节流
    self._traffic_offset = 0.0
    self._adjacent_lead_x = 0.0
    self._overtake_confirm_frames = 0
    self.last_ovt_log_t = 0.0

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

  def reset_mpc(self, x0=None):
    if x0 is None:
      x0 = np.zeros(4)
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def _get_mpc_weights(self, yaw_rate_pts):
    lookahead = yaw_rate_pts[:min(CURVE_LOOKAHEAD_POINTS, len(yaw_rate_pts))]
    peak_yaw_rate = float(np.max(np.abs(lookahead))) if len(lookahead) else 0.0
    curve_strength = float(np.clip(peak_yaw_rate / CURVE_YAW_RATE, 0.0, 1.0))

    path_cost = interp(curve_strength, [0.0, 1.0], [PATH_COST, CURVE_PATH_COST])
    lateral_jerk_cost = interp(curve_strength, [0.0, 1.0],
                               [STRAIGHT_LATERAL_JERK_COST, CURVE_LATERAL_JERK_COST])
    steering_rate_cost = interp(curve_strength, [0.0, 1.0],
                                [STEERING_RATE_COST, CURVE_STEERING_RATE_COST])

    lane_change_active = self.DH.desire in (log.LateralPlan.Desire.laneChangeLeft,
                                             log.LateralPlan.Desire.laneChangeRight)
    if lane_change_active:
      path_cost = PATH_COST
      lateral_jerk_cost = max(lateral_jerk_cost, LANE_CHANGE_LATERAL_JERK_COST)
      steering_rate_cost = max(steering_rate_cost, LANE_CHANGE_STEERING_RATE_COST)

    return path_cost, lateral_jerk_cost, steering_rate_cost

  def update(self, sm):
    self.v_ego = max(MIN_SPEED, sm['carState'].vEgo)
    measured_curvature = sm['controlsState'].curvature

    if self.param_read_counter % 50 == 0:
      self._dp_lat_lane_priority_mode = self.params.get_bool("dp_lat_lane_priority_mode")
      try:
        self._dp_path_offset = float(self.params.get("dp_path_offset", encoding="utf-8")) * 0.01
      except (TypeError, ValueError):
        self._dp_path_offset = 0.0
      if self._dp_lat_lane_priority_mode:
        self._dp_lat_lane_priority_mode_speed_based = int(self.params.get("dp_lat_lane_priority_mode_speed_based", encoding="utf-8"))
    self.param_read_counter += 1

    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

    if self._dp_lat_lane_priority_mode:
      self.LP.parse_model(md)
      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    else:
      lane_change_prob = self.l_lane_change_prob + self.r_lane_change_prob

    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob,
                   self._dp_lat_lane_change_assist_speed, md.roadEdges)

    if self._dp_lat_lane_priority_mode:
      d_path_xyz = self._get_laneless_laneline_d_path_xyz()
    else:
      d_path_xyz = self.path_xyz
    d_path_xyz = self._apply_traffic_clearance(d_path_xyz, sm)
    self._d_path_w_lines_xyz = d_path_xyz

    path_distances = np.linalg.norm(d_path_xyz, axis=1)
    model_path_distances = np.linalg.norm(self.path_xyz, axis=1)
    query_distances = self.v_ego * self.t_idxs[:LAT_MPC_N + 1]
    y_pts = np.interp(query_distances, path_distances, d_path_xyz[:, 1])
    heading_pts = np.interp(query_distances, model_path_distances, self.plan_yaw)
    yaw_rate_pts = np.interp(query_distances, model_path_distances, self.plan_yaw_rate)
    self.y_pts = y_pts

    path_cost, lateral_jerk_cost, steering_rate_cost = self._get_mpc_weights(yaw_rate_pts)
    self.lat_mpc.set_weights(path_cost, LATERAL_MOTION_COST,
                             LATERAL_ACCEL_COST, lateral_jerk_cost,
                             steering_rate_cost)

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = max(0, self.factor1 - (self.factor2 * self.v_ego**2))
    p = np.array([self.v_ego, lateral_factor])
    self.lat_mpc.run(self.x0, p, y_pts, heading_pts, yaw_rate_pts)
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = time.monotonic()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 1e6 or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3] / self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x.item() / self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time
    if self.debug_mode:
      lateralPlan.solverCost = self.lat_mpc.cost
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.lat_mpc.x_sol.tolist()
      lateralPlan.solverState.u = self.lat_mpc.u_sol.flatten().tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self._dp_lat_lane_priority_mode and self._dp_lat_lane_priority_mode_active
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction
    pm.send('lateralPlan', plan_send)

    plan_ext_send = messaging.new_message('lateralPlanExt')
    lateralPlanExt = plan_ext_send.lateralPlanExt
    lateralPlanExt.dPathWLinesX = [float(x) for x in self._d_path_w_lines_xyz[:, 0]]
    lateralPlanExt.dPathWLinesY = [float(y) for y in self._d_path_w_lines_xyz[:, 1]]
    pm.send('lateralPlanExt', plan_ext_send)

    if self.debug_mode:
      t = time.monotonic()
      if t > self.last_ovt_log_t + 5.0 and abs(self._traffic_offset) > 1e-4:
        self.last_ovt_log_t = t
        cloudlog.warning(f"[OVT] traffic_offset={self._traffic_offset:+.3f}m "
                         f"adjacent_x={self._adjacent_lead_x:+.1f}m")

  def _apply_path_offset(self, path_xyz):
    # global lateral path shift; negative = left (away from right lane edge)
    if abs(self._dp_path_offset) < 1e-6:
      return path_xyz
    shifted = np.array(path_xyz, copy=True)
    shifted[:, 1] += self._dp_path_offset
    return shifted

  def _compute_traffic_offset(self, sm, d_path_xyz):
    """计算超车远离的目标偏移量。

    返回 (target_offset, lead_x_eff, detected)。openpilot 坐标系 y 正 = 左：
    旁车在左 (lead_y>0) → target<0 → 路径往右挪，远离旁车。

    数据源：modelV2.leadsV3（主源，带 score 门控）→ radarState.leadTwo（兜底）。
    """
    md = sm['modelV2']
    best = None
    for lead in md.leadsV3:
      if lead.score < OVERTAKE_SCORE_MIN:
        continue
      ay = float(lead.y)
      ax = float(lead.x)
      if OVERTAKE_Y_DETECT < abs(ay) < OVERTAKE_Y_MAX and \
         -OVERTAKE_WINDOW_REAR < ax < OVERTAKE_WINDOW_FRONT:
        # 视觉 velocity 是绝对速度 → vRel = v_lead - v_ego
        if best is None or abs(ax) < abs(best[0]):
          best = (ax, ay, float(lead.velocity) - self.v_ego)

    if best is None:  # 兜底：雷达 adjacent lead（vRel 直接是相对速度）
      lt = sm['radarState'].leadTwo
      if lt.status:
        ay = float(lt.yRel)
        ax = float(lt.dRel)
        if OVERTAKE_Y_DETECT < abs(ay) < OVERTAKE_Y_MAX and \
           -OVERTAKE_WINDOW_REAR < ax < OVERTAKE_WINDOW_FRONT:
          best = (ax, ay, float(lt.vRel))

    if best is None:
      return 0.0, 0.0, False

    lead_x, lead_y, v_rel = best
    v_rel = float(np.clip(v_rel, -OVERTAKE_VREL_CLAMP, OVERTAKE_VREL_CLAMP))

    # 速度前瞻：预测 OVERTAKE_LOOKAHEAD_T 秒后的相对距离作鼓包中心，
    # 补偿下游低通的建立滞后（旁车更快 → 中心前移，更慢 → 后移）
    lead_x_eff = lead_x + v_rel * OVERTAKE_LOOKAHEAD_T

    # 横向邻近度：正旁边躲最多；旁车切入 (|y| 逼近下限) 时平滑衰减，无阶跃
    prox = interp(abs(lead_y),
                  [OVERTAKE_Y_DETECT, OVERTAKE_Y_FULL, 2.0, 3.0, OVERTAKE_Y_MAX],
                  [0.3, 1.0, 0.7, 0.3, 0.0])
    target = -np.sign(lead_y) * OVERTAKE_MAX_OFFSET * prox

    # 护栏：偏移不能顶出本车道。车道线取自 modelV2（两种路径模式都可用）
    if len(md.laneLines) == 4:
      ll = md.laneLines[1]
      rl = md.laneLines[2]
      if len(ll.y) == TRAJECTORY_SIZE and len(rl.y) == TRAJECTORY_SIZE and len(ll.x) == TRAJECTORY_SIZE:
        xs_ll = np.array(ll.x)
        ll_y = np.array(ll.y)
        rl_y = np.array(rl.y)
        i = int(np.clip(np.searchsorted(xs_ll, lead_x_eff), 0, TRAJECTORY_SIZE - 1))
        # 当前路径在该距离处的横向位置（兼容已有 _dp_path_offset）
        y0 = float(np.interp(lead_x_eff, d_path_xyz[:, 0], d_path_xyz[:, 1]))
        lo = float(rl_y[i]) + OVERTAKE_MARGIN - y0   # 右移下限
        hi = float(ll_y[i]) - OVERTAKE_MARGIN - y0   # 左移上限
        target = float(np.clip(target, lo, hi))

    return target, lead_x_eff, True

  def _apply_traffic_clearance(self, d_path_xyz, sm):
    """超车远离偏移：旁车道有车时，期望路径向远侧鼓出平滑偏移，超完自然回正。

    注入点在两种路径模式合并之后、MPC 之前；OVERTAKE_ENABLE=False 时
    原样返回，行为与改动前完全一致。
    """
    if not OVERTAKE_ENABLE:
      return d_path_xyz

    in_lane_change = self.DH.desire in (log.LateralPlan.Desire.laneChangeLeft,
                                        log.LateralPlan.Desire.laneChangeRight)
    lat_accel = abs(sm['controlsState'].curvature) * self.v_ego ** 2

    if self.v_ego < OVERTAKE_MIN_SPEED or in_lane_change or lat_accel > OVERTAKE_MAX_LAT_ACCEL:
      target, lead_x_eff, detected = 0.0, 0.0, False
    else:
      target, lead_x_eff, detected = self._compute_traffic_offset(sm, d_path_xyz)

    # 连续帧门控：抗单帧误检
    if detected:
      self._overtake_confirm_frames = min(self._overtake_confirm_frames + 1, OVERTAKE_CONFIRM_FRAMES)
    else:
      self._overtake_confirm_frames = 0
    if self._overtake_confirm_frames < OVERTAKE_CONFIRM_FRAMES:
      target = 0.0

    # 一阶低通：抗目标抖动，保证输出无阶跃
    alpha = DT_MDL / (OVERTAKE_TAU + DT_MDL)
    self._traffic_offset += alpha * (target - self._traffic_offset)
    self._adjacent_lead_x = lead_x_eff

    if abs(self._traffic_offset) < 1e-4:
      return d_path_xyz

    out = np.array(d_path_xyz, copy=True)
    xs = out[:, 0]
    w = np.clip(1.0 - np.abs(xs - lead_x_eff) / OVERTAKE_HALF_WIDTH, 0.0, 1.0)
    w = 0.5 * (1.0 + np.cos(np.pi * (1.0 - w)))   # 升余弦鼓包，比三角更平滑
    out[:, 1] += self._traffic_offset * w
    return out

  def _get_laneless_laneline_d_path_xyz(self):
    if self._dp_lat_lane_priority_mode and self.LP is not None:
      if self.DH.desire in (log.LateralPlan.Desire.laneChangeRight,
                            log.LateralPlan.Desire.laneChangeLeft):
        self.LP.lll_prob *= self.DH.lane_change_ll_prob
        self.LP.rll_prob *= self.DH.lane_change_ll_prob

      if (self.LP.lll_prob + self.LP.rll_prob) / 2 < 0.3:
        self._dp_lat_lane_priority_mode_active = False
      if (self.LP.lll_prob + self.LP.rll_prob) / 2 > 0.5:
        self._dp_lat_lane_priority_mode_active = True

      if (self._dp_lat_lane_priority_mode_active and
          self._dp_lat_lane_priority_mode_speed_based > 0 and
          self.v_ego * 3.6 < self._dp_lat_lane_priority_mode_speed_based):
        self._dp_lat_lane_priority_mode_active = False

      if self._dp_lat_lane_priority_mode_active != self._dp_lat_lane_priority_mode_active_prev:
        self.reset_mpc()
      self._dp_lat_lane_priority_mode_active_prev = self._dp_lat_lane_priority_mode_active

      if not self._dp_lat_lane_priority_mode_active:
        return self._apply_path_offset(self.path_xyz)
      return self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
    return self._apply_path_offset(self.path_xyz)
