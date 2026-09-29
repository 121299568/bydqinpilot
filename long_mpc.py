#!/usr/bin/env python3
import os
import time
from types import SimpleNamespace
import numpy as np
from cereal import log
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL
from openpilot.system.swaglog import cloudlog
from openpilot.selfdrive.hybrid_modeld.constants import index_function
from openpilot.selfdrive.car.interfaces import ACCEL_MIN
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']
X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 5.0  # [C2-OSC-FIX] 原 3.0：距离误差驱动力太弱(相对 A_CHANGE 220)，MPC 在目标距离附近不修正，导致加速-点刹极限环；提至 5.0 增强距离维持
X_EGO_COST = 0.0
V_EGO_COST = 0.0
A_EGO_COST = 0.0
J_EGO_COST = 5.0
A_CHANGE_COST = 200.0
DANGER_ZONE_COST = 100.0  # [C2-OSC-FIX] 原 130：危险区软约束过陡，接近目标距离即被强推重刹；降至 100 让刹车柔和、减少点刹感
CRASH_DISTANCE = 0.25
LEAD_DANGER_FACTOR = 0.65  # [C2-OSC-FIX] 原 0.75：危险区边界太贴目标距离(75%)，平衡点附近反复穿越边界触发刹车；降至 0.65 拉开边界与平衡点距离，减少穿越振荡（仍比原版 0.3 激进）
# [C2-OSC-FIX2] 前车信号瞬时丢失保持窗口(s)：雷达 leadOne.status 实测 1.3s 内可翻转 11 次，
# 裸用 status 时丢一帧 process_lead 即回退 50m/v+10 假目标(障碍物等效跳到 300m+)，
# MPC 当帧全力给油、下一帧前车回来又重刹 —— 即"一脚油门一脚刹车"循环的主因。
# 窗口内沿用最后有效前车外推(多刹一点属安全方向)；真实切出/并线最多多保持 0.6s 车距。
LEAD_HOLD_T = 0.6
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Runtime blended costs do not require regenerating the solver. These values
# add meaningful damping to ordinary model/cruise acceleration changes while
# leaving all hard bounds, danger-zone constraints and FCW logic unchanged.
BLENDED_A_CHANGE_COST = 65.0
BLENDED_JERK_COST = 1.5

N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N + 1)]
T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.0])
COMFORT_BRAKE = 1.5
STOP_DISTANCE = 4
LOW_SPEED_STOP_EXTRA = 3.5  # [C2-STOP-GAP] 原 1.0：停车间距太近(约半个车身)，低速/静止段额外 +3.5m
LOW_SPEED_STOP_FADE_V = 5.0  # [C2-LONG-SMOOTH] 原 3.0，避免末端突然多要 1m 造成急刹


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.5
  if personality == log.LongitudinalPersonality.standard:
    return 1.25
  if personality == log.LongitudinalPersonality.aggressive:
    return 1.25  # [C2-OSC-FIX] 原 1.1：aggressive 档 A_CHANGE/J 阻尼低于 standard(1.25)，输出抖振抑制最弱；提至与 standard 持平减少油门-刹车抖动
  raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.8
  if personality == log.LongitudinalPersonality.standard:
    return 1.3
  if personality == log.LongitudinalPersonality.aggressive:
    return 0.9
  raise NotImplementedError("Longitudinal personality not supported")


def get_dynamic_follow(v_ego, personality=log.LongitudinalPersonality.standard):
  # Preserve low-speed launch spacing while avoiding the very short baseline
  # gaps at urban and highway speeds. Personality ordering remains unchanged.
  if personality == log.LongitudinalPersonality.relaxed:
    x_vel = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0, 25.0]
    y_dist = [2.6, 2.4, 2.3, 2.1, 2.0, 1.9, 1.8]
  elif personality == log.LongitudinalPersonality.standard:
    x_vel = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0]
    y_dist = [2.2, 2.0, 1.9, 1.7, 1.6, 1.6]  # 精调0923: standard 25m/s段 1.5->1.6, 高速留余量
  elif personality == log.LongitudinalPersonality.aggressive:
    x_vel = [0.0, 6.0, 12.0, 18.0, 24.0]
    y_dist = [1.8, 1.7, 1.6, 1.5, 1.4]
  else:
    raise NotImplementedError("Dynamic Follow personality not supported")
  return np.interp(v_ego, x_vel, y_dist)


def get_stopped_equivalence_factor(v_lead):
  return (v_lead ** 2) / (2 * COMFORT_BRAKE)


def get_safe_obstacle_distance(v_ego, t_follow):
  low_speed_extra = LOW_SPEED_STOP_EXTRA * np.clip(1.0 - v_ego / LOW_SPEED_STOP_FADE_V, 0.0, 1.0)
  return (v_ego ** 2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE + low_speed_extra


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)


def get_stopped_equivalence_factor_krkeegen(v_lead, v_ego):
  v_diff_offset = 0
  if np.all(v_lead - v_ego > 0):
    v_diff_offset = v_lead - v_ego
    v_diff_offset = np.clip(v_diff_offset, 0, STOP_DISTANCE / 2)
    v_diff_offset = np.maximum(v_diff_offset * ((10 - v_ego) / 10), 0)
  return (v_lead ** 2) / (2 * COMFORT_BRAKE) + v_diff_offset


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor)
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()
  ocp.dims.N = N
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'
  ocp.cost.W = np.zeros((COST_DIM, COST_DIM))
  ocp.cost.W_e = np.zeros((COST_E_DIM, COST_E_DIM))

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]
  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle, prev_a = ocp.model.p[2], ocp.model.p[3]
  lead_t_follow, lead_danger_factor = ocp.model.p[4], ocp.model.p[5]
  ocp.cost.yref = np.zeros((COST_DIM,))
  ocp.cost.yref_e = np.zeros((COST_E_DIM,))
  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)
  costs = [((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.0),
           x_ego, v_ego, a_ego, a_ego - prev_a, j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])
  ocp.model.con_h_expr = vertcat(v_ego,
                                 a_ego - a_min,
                                 a_max - a_ego,
                                 ((x_obstacle - x_ego) - lead_danger_factor * desired_dist_comfort) / (v_ego + 10.0))
  ocp.constraints.x0 = np.zeros(X_DIM)
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights
  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3
  ocp.solver_options.tf = T_IDXS[-1]
  ocp.solver_options.shooting_nodes = T_IDXS
  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc'):
    self.mode = mode
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

  def reset(self):
    self.solver.reset()
    self.v_solution = np.zeros(N + 1)
    self.a_solution = np.zeros(N + 1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N + 1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N + 1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.params = np.zeros((N + 1, PARAM_DIM))
    for i in range(N + 1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self._lead0_hold = None      # [C2-OSC-FIX2] 前车保持快照 [dRel, vLead]
    self._lead0_hold_age = 0.0   # [C2-OSC-FIX2] 丢失持续时长(s)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4, 4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True,
                  personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0.0
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST,
                      A_EGO_COST, jerk_factor * a_change_cost,
                      jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = BLENDED_A_CHANGE_COST if prev_accel_constraint else 0.0
      cost_weights = [0.0, 0.1, 0.2, 5.0, a_change_cost, BLENDED_JERK_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.0:
      for i in range(N + 1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS ** 2) / 2.0)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    return np.column_stack((x_lead_traj, v_lead_traj))

  def _lead0_with_hold(self, lead):
    # [C2-OSC-FIX2] 前车信号瞬时丢失保持（防"油门-刹车循环"核心修复）。
    # leadOne.status 抖动丢失时，不立刻回退"无前车"假目标，而是在 LEAD_HOLD_T 窗口内
    # 沿用最后一次有效前车：dRel 按相对速度积分外推、aLead 归零（不向危险方向放大），
    # 窗口过后才回退无前车。保持方向是"多刹一点"（保守侧），防丢帧给油。
    if lead is not None and lead.status:
      self._lead0_hold = [float(lead.dRel), float(lead.vLead)]
      self._lead0_hold_age = 0.0
      return lead
    if self._lead0_hold is not None and self._lead0_hold_age < LEAD_HOLD_T:
      v_ego = self.x0[1]
      self._lead0_hold[0] = max(self._lead0_hold[0] + (self._lead0_hold[1] - v_ego) * DT_MDL, 0.5)
      self._lead0_hold_age += DT_MDL
      return SimpleNamespace(status=True, dRel=self._lead0_hold[0], vLead=self._lead0_hold[1],
                             aLeadK=0.0, aLeadTau=_LEAD_ACCEL_TAU)
    self._lead0_hold = None
    return lead

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU
    min_x_lead = ((v_ego + v_lead) / 2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10.0, 5.0)
    return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, radarstate, v_cruise, x, v, a, j,
             personality=log.LongitudinalPersonality.standard,
             use_df_tune=False, use_krkeegen_tune=False):
    v_ego = self.x0[1]
    # # [C2-LONG-SMOOTH] 雷达抖动 / df_tune 开关切换会让 t_follow 突变（实测单次导致目标跟车距离
    # 瞬变 2.6m，是减速中"松刹车补油门"的主因）。一阶滤波把跳变摊平到约 1 秒。
    t_follow_raw = get_dynamic_follow(v_ego, personality) if use_df_tune else get_T_FOLLOW(personality)
    if getattr(self, 't_follow_smoothed', None) is None:
      self.t_follow_smoothed = float(t_follow_raw)
    else:
      self.t_follow_smoothed += (float(t_follow_raw) - self.t_follow_smoothed) * 0.05
    t_follow = self.t_follow_smoothed
    # [C2-OSC-FIX2] lead0 走保持通道：丢帧瞬间障碍物不再跳 300m+ 假目标
    lead0 = self._lead0_with_hold(radarstate.leadOne)
    self.status = lead0.status or radarstate.leadTwo.status
    lead_xv_0 = self.process_lead(lead0)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    if use_krkeegen_tune:
      lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor_krkeegen(lead_xv_0[:, 1], v_ego)
      lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor_krkeegen(lead_xv_1[:, 1], v_ego)
    else:
      lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(lead_xv_0[:, 1])
      lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(lead_xv_1[:, 1])

    self.params[:, 0] = ACCEL_MIN
    self.params[:, 1] = self.max_a

    if self.mode == 'acc':
      self.params[:, 5] = LEAD_DANGER_FACTOR
      v_lower = v_ego + T_IDXS * self.cruise_min_a * 1.05
      v_upper = v_ego + T_IDXS * self.max_a * 1.05
      v_cruise_clipped = np.clip(v_cruise * np.ones(N + 1), v_lower, v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0
    elif self.mode == 'blended':
      self.params[:, 5] = 1.0
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)
      self.source = 'e2e' if x_and_cruise[1, 0] < x_and_cruise[1, 1] else 'cruise'
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:, 1] = x
    self.yref[:, 2] = v
    self.yref[:, 3] = a
    self.yref[:, 5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.prev_a)
    self.params[:, 4] = t_follow
    self.run()

    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
        radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0):
        self.source = 'lead0'
      if (any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0) and
          (lead_1_obstacle[0] - lead_0_obstacle[0])):
        self.source = 'lead1'

  def run(self):
    for i in range(N + 1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)
    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])
    for i in range(N + 1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')
    self.v_solution = self.x_sol[:, 1]
    self.a_solution = self.x_sol[:, 2]
    self.j_solution = self.u_sol[:, 0]
    self.prev_a = np.interp(T_IDXS + 0.05, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
