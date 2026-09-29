from cereal import car
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, apply_deadzone
from openpilot.selfdrive.controls.lib.pid import PIDController
from openpilot.selfdrive.hybrid_modeld.constants import ModelConstants

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego, v_target,
                             v_target_1sec, brake_pressed, cruise_standstill, lead_moving=None):
  # 如果车辆有油门踏板拦截器，则忽略巡航停驶状态
  cruise_standstill = cruise_standstill and not CP.enableGasInterceptor
  
  # 修改逻辑，当前车移动时允许自动启动
  # 判断是否应该忽略巡航停驶条件
  should_ignore_standstill = cruise_standstill
  if lead_moving is not None:
    # 如果检测到前车正在移动，不管停驶状态如何都允许启动
    should_ignore_standstill = cruise_standstill and not lead_moving
    # 修正逻辑：如果前车在移动，则忽略停驶状态
    if lead_moving:
      should_ignore_standstill = False

  accelerating = v_target_1sec > v_target
  planned_stop = (v_target < CP.vEgoStopping and
                  v_target_1sec < CP.vEgoStopping and
                  not accelerating)
  stay_stopped = (v_ego < CP.vEgoStopping and
                  (brake_pressed or (cruise_standstill and not should_ignore_standstill)))
  stopping_condition = planned_stop or stay_stopped

  starting_condition = (v_target_1sec > CP.vEgoStarting and
                        accelerating and
                        not should_ignore_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state in (LongCtrlState.off, LongCtrlState.pid):
      long_control_state = LongCtrlState.pid
      if stopping_condition:
        long_control_state = LongCtrlState.stopping

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid
      # 额外条件：如果刹车释放且目标速度为正值，允许重新启动
      elif (not brake_pressed and v_target > CP.vEgoStarting and 
            v_target_1sec > v_target and v_ego <= CP.vEgoStopping):
        long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.starting:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid

  return long_control_state


class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off  # initialized to off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.v_pid = 0.0
    self.last_output_accel = 0.0
    # 添加用于跟踪前车位置的变量
    self.prev_lead_distance = None
    self.distance_history = []  # 记录距离历史，用于过滤噪声
    self.movement_threshold = 0.5  # 距离变化超过0.5米才考虑是真实移动
    self.min_continuous_movement = 1.0  # 累计移动距离需超过1米才认为前车移动

  def reset(self, v_pid):
    """Reset PID controller and change setpoint"""
    self.pid.reset()
    self.v_pid = v_pid

  def update(self, active, CS, long_plan, accel_limits, t_since_plan):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # Interp control trajectory
    speeds = long_plan.speeds
    if len(speeds) == CONTROL_N:
      v_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_now = interp(t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], long_plan.accels)

      v_target_lower = interp(self.CP.longitudinalActuatorDelayLowerBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_lower = 2 * (v_target_lower - v_target_now) / self.CP.longitudinalActuatorDelayLowerBound - a_target_now

      v_target_upper = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan, ModelConstants.T_IDXS[:CONTROL_N], speeds)
      a_target_upper = 2 * (v_target_upper - v_target_now) / self.CP.longitudinalActuatorDelayUpperBound - a_target_now

      v_target = min(v_target_lower, v_target_upper)
      a_target = min(a_target_lower, a_target_upper)

      v_target_1sec = interp(self.CP.longitudinalActuatorDelayUpperBound + t_since_plan + 1.0, ModelConstants.T_IDXS[:CONTROL_N], speeds)
    else:
      v_target = 0.0
      v_target_now = 0.0
      v_target_1sec = 0.0
      a_target = 0.0

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    output_accel = self.last_output_accel
    
    # 判断前车是否移动了至少1米，以帮助处理停车启动情况
    # 增强算法：过滤噪声，识别真实移动
    lead_moving = None
    if hasattr(CS, 'radarState') and hasattr(CS.radarState, 'leadOne'):
        lead = CS.radarState.leadOne
        if lead and lead.dRel > 0:  # 确保前车距离有效
            current_lead_distance = lead.dRel
            
            if self.prev_lead_distance is not None:
                # 计算距离变化量
                distance_change = current_lead_distance - self.prev_lead_distance
                
                # 将距离变化加入历史记录
                self.distance_history.append(distance_change)
                
                # 保持历史记录最多20个数据点（约0.4秒，假设每帧50ms）
                if len(self.distance_history) > 20:
                    self.distance_history.pop(0)
                
                # 计算累计移动距离，但过滤掉小幅度波动
                net_movement = sum(change for change in self.distance_history if abs(change) > self.movement_threshold)
                
                # 如果净移动距离超过阈值，则认为前车开始移动
                lead_moving = abs(net_movement) >= self.min_continuous_movement
            else:
                # 第一次检测，只记录距离
                lead_moving = False
                
            self.prev_lead_distance = current_lead_distance

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       v_target, v_target_1sec, CS.brakePressed,
                                                       CS.cruiseState.standstill, lead_moving)

    if self.long_control_state == LongCtrlState.off:
      self.reset(CS.vEgo)
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset(CS.vEgo)

    elif self.long_control_state == LongCtrlState.pid:
      self.v_pid = v_target_now

      # Toyota starts braking more when it thinks you want to stop
      # Freeze the integrator so we don't accelerate to compensate, and don't allow positive acceleration
      # TODO too complex, needs to be simplified and tested on toyotas
      prevent_overshoot = not self.CP.stoppingControl and CS.vEgo < 1.5 and v_target_1sec < 0.7 and v_target_1sec < self.v_pid
      deadzone = interp(CS.vEgo, self.CP.longitudinalTuning.deadzoneBP, self.CP.longitudinalTuning.deadzoneV)
      freeze_integrator = prevent_overshoot

      error = self.v_pid - CS.vEgo
      error_deadzone = apply_deadzone(error, deadzone)
      output_accel = self.pid.update(error_deadzone, speed=CS.vEgo,
                                     feedforward=a_target,
                                     freeze_integrator=freeze_integrator)

    self.last_output_accel = clip(output_accel, accel_limits[0], accel_limits[1])

    return self.last_output_accel
