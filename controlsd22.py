# 秦专用

import os,math,time
import cereal.messaging as messaging
from typing import SupportsFloat
from cereal import car, log
from openpilot.common.numpy_fast import clip, interp
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
# from openpilot.selfdrive.controls.sumtj import DriveStats
# from openpilot.selfdrive.controls.rainbow import create_rainbow

SOFT_DISABLE_TIME = 3
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ
NOSENSOR = "NOSENSOR" in os.environ
IGNORE_PROCESSES = {"loggerd", "encoderd", "statsd", "mapd", "gpxd"}
# NO_IR_CTRL = Params().get_bool("dp_device_no_ir_ctrl")
# if NO_IR_CTRL:
#   IGNORE_PROCESSES |= {'driverCameraState', 'driverMonitoringState'}
# ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
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
  def _init_state_attrs(self):
    """统一初始化所有控制逻辑所需的状态属性（避免动态初始化和 hasattr 检查）"""
    # ========== 积极加速控制相关 ==========
    self._aggressive_accel_timer = 0
    # ========== 自适应跟车加速度控制相关 ==========
    self._adaptive_accel_enabled = True
    self._lead_status = False
    self._lead_status_timer = 0
    self._last_safe_lead_distance = 0.0
    self._should_resume = False
    # ========== 前车状态跟踪相关 ==========
    self._stopped_lead_detected = False
    self._stopped_lead_timer = 0.0
    self._last_lead_distance = 0.0
    # ========== ALKA 功能相关 ==========
    #self._dp_alka_active = False
    #self._dp_alka_btn_block_frame = 0
    self._alka_pause_timer = 0.0
    self._alka_last_override_time = 0.0
    self._alka_override_count = 0          # 接管次数计数器
    self._alka_override_count_reset_time = 0.0  # 计数器重置时间
    self._alka_consecutive_pause = False    # 是否处于连续暂停状态
    # ========== 车道变换辅助相关 ==========
    self._dp_lat_lane_change_assist_disabled_active = False
    # ========== 实验模式相关 ==========
    self._experimental_mode_start_timer = 0.0
    # ========== 弯道降速相关 ==========
    self._curve_saved_cruise_kph = 0.0
    self._curve_target_kph = None
    self._stock_set_kph = None
  def __init__(self, sm=None, pm=None, can_sock=None, CI=None):
    config_realtime_process(4 if TICI else 3, Priority.CTRL_HIGH)
    self.dp_gps_ok_once = False
    self.branch = get_short_branch("")
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState','carControl', 'carEvents', 'carParams', 'controlsStateExt'])
    # if NO_IR_CTRL:
    #   self.camera_packets = ["roadCameraState"]
    # else:
    #   self.camera_packets = ["roadCameraState", "driverCameraState"]
    can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 20
    self.can_sock = messaging.sub_sock('can', timeout=can_timeout)
    self.log_sock = messaging.sub_sock('androidLog')
    self.params = Params()
    self.dp_no_gps_ctrl = self.params.get_bool("dp_no_gps_ctrl")
    self.dp_no_fan_ctrl = self.params.get_bool("dp_no_fan_ctrl")
    self.dp_0813 = self.params.get_bool("dp_0813")
    self._dp_alka = self.params.get_bool("dp_alka")
    self._dp_alka_active = True
    self._dp_alka_trigger_count = 0
    self._dp_alka_btn_block_frame = 0
    self._dp_lat_lane_change_assist_disabled = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) == 0
    self._dp_lat_lane_change_assist_disabled_active = False
    self.torqued_override = self.params.get_bool("CustomTorqueLateral")

    self._dp_lat_lane_change_assist_disabled = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) == 0
    self.torqued_override = self.params.get_bool("CustomTorqueLateral")
    self.sm = sm
    if self.sm is None:
      ignore = ['testJoystick']
      if SIMULATION:
        ignore += ['driverCameraState', 'managerState']
    #   if NO_IR_CTRL:
        # ignore += ['driverCameraState', 'driverMonitoringState']
      self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration', 'driverMonitoringState', 'longitudinalPlan', 'lateralPlan', 'liveLocationKalman','managerState', 'liveParameters', 'radarState', 'liveTorqueParameters', 'testJoystick'], ignore_alive=ignore, ignore_avg_freq=['radarState', 'testJoystick'])
    if CI is None:
      get_one_can(self.can_sock)

      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
      experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")# and not self.dp_0813 # and not is_release_branch()
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
    else:
      self.CI, self.CP = CI, CI.CP

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
    # self.drive_stats = DriveStats()
    # self.rainbow_colors = create_rainbow()
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
    # ALKA combination
    if self._dp_alka and CS.brakePressed:
      # rick - allow ALKA to be enabled/disabled when brake + main pressed twice in 0.5 secs
      if self.CP.pcmCruise and CS.cruiseState.available != self.CS_prev.cruiseState.available:
        self._dp_alka_trigger_count += 1
      if self._dp_alka_trigger_count == 2:
        self._dp_alka_active = not self._dp_alka_active
      if self.sm.frame % 50 == 0:
        self._dp_alka_trigger_count = 0
      # rick - allow ALKA to be enabled/disabled when brake + set is pressed
      # some HKGs doesnt have main buttons like other cars (e.g. EV6)
      if not self.CP.pcmCruise and self._dp_alka_btn_block_frame < self.sm.frame:
        # set/- is pressed
        if any(be.type in (ButtonType.decelCruise, ButtonType.setCruise) for be in CS.buttonEvents):
          self._dp_alka_active = not self._dp_alka_active
          # block activity for a sec
          self._dp_alka_btn_block_frame = self.sm.frame + 100
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
    # if not self.CP.notCar and not NO_IR_CTRL:
    #   self.events.add_from_msg(self.sm['driverMonitoringState'].events)
    if CS.canValid:
      self.events.add_from_msg(CS.events)
    # if not self.dp_device_disable_temp_check and self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
    #   self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)
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
    
    if len(self.sm['radarState'].radarErrors) or (not self.rk.lagging and not self.sm.all_checks(['radarState'])):
      self.events.add(EventName.radarFault)
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
    # [C2-CURVE-FIX] pcmCruise 车每帧被车机巡航设定速度覆盖
    #   (drive_helpers.py:69  self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH)。
    #   弯道降速写在 state_control 里，若不在此处恢复，降的值下一帧就被抹掉，
    #   弯道减速实际只生效一帧 —— 这正是"弯道减速不起效果"的根因。
    if CS.cruiseState.available:
      stock_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      if self._stock_set_kph is None or abs(stock_kph - self._stock_set_kph) > 0.5:
        # 用户按加/减键改了车机设定：清除弯道目标，避免按键被弯道逻辑顶掉
        self._stock_set_kph = stock_kph
        self._curve_target_kph = None
        self._curve_saved_cruise_kph = 0.0
      elif self._curve_target_kph is not None:
        self.v_cruise_helper.v_cruise_kph = min(self.v_cruise_helper.v_cruise_kph, self._curve_target_kph)
        self.v_cruise_helper.v_cruise_cluster_kph = self.v_cruise_helper.v_cruise_kph
    else:
      self._stock_set_kph = None
      self._curve_target_kph = None
      self._curve_saved_cruise_kph = 0.0
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
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
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
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
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

          initial_speed = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if self.experimental_mode else V_CRUISE_INITIAL
          self.v_cruise_helper.v_cruise_kph = initial_speed
          self.v_cruise_helper.v_cruise_cluster_kph = initial_speed

    self.enabled = self.state in ENABLED_STATES
    self.active = self.state in ACTIVE_STATES
    if self.active or (self._dp_alka and self._dp_alka_active):
      self.current_alert_types.append(ET.WARNING)

  def state_control(self, CS):

    self.a_target_max = 1.8  
    self.a_target_min = -5.0  
    
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)
    v_ego_kph = CS.vEgo * CV.MS_TO_KPH

    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams and not self.torqued_override:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    lat_plan = self.sm['lateralPlan']
    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled
    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state
    standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    driver_override = CS.steeringPressed and (self.active or self._dp_alka_active) and not (
        (CS.leftBlinker and CS.steeringTorque > 0) or 
        (CS.rightBlinker and CS.steeringTorque < 0)
    )
    
    if self._dp_alka_active and driver_override:
      current_time = time.time()
      
      # 重置计数器（每3秒重置一次）
      if current_time - self._alka_override_count_reset_time > 3.0:
        self._alka_override_count = 0
        self._alka_override_count_reset_time = current_time
        self._alka_consecutive_pause = False
      
      # 增加接管计数
      self._alka_override_count += 1
      
      # 智能暂停策略：
      # - 第1次接管：正常暂停0.5秒
      # - 第2-3次接管（3秒内）：延长暂停至1.0秒
      # - 第4次及以上：延长暂停至2.0秒，并标记为连续暂停
      # - 连续暂停期间不再重复触发，直到暂停结束
      if not self._alka_consecutive_pause and current_time - self._alka_last_override_time > 0.3:
        if self._alka_override_count <= 1:
          pause_duration = 0.5
        elif self._alka_override_count <= 3:
          pause_duration = 1.0
        else:
          pause_duration = 2.0
          self._alka_consecutive_pause = True
        
        self._alka_pause_timer = pause_duration
        self._alka_last_override_time = current_time
    
    if self._alka_pause_timer > 0:
      self._alka_pause_timer -= DT_CTRL
      if self._alka_pause_timer <= 0:
        self._alka_pause_timer = 0.0
        self._alka_consecutive_pause = False  # 暂停结束，清除连续标记
    
    alka_paused = self._dp_alka_active and self._alka_pause_timer > 0
    
    recent_steering_pressed_short = (self.sm.frame - self.last_steering_pressed_frame) * DT_CTRL < 0.5
    lane_center_tolerance = 1.2
    vehicle_is_centered = abs(self.sm['lateralPlan'].dPathPoints[0]) < lane_center_tolerance if len(self.sm['lateralPlan'].dPathPoints) > 0 else True
    lane_change_in_progress = False
    if hasattr(self, 'DH'):
      lane_change_in_progress = self.DH.lane_change_state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    CC.latActive = self.active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode) and \
                   (not driver_override or lane_change_in_progress) and not recent_steering_pressed_short and vehicle_is_centered and not alka_paused
    CC.longActive = self.enabled and not self.events.contains(ET.OVERRIDE_LONGITUDINAL) and self.CP.openpilotLongitudinalControl

    # 提前初始化 use_stock_acc：原先只在 if self.enabled 内赋值、外部使用，
    # 依赖 CC.longActive=False 的短路求值才不触发 NameError，极其脆弱
    use_stock_acc = True
    if self.enabled:
      v_ego_prev_kph = self.CS_prev.vEgo * CV.MS_TO_KPH
      is_decelerating = v_ego_kph < v_ego_prev_kph - 1.0
      if is_decelerating:
          use_stock_acc = 3.5 < v_ego_kph
      else:
          use_stock_acc = 10.0 < v_ego_kph

      lead_one = self.sm['radarState'].leadOne    
       
      no_lead_vehicle = not (lead_one.status and lead_one.dRel < 30.0)
 
      # ========== 弯道检测 ==========
      if hasattr(model_v2, 'orientationRate') and len(model_v2.orientationRate.z) > 0:
        omega_z = model_v2.orientationRate.z[0]
        if v_ego_kph > 25.0:
          curvature = abs(omega_z / CS.vEgo)
          thresholds = [0.0015, 0.002, 0.005, 0.008]
          # 极简策略计算
          prev_ratio = getattr(self, '_prev_speed_ratio', 1.0)
          target_ratio = 1.0
          if curvature < thresholds[0]:
              target_ratio = 1.0
          elif curvature < thresholds[1]:
              if prev_ratio < 0.95:
                  target_ratio = prev_ratio
              else:
                  t = (curvature - thresholds[0]) / (thresholds[1] - thresholds[0])
                  target_ratio = 1.0 - ((1 - math.cos(math.pi * t)) / 2) * 0.05
          elif curvature < thresholds[2]:
              t = (curvature - thresholds[1]) / (thresholds[2] - thresholds[1])
              target_ratio = 1.0 - ((1 - math.cos(math.pi * t)) / 2) * 0.20
          elif curvature < thresholds[3]:
              t = (curvature - thresholds[2]) / (thresholds[3] - thresholds[2])
              target_ratio = 0.80 - ((1 - math.cos(math.pi * t)) / 2) * 0.25
          else:
              target_ratio = 0.55
          # [C2-CURVE-FIX] 目标速度必须基于"弯前原始设定速度"，不能用当前 v_cruise。
          # 否则 target = v_cruise * ratio < v_cruise 恒成立，速度会一路降到停车。
          # （此前因 v_cruise 每帧被车机覆盖，该缺陷被掩盖；修覆盖后会立即暴露）
          _curve_base_kph = self._curve_saved_cruise_kph if self._curve_saved_cruise_kph > 0.0 else self.v_cruise_helper.v_cruise_kph
          target_speed_kph = _curve_base_kph * target_ratio
          # 极简控制
          if curvature >= thresholds[0]:
            self.a_target_max = min(self.a_target_max, 0.3)
            self.a_target_min = max(self.a_target_min, -1.5)
            use_stock_acc = False
            if self.v_cruise_helper.v_cruise_kph > target_speed_kph:
                # 记录弯道前的原始设定速度，出弯后用于恢复
                if self._curve_saved_cruise_kph < self.v_cruise_helper.v_cruise_kph:
                  self._curve_saved_cruise_kph = self.v_cruise_helper.v_cruise_kph
                self.v_cruise_helper.v_cruise_kph = max(self.v_cruise_helper.v_cruise_kph - 3.0 * DT_CTRL, target_speed_kph)
                self._curve_target_kph = self.v_cruise_helper.v_cruise_kph  # 锁存，供下一帧恢复
                # 同步 HUD 显示值，避免仪表速度与实际控制速度不一致
                self.v_cruise_helper.v_cruise_cluster_kph = self.v_cruise_helper.v_cruise_kph
          elif self._curve_saved_cruise_kph > 0.0:
              # 出弯恢复：向弯前原始设定速度恢复。
              # 原来的 +0.2 km/h/s 恢复分支不可达（目标值由当前值推导，恒不触发），
              # 导致降过的设定速度永远回不去；现改为 2.0 km/h/s 向保存值恢复
              self.v_cruise_helper.v_cruise_kph = min(self.v_cruise_helper.v_cruise_kph + 5.0 * DT_CTRL, self._curve_saved_cruise_kph)
              self._curve_target_kph = self.v_cruise_helper.v_cruise_kph
              self.v_cruise_helper.v_cruise_cluster_kph = self.v_cruise_helper.v_cruise_kph
              if self.v_cruise_helper.v_cruise_kph >= self._curve_saved_cruise_kph:
                self._curve_saved_cruise_kph = 0.0
                self._curve_target_kph = None

      # 基于雷达数据的前车状态检测与实验模式控制逻辑
      current_lead_status = False
      # 关键修复：必须将前车真实速度赋给 v_lead_kph，否则下面的静止/低速前车判断永远为 False
      v_lead_kph = lead_one.vLead * CV.MS_TO_KPH if lead_one.status else float('inf')
      distance_threshold = interp(v_ego_kph, [0, 15, 40, 80, 120], [1, 5, 15, 40, 60]) #左边车速右边距离
      danger_distance_threshold = distance_threshold * 0.8
      if lead_one.status:
          if v_lead_kph < 5 and lead_one.dRel < distance_threshold:
            current_lead_status = True
            self._experimental_mode_start_timer += DT_CTRL
            if v_ego_kph > 15:
              self.experimental_mode = True
          lead_decel_threshold = 1.8
          if lead_one.dRel < distance_threshold and lead_one.aLeadK < -lead_decel_threshold:
            current_lead_status = True
            self._experimental_mode_start_timer += DT_CTRL
          if lead_one.dRel < danger_distance_threshold:
            current_lead_status = True
            self._experimental_mode_start_timer += DT_CTRL
            if not self.experimental_mode and self._experimental_mode_start_timer >= 1.0:
              self.experimental_mode = True
          if v_lead_kph < 0.2:
            if lead_one.dRel < 60.0:
              current_lead_status = True
              self._stopped_lead_detected = True
              relative_speed_kph = abs(v_ego_kph - v_lead_kph)
              use_stock_acc = False
              self.a_target_max = min(self.a_target_max, 0.0)  # 禁止加速
              # 制动上限随距离动态放开：按当前车速和距离计算"刹停所需减速度"（预留5m余量），
              # 远处只允许缓刹，越近放开越多，确保任何车速下都能刹停。
              # 注意：此处直接覆盖而非叠加收紧——弯道逻辑可能已把 a_target_min 收到 -1.5，
              # 静止前车必须按刹停需要放开制动上限，否则弯道中遇静止车刹不住
              a_stop_required = (CS.vEgo ** 2) / (2.0 * max(lead_one.dRel - 5.0, 2.0))
              self.a_target_min = max(-5.0, -min(5.0, max(0.8, a_stop_required)))
              if relative_speed_kph > 10.0:
                # 正在接近静止前车：60米即开始缓慢减速，距离越近减速指令越强
                if lead_one.dRel >= 40.0:
                  self.a_target_max = min(self.a_target_max, -0.4)  # 40~60m：缓慢减速
                elif lead_one.dRel >= 25.0:
                  self.a_target_max = min(self.a_target_max, -0.7)  # 25~40m：中等减速
                else:
                  self.a_target_max = min(self.a_target_max, -1.2)  # 25m内：强制减速
              else:
                # 低速接近静止前车时，确保不会加速
                self.a_target_max = min(self.a_target_max, 0.0)
                # 延长停止检测时间，确保稳定停车
                if self._stopped_lead_timer < 2.0:
                  self._stopped_lead_timer += DT_CTRL
                else:
                  # 在完全停止前使用更保守的加速度限制
                  if v_ego_kph < 3.0:  # 低于3km/h时特别保守
                    self.a_target_max = min(self.a_target_max, 0.0)  # 完全禁止加速
                    self.a_target_min = max(self.a_target_min, -0.3)
          else:
            self._stopped_lead_timer = 0.0
            self.experimental_mode = False
            if not self._stopped_lead_detected:
              self._stopped_lead_detected = False
      # 前车起步检测和蠕行跟车逻辑
      if lead_one.status and v_ego_kph < 0.1:
        current_distance = lead_one.dRel
        if v_ego_kph < 0.1 and lead_one.vLead < 0.1:
            self._last_lead_distance = lead_one.dRel
            self.experimental_mode = False
            self.v_cruise_helper.v_cruise_kph += 5
            self.v_cruise_helper.v_cruise_kph -= 5
            use_stock_acc = True
        if self._last_lead_distance > 0:
          distance_moved_since_stop = current_distance - self._last_lead_distance
          if distance_moved_since_stop >= 0.5 or lead_one.vLead > 0.1:
            current_lead_status = True
            self.experimental_mode = True
            self._aggressive_accel_timer = 2.0 / DT_CTRL
            self._should_resume = True
            use_stock_acc = False
      # ========== 基于视觉模型的前车状态检测 ==========
      if len(model_v2.leadsV3) > 0:
        lead_v3 = model_v2.leadsV3[0]
        if lead_v3.prob > 0.5:
          if len(lead_v3.v) > 0:
            v_vision_lead_kph = lead_v3.v[0] * CV.MS_TO_KPH
            relative_vision_speed_kph = v_ego_kph - v_vision_lead_kph
            if (v_vision_lead_kph > 1.0 and v_ego_kph < 0.1) or relative_vision_speed_kph > 20.0 or v_vision_lead_kph < 0.2:
              self.experimental_mode = True
              current_lead_status = True
              use_stock_acc = False
            else:
              self.experimental_mode = False
      # 前车状态防抖
      # [C2-STOCK-ACC-FIX] 原实现有两个问题：
      #   1) 去抖固定 0.3s —— 视觉前车瞬时丢失就会把纵向交还原车 ACC
      #   2) 更关键：去抖**等待期内**没有保持 openpilot 接管，use_stock_acc 仍是
      #      前面按车速算出的 True，等于去抖计时一开始就已经交还了，防抖形同虚设。
      # 原车 ACC 看不到 openpilot 的视觉前车，一交还就补油门，再接管又急刹 ——
      # 这正是"减速中途松刹车来一脚油门"的直接原因。
      # 改为：确认有车快(0.1s)保证安全，确认丢车慢(1.5s)避免误交还，
      #       且等待期内继续保持 openpilot 接管。
      if current_lead_status != self._lead_status:
        self._lead_status_timer += DT_CTRL
        debounce_s = 1.5 if self._lead_status else 0.1
        if self._lead_status_timer >= debounce_s:
          self._lead_status = current_lead_status
          self._lead_status_timer = 0
          if current_lead_status:
            use_stock_acc = False
        elif self._lead_status:
          # 丢车去抖等待期：继续由 openpilot 接管，不能提前交还原车 ACC
          use_stock_acc = False
      else:
        self._lead_status_timer = 0
        if self._lead_status and v_ego_kph > 0.1:
          # 修复：移动中且前车状态稳定时必须保持 openpilot 纵向控制。
          # 原代码此处把 use_stock_acc 改回 True，导致静止前车被连续识别超过
          # 0.3 秒后纵向控制交还原车 ACC（对静止目标不刹车）。
          # 静止（<0.1km/h）时不干预，保留原车蠕行/驻车逻辑。
          use_stock_acc = False

    # self.CP.openpilotLongitudinalControl: #车辆是否支持openpilot纵向控制
    if self.CP.openpilotLongitudinalControl:
      CC.longActive = CC.longActive and not use_stock_acc
      
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

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions
    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      t_since_plan = (self.sm.frame - self.sm.rcv_frame['longitudinalPlan']) * DT_CTRL
      actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)
      # 积极加速控制逻辑
      lead_one = self.sm['radarState'].leadOne
      if self._aggressive_accel_timer > 0 and v_ego_kph < 0.3 and lead_one.vLead > 0.1:
        speed_diff = lead_one.vLead - CS.vEgo
        if speed_diff > 0:
          distance_factor = min(lead_one.dRel / 3.0, 4.8) if lead_one.dRel > 0 else 4.8
          # 显式钳制目标加速度（原公式可计算出数十 m/s²，仅靠 a_target_max=1.8 兜底）
          target_accel = clip(max(speed_diff * 2.8, pid_accel_limits[1] * 0.4) * distance_factor, 0.0, 1.8)
          actuators.accel = max(actuators.accel, target_accel)
        self._aggressive_accel_timer -= DT_CTRL
      """
      # 自适应跟车加速度控制逻辑（逻辑 2）
      if not self.is_comfort_mode():
        if self._adaptive_accel_enabled and lead_one.status:
          if lead_one.dRel > 30.0 and v_ego_kph > 60.0 and lead_one.vLead > 0.5:
            distance, lead_speed, ego_speed = lead_one.dRel, lead_one.vLead, CS.vEgo
            speed_diff = ego_speed - lead_speed
            ideal_dist = 1.85 * ego_speed + 4.0  
            deviation = ideal_dist - distance
            base_accel = 0.8 if ego_speed < 10.0 else (0.5 if ego_speed < 20.0 else 0.3)
            actuators.accel = max(actuators.accel, base_accel)
            if deviation > 0 and speed_diff > 0:
              intensity = min(deviation/15.0, 1.0) * min(speed_diff/6.0, 1.0)
              target_decel = max(pid_accel_limits[0], -2.5) * intensity
              actuators.accel = min(actuators.accel, target_decel)
            elif deviation < -3.0 and speed_diff < -0.5:
              intensity = min(abs(deviation)/20.0, 1.0) * min(abs(speed_diff)/4.0, 1.0)
              distance_bonus = min(distance/60.0, 0.3)
              final_intensity = min(intensity + distance_bonus, 1.0)
              target_accel = min(pid_accel_limits[1], 1.8) * final_intensity
              actuators.accel = max(actuators.accel, target_accel)
        # 智能跟车加速度控制与前车状态确认逻辑（逻辑 3）
        if v_ego_kph > 60.0 and CC.longActive and hasattr(lead_one, 'vLead') and lead_one.vLead > 0.5 and lead_one.dRel > 30.0:
          v_ego, v_lead, distance, v_rel = CS.vEgo, lead_one.vLead, lead_one.dRel, lead_one.vRel
          speed_diff, approaching, distance_safe = v_lead - v_ego, v_rel > 0.2, distance > 25.0
          stable = (distance - self._last_safe_lead_distance) >= -0.03
          self._last_safe_lead_distance = distance
          if speed_diff > 0.3 and not approaching and distance_safe and stable:
            dist_factor = max(0.05, min(0.8, (distance - 25.0) / 75.0 * 0.75 + 0.05))
            target_accel = dist_factor * 0.5 + speed_diff * 0.5
            max_accel = 1.2 if v_ego < 5.0 else (0.9 if v_ego < 15.0 else 0.6)
            actuators.accel = max(actuators.accel, max(0.0, min(target_accel, max_accel, pid_accel_limits[1])))
            self._should_resume = True
          else:
            self._should_resume = False
      """

      if hasattr(self, 'a_target_max'):
        actuators.accel = min(actuators.accel, self.a_target_max)
      if hasattr(self, 'a_target_min'):
        actuators.accel = max(actuators.accel, self.a_target_min)
    
      # Steering PID loop
      self.desired_curvature, self.desired_curvature_rate = get_lag_adjusted_curvature(self.CP, CS.vEgo,lat_plan.psis,lat_plan.curvatures,lat_plan.curvatureRates)
      lat_tuning = self.CP.lateralTuning.which()
      if lat_tuning == 'torque':
        actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,self.last_actuators, self.steer_limited, self.desired_curvature,self.desired_curvature_rate, self.sm['liveLocationKalman'], model_data=model_v2)
      else:
        actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,self.last_actuators, self.steer_limited, self.desired_curvature,self.desired_curvature_rate, self.sm['liveLocationKalman'])
      actuators.curvature = self.desired_curvature
    else:
      # 修复：joystick 模式下 lac_log 未定义，下方 lac_log.active 访问会 NameError（上游此处有该分支）
      lac_log = log.ControlsState.LateralControlState.new_message()

    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0
 
    if lac_log.active and not recent_steer_pressed and not self.CP.notCar:
      if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
        undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 2.2
        turning = abs(lac_log.desiredLateralAccel) > 2.99
        good_speed = CS.vEgo > 5
        max_torque = abs(self.last_actuators.steer) > 2.89
        if undershooting and turning and good_speed and max_torque:
          lac_log.active and self.events.add(EventName.steerSaturated)

    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue
      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)
       
    return CC, lac_log
  def publish_logs(self, CS, start_time, CC, lac_log):
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
    speeds = self.sm['longitudinalPlan'].speeds
    accels = self.sm['longitudinalPlan'].accels
    if len(speeds):
      CC.cruiseControl.resume = self.enabled and CS.cruiseState.standstill and speeds[-1] > 0.05

    hudControl = CC.hudControl  
    hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_cluster_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled  
    hudControl.lanesVisible = self.enabled  
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead  
    hudControl.rightLaneVisible = True  
    hudControl.leftLaneVisible = True  

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
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))
      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

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
    force_decel = (self.state == State.softDisabling)

    force_decel = self.state == State.softDisabling
    lp = self.sm['liveParameters']
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)
    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertSound = current_alert.audible_alert
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
  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()
      
  def is_comfort_mode(self):
    """获取当前是否为舒适模式"""
    personality_param = Params().get("LongitudinalPersonality")
    return personality_param is not None and int(personality_param) == log.LongitudinalPersonality.relaxed
      
def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()
if __name__ == "__main__":
  main()