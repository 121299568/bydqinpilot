from cereal import log
# from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

# LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_COOLDOWN = 5.0
BLINKER_HOLD_TIME = 1.0
ROAD_EDGE_LANE_CHANGE_MARGIN = 3.5

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    self.blinker_hold_timer = 0.0
    self.lane_change_cooldown_timer = 0.0

  @staticmethod
  def _road_edge_blocked(road_edges, direction):
    if road_edges is None or len(road_edges) != 2:
      return False

    edge_idx = 0 if direction == LaneChangeDirection.left else 1
    edge = road_edges[edge_idx]
    if len(edge.y) == 0:
      return False

    # Match dragonpilot beta2: use the road edge nearest the vehicle.
    edge_y = float(edge.y[0])
    distance = -edge_y if direction == LaneChangeDirection.left else edge_y
    return distance == distance and distance < ROAD_EDGE_LANE_CHANGE_MARGIN

  def update(self, carstate, lateral_active, lane_change_prob, dp_lat_lane_change_assist_speed,
             road_edges=None):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < dp_lat_lane_change_assist_speed if dp_lat_lane_change_assist_speed > 0 else True

    self.lane_change_cooldown_timer = max(self.lane_change_cooldown_timer - DT_MDL, 0.0)

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.blinker_hold_timer = 0.0
    elif not one_blinker and self.lane_change_state in (LaneChangeState.laneChangeStarting,
                                                         LaneChangeState.laneChangeFinishing):
      # Releasing the turn signal cancels an in-progress lane change.
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.blinker_hold_timer = 0.0
      self.lane_change_ll_prob = 1.0
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.blinker_hold_timer = 0.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        at_boundary = self._road_edge_blocked(road_edges, self.lane_change_direction)

        if self.lane_change_cooldown_timer > 0:
          at_boundary = True

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.blinker_hold_timer = 0.0
        elif blindspot_detected or at_boundary:
          self.blinker_hold_timer = 0.0
        else:
          self.blinker_hold_timer += DT_MDL
          if self.blinker_hold_timer >= BLINKER_HOLD_TIME:
            self.lane_change_state = LaneChangeState.laneChangeStarting
            self.blinker_hold_timer = 0.0

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_cooldown_timer = LANE_CHANGE_COOLDOWN
          self.lane_change_direction = LaneChangeDirection.none
          # Require the blinker to be released before another lane change.
          self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none
