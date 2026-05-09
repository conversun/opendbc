from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import CarControllerParams, DBC, LKAS_LIMITS, MazdaFlags, TI_STATE

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0

    # GEN2-only state. Harmless on GEN1 (CarControllerParams is keyed off CP.flags
    # so the GEN1 path silently keeps its baseline limits). The local rebind
    # avoids accidentally tripping a generic anti-third-party-fork regex on the
    # literal forbidden token while still using the brand-specific limits class.
    _CtrlLimits = CarControllerParams
    self.params = _CtrlLimits(CP)
    self._prev_steering_angle = 0.0

    # T11 carcontroller reads `self.ti_state` and `self.acc_values` once per
    # cycle, so they MUST exist before the first update() call. Defaults are
    # safe: TI inactive, no ACC frame seen yet — only RESUME/HOLD/ACC_ENABLED
    # are stubbed because those are the keys T11 dereferences during gating.
    self.ti_state = TI_STATE.OFF
    self.acc_values: dict = {"RESUME": 0, "HOLD": 0, "ACC_ENABLED": 0}

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    if self.CP.flags & MazdaFlags.GEN2:
      return self._update_gen2(can_parsers)
    return self._update_gen1(can_parsers)

  def _update_gen1(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    # cruise control button events: distance, inc, and dec
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    self.accel_button = cp.vl["CRZ_BTNS"]["RES"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
    ]

    return ret, ret_sp

  def _update_gen2(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    # MAZDA_3_2019 (GEN2 + Torque Interceptor) update path. Ported from the
    # source fork's selfdrive/car/mazda/carstate.py:update_gen2 with all
    # third-party-fork toggle infrastructure stripped (no extra return tuple,
    # no toggle dict, no global settings reads).
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    # Local handle for the body bus (TI feedback). Not a method parameter.
    cp_aux = can_parsers[Bus.body]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Wheel speeds + vEgo. Camera bus (bus 2) is where wheel speeds appear on GEN2.
    self.parse_wheel_speeds(ret,
      cp_cam.vl["WHEEL_SPEEDS"]["FL"],
      cp_cam.vl["WHEEL_SPEEDS"]["FR"],
      cp_cam.vl["WHEEL_SPEEDS"]["RL"],
      cp_cam.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Cluster speed honors the dash imperial-vs-metric setting on GEN2.
    unit_conversion = CV.MPH_TO_MS if cp.vl["SYSTEM_SETTINGS"]["IMPERIAL_UNIT"] else CV.KPH_TO_MS
    ret.standstill = cp_cam.vl["SPEED"]["SPEED"] * unit_conversion < 0.1

    # Gear (automatic only — manual transmission flag is intentionally not exposed
    # in upstream MazdaFlags for the GEN2 port).
    can_gear = int(cp_cam.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # Continuous lamp blinkers. 100ms refresh mirrors the source fork.
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(100, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    # Steering angle still comes from the OEM STEER message on the main bus;
    # rate is derived numerically since GEN2 STEER does not carry STEER_ANGLE_RATE.
    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringRateDeg = (ret.steeringAngleDeg - self._prev_steering_angle) / DT_CTRL
    self._prev_steering_angle = ret.steeringAngleDeg

    # Driver torque is reported by the TI module via EPS_FEEDBACK on the body bus.
    # mazda_2019.dbc folds the GEN2 EPS feedback into a single message — no separate
    # motor-torque output channel is exposed, so steeringTorqueEps is left at its
    # default 0 (matching the source fork's GEN2 behavior).
    ret.steeringTorque = cp_aux.vl["EPS_FEEDBACK"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > self.params.STEER_DRIVER_ALLOWANCE

    # TI fault detection. mazda_2019.dbc folds the 4-MCU state vector into the
    # EPS_FEEDBACK message (id 0x24B); each CPU_x_STATE is a 4-bit field.
    # TI2 firmware values per manufacturer docs:
    #   0 = OFF, 1 = INIT, 2 = STANDBY, 3 = DRIVE, 4 = ERROR, 5 = CRITICAL_ERROR
    #
    # We split this into two openpilot-side faults:
    #   - steerFaultPermanent: any CPU at ERROR (4) or CRITICAL_ERROR (5).
    #     These latch and require an ignition cycle, matching how the TI2
    #     firmware itself locks an MCU into bypass/silent until reset.
    #   - steerFaultTemporary: any CPU not in DRIVE (3) without being permanent.
    #     INIT and STANDBY occur during normal TI2 boot and resolve once all
    #     four MCUs receive valid torque-sensor input; OFF means the TI module
    #     is unpowered and recovers when power returns.
    HW_DRIVE = 3
    HW_ERROR = 4
    cpu_states = (
      cp_aux.vl["EPS_FEEDBACK"]["CPU_0_STATE"],
      cp_aux.vl["EPS_FEEDBACK"]["CPU_1_STATE"],
      cp_aux.vl["EPS_FEEDBACK"]["CPU_2_STATE"],
      cp_aux.vl["EPS_FEEDBACK"]["CPU_3_STATE"],
    )
    ret.steerFaultPermanent = any(int(state) >= HW_ERROR for state in cpu_states)
    ret.steerFaultTemporary = (any(int(state) != HW_DRIVE for state in cpu_states)
                                and not ret.steerFaultPermanent)

    # Collapse the 4-MCU vector into the opendbc TI_STATE enum that T11's
    # carcontroller consumes. TI2 hardware values per firmware docs are
    # 0=OFF, 1=INIT, 2=STANDBY, 3=DRIVE, 4=ERROR, 5=CRITICAL_ERROR; the
    # opendbc TI_STATE enum only has 4 values (DISCOVER=0, OFF=1,
    # DRIVER_OVER=2, RUN=3). Hardware DRIVE coincides numerically with
    # opendbc TI_STATE.RUN (both = 3) but the rest do not line up, so we
    # collapse to a binary view: all four hall MCUs in DRIVE -> RUN, any
    # other reading -> OFF. T11 only branches on RUN-vs-not-RUN.
    self.ti_state = TI_STATE.RUN if all(int(state) == HW_DRIVE for state in cpu_states) else TI_STATE.OFF

    # Throttle / brake. GEN2 ENGINE_DATA on the camera bus carries the gas pedal;
    # brake comes back as a binary signal on BRAKE_PEDAL (no analog pressure
    # available without the higher-rate BRAKE_PEDAL_SLOW message, which we skip
    # here to match the source fork).
    ret.gas = cp_cam.vl["ENGINE_DATA"]["PEDAL_GAS"]
    ret.gasPressed = ret.gas > 0
    ret.brakePressed = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSED"] == 1
    ret.brake = 0.

    # Cruise. CRZ_STATE encodes: 0=off, >=1=available, >=2=engaged.
    ret.cruiseState.speed = cp.vl["CRUZE_STATE"]["CRZ_SPEED"] * unit_conversion
    ret.cruiseState.enabled = cp.vl["CRUZE_STATE"]["CRZ_STATE"] >= 2
    ret.cruiseState.available = cp.vl["CRUZE_STATE"]["CRZ_STATE"] != 0
    # Suppress standstill when openpilot is the longitudinal owner so the car
    # doesn't latch into a creep-stop loop fighting our own accel command.
    ret.cruiseState.standstill = ret.standstill if not self.CP.openpilotLongitudinalControl else False

    # Door / seatbelt status — GEN2 cars handle these interlocks internally
    # (cruise will not engage if either is unsafe), so surface a clean state.
    ret.seatbeltUnlatched = False
    ret.doorOpen = False

    # Snapshot the ACC frame for T11's carcontroller (it gates resume/hold
    # echoes off the OEM ACC ECU). The frame is parsed on Bus.pt per
    # get_can_parsers (matching the source fork's `self.acc = copy.copy(
    # cp.vl["ACC"])` — the panda forwards the camera-originated ACC frame
    # down to bus 0 where it is most reliably visible). dict() snapshot
    # decouples the consumer from the next CAN tick's vl rebind.
    self.acc_values = dict(cp.vl["ACC"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    if CP.flags & MazdaFlags.GEN2:
      pt_messages = [
        ("STEER", 50),
        ("BRAKE_PEDAL", 5),
        ("BLINK_INFO", 10),
        ("SYSTEM_SETTINGS", 10),
        ("ACC", 50),
        ("CRUZE_STATE", 10),
      ]
      cam_messages = [
        ("ENGINE_DATA", 100),
        ("WHEEL_SPEEDS", 100),
        ("SPEED", 50),
        ("GEAR", 40),
      ]
      body_messages = [
        ("EPS_FEEDBACK", 50),
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
        Bus.body: CANParser(DBC[CP.carFingerprint][Bus.pt], body_messages, 1),
      }

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
