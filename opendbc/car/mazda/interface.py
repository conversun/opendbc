#!/usr/bin/env python3
from math import exp, fabs

import numpy as np

from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, LateralAccelFromTorqueCallbackType, TorqueFromLateralAccelCallbackType
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, MazdaFlags


# Sigmoid+linear lateral-accel-to-torque coefficients for GEN2 platforms.
# Tuple is (a, b, c, latAccelFactor); latAccelFactor is unused here (only the linear fallback uses it).
# Values ported verbatim from upstream source fork: selfdrive/car/mazda/interface.py:17-22.
NON_LINEAR_TORQUE_PARAMS = {
  CAR.MAZDA_3_2019: (4.6, 0.6, 0.134, 0.3605),
}


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def get_lataccel_torque_siglin(self):
    # The "lat_accel vs torque" relationship is the sum of "sigmoid + linear" curves. The slope at 0
    # should be > 0 (ideally > 1) for stability about 0 (noise when going straight). We pre-compute a
    # lookup table so the runtime callback is a constant-time np.interp instead of a sigmoid eval.
    def torque_from_lateral_accel_siglin_func(lateral_acceleration: float) -> float:
      non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
      assert non_linear_torque_params, "The params are not defined"
      a, b, c, _ = non_linear_torque_params
      sig_input = a * lateral_acceleration
      sig = np.sign(sig_input) * (1 / (1 + exp(-fabs(sig_input))) - 0.5)
      steer_torque = (sig * b) + (lateral_acceleration * c)
      return float(steer_torque)

    lataccel_values = np.arange(-8.0, 8.0, 0.01)
    torque_values = [torque_from_lateral_accel_siglin_func(x) for x in lataccel_values]
    assert min(torque_values) < -1 and max(torque_values) > 1, "The torque values should cover the range [-1, 1]"
    return torque_values, lataccel_values

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def torque_from_lateral_accel_siglin(lateral_acceleration: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(lateral_acceleration, lataccel_values, torque_values)
      return torque_from_lateral_accel_siglin
    return self.torque_from_lateral_accel_linear

  def lateral_accel_from_torque(self) -> LateralAccelFromTorqueCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      torque_values, lataccel_values = self.get_lataccel_torque_siglin()

      def lateral_accel_from_torque_siglin(torque: float, torque_params: structs.CarParams.LateralTorqueTuning):
        return np.interp(torque, torque_values, lataccel_values)
      return lateral_accel_from_torque_siglin
    return self.lateral_accel_from_torque_linear

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021, CAR.MAZDA_3_2019)

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # GEN2 (e.g., MAZDA_3_2019) uses a different camera protocol with a larger steer envelope and
    # supports openpilot longitudinal control via the alpha_long opt-in. MazdaFlags.* bit values
    # mirror panda's safety_mazda flags (see values.py), so the same int passes through
    # ret.safetyConfigs[0].safetyParam without translation.
    if ret.flags & MazdaFlags.GEN2:
      ret.safetyConfigs[0].safetyParam |= int(MazdaFlags.GEN2)
      ret.steerActuatorDelay = 0.335
      ret.startingState = True
      ret.stopAccel = -0.5
      ret.vEgoStarting = 0.2
      ret.longitudinalActuatorDelay = 0.35  # gas is 0.25s, brake looks like 0.5
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [0.0, 0.0, 0.0]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.1, 0.1]

    # Torque interceptor add-on hardware bypasses the EPS minSteerSpeed lockout by injecting steering
    # torque directly. Detected via the static MazdaFlags.TORQUE_INTERCEPTOR flag set declaratively in
    # MazdaPlatformConfig; never via runtime params.
    if ret.flags & MazdaFlags.TORQUE_INTERCEPTOR:
      ret.safetyConfigs[0].safetyParam |= int(MazdaFlags.TORQUE_INTERCEPTOR)

    # Pre-GEN2 EPS locks out steering below LKAS_LIMITS.DISABLE_SPEED unless a torque interceptor is
    # installed. CX5_2022 has the lockout disabled at the firmware level.
    if candidate not in (CAR.MAZDA_CX5_2022,) and not (ret.flags & (MazdaFlags.GEN2 | MazdaFlags.TORQUE_INTERCEPTOR)):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    # TODO(mazda3-2019): alpha longitudinal is disabled until carcontroller writes
    # ACCEL_CMD from CC.actuators.accel. Today we only echo the stock ACC frame and
    # override HOLD/RESUME, which is insufficient for openpilot to actually drive
    # gas/brake. Re-enable after porting the source fork's accel translation
    #   raw_acc_output = (CC.actuators.accel * 200) + 2000
    # into mazdacan.create_acc_cmd, gated on CC.longActive. Until then GEN2 uses
    # stock MRCC for longitudinal and openpilot owns lateral only.
    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = False

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret
