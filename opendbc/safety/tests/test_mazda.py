#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

FLAG_MAZDA_GEN2 = 2
FLAG_MAZDA_TORQUE_INTERCEPTOR = 8
FLAG_MAZDA_LONG = 16
FLAG_MAZDA_LOWSPEED_LONG = 32

MAZDA_MAIN = 0
MAZDA_AUX = 1
MAZDA_CAM = 2

MAZDA_2019_ACC = 0x220
MAZDA_2019_ACC_2 = 0x222
MAZDA_TI_LKAS = 0x249


class TestMazdaSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  TX_MSGS = [[0x243, 0], [0x09d, 0], [0x440, 0]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {0: (0x243, 0x440)}
  FWD_BLACKLISTED_ADDRS = {2: [0x243, 0x440]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]

  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 15
  DRIVER_TORQUE_FACTOR = 1

  # Mazda actually does not set any bit when requesting torque
  NO_STEER_REQ_BIT = True

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2017")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, 0)
    self.safety.init_tests()

  def _torque_meas_msg(self, torque):
    values = {"STEER_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"STEER_TORQUE_SENSOR": torque}
    return self.packer.make_can_msg_safety("STEER_TORQUE", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_REQUEST": torque}
    return self.packer.make_can_msg_safety("CAM_LKAS", 0, values)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_ON": brake}
    return self.packer.make_can_msg_safety("PEDALS", 0, values)

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRZ_ACTIVE": enable}
    return self.packer.make_can_msg_safety("CRZ_CTRL", 0, values)

  def _button_msg(self, resume=False, cancel=False):
    values = {
      "CAN_OFF": cancel,
      "CAN_OFF_INV": (cancel + 1) % 2,
      "RES": resume,
      "RES_INV": (resume + 1) % 2,
    }
    return self.packer.make_can_msg_safety("CRZ_BTNS", 0, values)

  def test_buttons(self):
    # only cancel allows while controls not allowed
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertFalse(self._tx(self._button_msg(resume=True)))

    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(cancel=True)))
    self.assertTrue(self._tx(self._button_msg(resume=True)))



class TestMazdaGen2Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  FLAGS = FLAG_MAZDA_GEN2
  TX_MSGS = [[MAZDA_TI_LKAS, MAZDA_AUX], [MAZDA_2019_ACC, MAZDA_CAM]]
  STANDSTILL_THRESHOLD = .1
  RELAY_MALFUNCTION_ADDRS = {MAZDA_AUX: (MAZDA_TI_LKAS,), MAZDA_CAM: (MAZDA_2019_ACC,)}
  FWD_BLACKLISTED_ADDRS = {MAZDA_MAIN: [MAZDA_2019_ACC, MAZDA_TI_LKAS], MAZDA_CAM: [MAZDA_TI_LKAS]}

  MAX_RATE_UP = 45
  MAX_RATE_DOWN = 80
  MAX_TORQUE_LOOKUP = [0], [8000]

  MAX_RT_DELTA = 1688

  DRIVER_TORQUE_ALLOWANCE = 1400
  DRIVER_TORQUE_FACTOR = 1

  # Mazda TI LKAS does not use a separate request bit.
  NO_STEER_REQ_BIT = True
  DRIVER_TORQUE_BUS = MAZDA_AUX

  def setUp(self):
    self.packer = CANPackerSafety("mazda_2019")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, self.FLAGS)
    self.safety.init_tests()

  @staticmethod
  def _signed_word_msg(addr, bus, value):
    return libsafety_py.make_CANPacket(addr, bus, int(value).to_bytes(2, "big", signed=True) + b"\x00" * 6)

  def _torque_meas_msg(self, torque):
    return self._torque_driver_msg(torque)

  def _torque_driver_msg(self, torque):
    return self._signed_word_msg(0x24b, self.DRIVER_TORQUE_BUS, torque)

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._signed_word_msg(MAZDA_TI_LKAS, MAZDA_AUX, torque)

  def _speed_msg(self, speed):
    values = {"SPEED": speed}
    return self.packer.make_can_msg_safety("SPEED", MAZDA_CAM, values)

  def _speed_msg_2(self, speed: float):
    return None

  def _wheel_speeds_msg(self, speed):
    values = {s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", MAZDA_CAM, values)

  def _user_brake_msg(self, brake):
    dat = bytearray(8)
    dat[5] = 0x4 if brake else 0x0
    return libsafety_py.make_CANPacket(0x43f, MAZDA_MAIN, bytes(dat))

  def _user_gas_msg(self, gas):
    values = {"PEDAL_GAS": gas}
    return self.packer.make_can_msg_safety("ENGINE_DATA", MAZDA_CAM, values)

  def _pcm_status_msg(self, enable):
    dat = bytearray(8)
    dat[0] = 0x20 if enable else 0x0
    return libsafety_py.make_CANPacket(0x44a, MAZDA_MAIN, bytes(dat))

  def _button_msg(self):
    values = {"CAN": 0, "RES": 0, "SET_M": 0, "SET_P": 0}
    return self.packer.make_can_msg_safety("CRZ_BTNS", MAZDA_MAIN, values)

  def _send_valid_gen2_rx(self, speed_msg):
    self.assertTrue(self._rx(self._user_brake_msg(False)))
    self.assertTrue(self._rx(self._user_gas_msg(0)))
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertTrue(self._rx(speed_msg))
    self.assertTrue(self._rx(self._torque_driver_msg(0)))
    self.assertTrue(self._rx(self._button_msg()))

  def test_speed_rx_checks_accept_speed_msg(self):
    self._send_valid_gen2_rx(self._speed_msg(1.0))
    self.safety.set_timer(1000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_speed_rx_checks_accept_wheel_speeds_msg(self):
    self._send_valid_gen2_rx(self._wheel_speeds_msg(1.0))
    self.safety.set_timer(1000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())

  def test_gen2_acc_tx_allowed(self):
    self.assertTrue(self._tx(libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_CAM, b"\x00" * 8)))
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(MAZDA_2019_ACC, MAZDA_MAIN, b"\x00" * 8)))


class TestMazdaGen2TiSafety(TestMazdaGen2Safety):
  FLAGS = FLAG_MAZDA_GEN2 | FLAG_MAZDA_TORQUE_INTERCEPTOR
  DRIVER_TORQUE_BUS = MAZDA_MAIN

  def test_ti_lkas_is_only_allowed_bus_1_tx(self):
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

    for addr in [0x220, 0x243, 0x440, 0x24b, 0x74b, 0x74c]:
      self.assertFalse(self._tx(libsafety_py.make_CANPacket(addr, MAZDA_AUX, b"\x00" * 8)))

  def test_bus_0_driver_torque_blocks_ti_lkas(self):
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(-self.DRIVER_TORQUE_ALLOWANCE - 1)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE)))

  def test_aux_bus_not_forwarded(self):
    for addr in [0, MAZDA_TI_LKAS, 0x220, 0x7ff]:
      self.assertEqual(-1, self.safety.safety_fwd_hook(MAZDA_AUX, addr))

class TestMazdaGen2LowSpeedProbe(TestMazdaGen2Safety):
  # Low-speed longitudinal probe: engagement authority additionally follows the OEM ACC_2.ACC_ENABLED
  # bit (0x222) so controls_allowed persists below the CRZ_STATE display floor. All base GEN2 safety
  # behavior must remain unchanged when ACC_2 authority is absent (acc2_authority defaults False).
  FLAGS = FLAG_MAZDA_GEN2 | FLAG_MAZDA_LOWSPEED_LONG

  def _acc2_msg(self, enabled, not_enabled=False):
    dat = bytearray(8)
    if enabled:
      dat[2] |= 0x04  # ACC_2.ACC_ENABLED
    if not_enabled:
      dat[0] |= 0x02  # ACC_2.ACC_NOT_ENABLED
    return libsafety_py.make_CANPacket(MAZDA_2019_ACC_2, MAZDA_CAM, bytes(dat))

  def _send_valid_gen2_rx(self, speed_msg):
    super()._send_valid_gen2_rx(speed_msg)
    self.assertTrue(self._rx(self._acc2_msg(True)))

  def test_acc2_authority_engages_below_crz_floor(self):
    # OEM ACC_2 authority present while CRZ_STATE is NOT engaged -> controls allowed (low-speed hold)
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))   # establish cruise_engaged_prev = not engaged
    self._rx(self._acc2_msg(True))          # OEM authority on
    self._rx(self._pcm_status_msg(False))   # CRZ still not engaged -> engage via ACC_2 authority
    self.assertTrue(self.safety.get_controls_allowed())

  def test_acc2_not_enabled_blocks_authority(self):
    # ACC_NOT_ENABLED overrides ACC_ENABLED -> no authority -> not engaged
    self.safety.set_controls_allowed(0)
    self._rx(self._pcm_status_msg(False))
    self._rx(self._acc2_msg(True, not_enabled=True))
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc2_authority_lost_disengages(self):
    self._rx(self._pcm_status_msg(False))
    self._rx(self._acc2_msg(True))
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._acc2_msg(False))         # authority lost -> disengage on next cruise frame
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc2_required_in_rx_checks(self):
    # With everything fresh INCLUDING ACC_2 the config is valid
    self._send_valid_gen2_rx(self._speed_msg(1.0))
    self.safety.set_timer(1000)
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid())


class TestMazdaGen2LongAccel(unittest.TestCase):
  # Focused coverage for the GEN2 openpilot-longitudinal ACCEL_CMD TX safety check (FLAG_MAZDA_LONG).
  # The low-speed probe always runs under op-long, so this path is exercised in production. Standalone
  # (no generic CarSafetyTest inheritance) to avoid the base ACC-tx tests' zero-accel assumptions.
  def setUp(self):
    self.packer = CANPackerSafety("mazda_2019")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.mazda, FLAG_MAZDA_GEN2 | FLAG_MAZDA_LONG)
    self.safety.init_tests()

  def _acc_tx(self, accel_raw):
    # ACCEL_CMD raw = accel*200 + 2000; limits [1300, 2400], inactive 2000 (MAZDA_2019_LONG_LIMITS).
    return self.packer.make_can_msg_safety("ACC", MAZDA_CAM, {"ACCEL_CMD": accel_raw})

  def test_accel_cmd_limits_when_allowed(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_tx_hook(self._acc_tx(2000)))   # inactive
    self.assertTrue(self.safety.safety_tx_hook(self._acc_tx(1300)))   # min
    self.assertTrue(self.safety.safety_tx_hook(self._acc_tx(2400)))   # max
    self.assertFalse(self.safety.safety_tx_hook(self._acc_tx(1299)))  # below min blocked
    self.assertFalse(self.safety.safety_tx_hook(self._acc_tx(2401)))  # above max blocked

  def test_accel_cmd_inactive_only_when_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.safety_tx_hook(self._acc_tx(2000)))   # inactive sentinel ok
    self.assertFalse(self.safety.safety_tx_hook(self._acc_tx(1500)))  # non-inactive blocked

if __name__ == "__main__":
  unittest.main()
