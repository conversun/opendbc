from opendbc.car.mazda.values import Buttons, MazdaFlags


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": 0,
      "SET_P_INV": 1,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": 0,
      "SET_M_INV": 1,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)


def create_steering_control_gen2(packer, apply_torque):
  # GEN2 LKAS over EPS_LKAS (addr 0x249) on bus 1 (MAZDA_AUX). Replaces stock LKAS for
  # GEN2 platforms (e.g., MAZDA_3_2019). Same message also serves as the TI LKAS path
  # for GEN2 + TI hardware: panda safety treats addr 0x249 on bus 1 as MAZDA_TI_LKAS.
  # Counter (8-bit COUNTER signal) is auto-incremented by CANPacker.
  #
  # NOTE: The mazda_2019 DBC defines a CHECKSUM signal that the source fork computes via
  # the C++ packer's mazda2019_checksum hook (opendbc/can/common.cc). Upstream's pure-Python
  # CANPacker does not yet register a Mazda 2019 checksum function in opendbc/can/dbc.py's
  # get_checksum_state(), so CHECKSUM will currently pack as 0. A follow-up task is required
  # to add a Python mazda2019_checksum and wire it into get_checksum_state() before this
  # builder will be accepted by the EPS at runtime. The builder itself is ported 1:1 from
  # selfdrive/car/mazda/mazdacan.py:create_steering_control GEN2 branch and does not change.
  values = {
    "LKAS_REQUEST": apply_torque,
    "STEER_FEEL": 10000,
  }
  return packer.make_can_msg("EPS_LKAS", 1, values)


def create_acc_cmd(packer, values, hold, resume):
  # GEN2 longitudinal command over ACC (addr 0x220) on bus 2 (MAZDA_CAM). Forwarded from
  # the stock ACC values dict that carstate copied off the camera bus, with HOLD and RESUME
  # overridden when ACC_ENABLED is set. Ported 1:1 from selfdrive/car/mazda/mazdacan.py:
  # create_acc_cmd; the source took an unused `self` arg which is dropped here.
  if values["ACC_ENABLED"]:
    values["HOLD"] = hold
    values["RESUME"] = resume
  return packer.make_can_msg("ACC", 2, values)
