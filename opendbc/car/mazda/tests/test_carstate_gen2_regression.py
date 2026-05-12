"""GEN2 carstate regression guard: ensures T3.8 refactor doesn't change GEN2 signal routing.

For Scope C (offline-only), we verify key field assignments using a simple
integration check rather than a full pickle-based replay.
"""
import pytest
from opendbc.car.mazda.values import CAR
from opendbc.car.mazda.interface import CarInterface


def test_gen2_carparams_unchanged():
  """Smoke: GEN2 CarParams fields that carstate depends on remain stable."""
  cp = CarInterface.get_params(str(CAR.MAZDA_3_2019), {}, [], False, False, False)
  # These fields influence carstate._update_gen2 routing
  assert cp.safetyConfigs[0].safetyParam & 2  # GEN2 flag set
  assert cp.openpilotLongitudinalControl == False  # long not yet on by default
  assert cp.dashcamOnly == False


def test_gen2_dbc_routing():
  """GEN2 must use mazda_2019 DBC, not mazda_2017 or mazda_2023."""
  from opendbc.car.mazda.values import DBC
  from opendbc.car import Bus
  assert DBC[CAR.MAZDA_3_2019][Bus.pt] == "mazda_2019"
