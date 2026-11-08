"""Test that vehicle does not spin at high speed with moderate steering input."""
import pytest
from f1sim.physics.vehicle import Vehicle


def test_no_spin_at_300kmh_with_moderate_steer():
    """At 300 km/h, moderate steering should NOT cause yaw rate > 1.0 rad/s (57 deg/s)."""
    v = Vehicle()
    v.state.vx = 83.3  # 300 km/h
    v.state.gear = 8
    v.powertrain.set_running()
    # Simulate 0.5 seconds with moderate steering
    for _ in range(500):  # 500 steps * 0.001s = 0.5s
        v.step(throttle=0.8, brake=0.0, steer=0.3, dt=0.001)
    assert abs(v.state.yaw_rate) < 1.0, f"Yaw rate {v.state.yaw_rate:.2f} rad/s exceeds stability limit at 300 km/h"
    assert abs(v.state.vx) > 50.0, f"Vehicle decelerated excessively to {v.state.vx:.1f} m/s"


def test_full_steer_at_low_speed_still_works():
    """At low speed (30 km/h), full steering should still allow tight cornering."""
    v = Vehicle()
    v.state.vx = 8.3  # 30 km/h
    v.state.gear = 2
    v.powertrain.set_running()
    for _ in range(1000):  # 1 second
        v.step(throttle=0.3, brake=0.0, steer=0.9, dt=0.001)
    # Should have significant yaw rate for tight corner
    assert abs(v.state.yaw_rate) > 0.15, f"Yaw rate {v.state.yaw_rate:.2f} too low — steering limiter too aggressive at low speed"
