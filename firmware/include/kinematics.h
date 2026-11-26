#pragma once

#include "config.h"

namespace Kinematics {

/**
 * Konversi kecepatan linier (m/s) dan sudut (rad/s) menjadi target tick encoder per siklus kontrol.
 */
inline void computeWheelTicks(float linear, float angular, float &targetLeftTicks, float &targetRightTicks) {
  const float leftMps = linear - angular * WHEEL_SEPARATION_M / 2.0f;
  const float rightMps = linear + angular * WHEEL_SEPARATION_M / 2.0f;

  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  const float metersPerTick = (PI * WHEEL_DIAMETER_M) / PULSES_PER_WHEEL_REV;

  targetLeftTicks = leftMps * dt / metersPerTick;
  targetRightTicks = rightMps * dt / metersPerTick;
}

} // namespace Kinematics
