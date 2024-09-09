#pragma once

#include "soro/si/units.h"

#include "soro/rolling_stock/train_physics.h"

#include "soro/runtime/common/train_state.h"

#include "soro/runtime/common/interval.h"

namespace soro::runtime::rk4 {

train_state accelerate(si::speed const initial_speed, si::speed const max_speed,
                       si::speed const target_speed, si::length max_dist,
                       si::accel const deaccel, si::slope const slope,
                       si::length const stop_at, rs::train_physics const& tp);

train_state accelerate_backwards(train_state initial_state,interval const& interval ,rs::train_physics const& tp);


}  // namespace soro::runtime::rk4
