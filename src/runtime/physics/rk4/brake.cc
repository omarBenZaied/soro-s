#include "soro/runtime/physics/rk4/brake.h"

#include "soro/utls/sassert.h"

#include "soro/si/units.h"

#include "soro/runtime/common/runtime_result.h"

namespace soro::runtime::rk4 {

train_state brake(si::speed const initial_speed, si::speed const target_speed,
                  si::accel const deaccel) {
  utls::expect(deaccel < si::accel::zero(),
               "negative deacceleration required, got {}", deaccel);
  utls::expect(initial_speed > target_speed,
               "target speed higher than current speed in deacceleration!");

  auto const delta_speed = initial_speed - target_speed;

  train_state result;
  result.time_ = -delta_speed / deaccel;
  result.speed_ = target_speed;
  result.dist_ =
      (0.5 * deaccel * result.time_.pow<2>()) + (initial_speed * result.time_);

  return result;
}

train_state brake_over_distance(si::speed const initial_speed,
                                si::accel const deaccel,
                                si::length const distance) {
  utls::expect(deaccel < si::accel::zero(),
               "negative deacceleration required, got {}", deaccel);
  utls::expect(distance > si::length::zero(), "got negative distance");

  auto const v_inner = initial_speed.pow<2>() + (2 * deaccel * distance);
  if (v_inner.is_negative()) {
    return brake(initial_speed, si::speed::zero(), deaccel);
  }

  train_state result;
  result.speed_ = v_inner.sqrt();
  result.time_ = (initial_speed - result.speed_) / deaccel;
  result.dist_ = distance;

  return result;
}
train_state brake_backwards(train_state state,si::accel deaccel, si::length stop_distance,si::speed max_speed){
  train_state result;
  result.dist_ = stop_distance;
  result.speed_ = (state.speed_.pow<2>()+2*deaccel*(stop_distance-state.dist_)).sqrt();
  result.speed_ = std::min(result.speed_,max_speed);
  return result;
}
train_state brake_over_distance_with_target(train_state const& state,si::accel const& deaccel,si::length const&  stop_distance,si::speed const& target_speed){
  auto brake_state = brake_over_distance(state.speed_,deaccel,stop_distance);
  if(brake_state.speed_<target_speed) brake_state = brake(state.speed_,target_speed,deaccel);
  train_state result(state);
  result+=brake_state;
  result.speed_ = brake_state.speed_;
  return result;
}
}  // namespace soro::runtime::rk4
