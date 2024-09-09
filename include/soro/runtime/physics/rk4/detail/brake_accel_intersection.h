//
// Created by omarb on 02.09.2024.
//
#include "soro/runtime/common/train_state.h"
#include "soro/runtime/common/interval.h"
#include "soro/utls/sassert.h"
#include "soro/runtime/physics/rk4/detail/rk4_step.h"
#include "soro/runtime/physics/rk4/detail/delta_t.h"
#include "soro/runtime/physics/rk4/brake.h"
#pragma once
namespace b_a_intersection {
using namespace soro::runtime;

/**
 * returns the state, where a brake process and an acceleration meet
 * @param brake_state beginning of the brake_process
 * @param accel_state beginning of the acceleration process
 * @param interval the interval where the intersection must take place. This method wont work if they dont meet in this interval
 * @param tp the train physics
 * @return the state where the two processes meet
 */
// TODO: suche analytische lösung der beschleunigung
inline train_state brake_accel_intersection(train_state const& brake_state,
                                            train_state accel_state,
                                            interval const& interval,
                                            soro::rs::train_physics const& tp) {
  train_state second_to_last;
  do {
    second_to_last = accel_state;
    accel_state +=
        rk4::rk4_step(accel_state.speed_, rk4::delta_t, interval.slope(), tp);
  } while (accel_state.speed_ <
           rk4::brake_over_distance(
               brake_state.speed_,
               tp.braking_deaccel(interval.infra_limit(), interval.bwp_limit(),
                                  interval.brake_path_length()),
               accel_state.dist_)
               .speed_);
  if (accel_state.speed_ ==
      rk4::brake_over_distance(
          brake_state.speed_,
          tp.braking_deaccel(interval.infra_limit(), interval.bwp_limit(),
                             interval.brake_path_length()),
          accel_state.dist_)
          .speed_)
    return accel_state;
  auto s_a = second_to_last.dist_;
  auto s_b = brake_state.dist_;
  auto v_a = second_to_last.speed_;
  auto v_b = brake_state.speed_;
  auto a_a = tp.acceleration(second_to_last.speed_, interval.slope());
  auto a_b = tp.braking_deaccel(interval.infra_limit(), interval.bwp_limit(),
                                interval.brake_path_length());
  soro::utls::sassert(a_a != a_b,
                      "Acceleration and decceleration are the same, no brake_accel_intersection can be found");
  soro::utls::sassert(
      !a_a.is_zero(),
      "Acceleration is zero, no brake_accel_intersection can be calculated");
  soro::utls::sassert(
      !a_b.is_zero(),
      "Decceleration is zero, no brake_accel_intersection can be calculated");

  auto result_speed =
      (s_a - s_b - v_a.pow<2>() / (2 * a_a) + v_b.pow<2>() / (2 * a_b)) /
      (1 / (2 * a_a) - 1 / (2 * a_b));
  soro::utls::sassert(!result_speed.is_negative(), "Intersection doesnt exist");
  train_state state;
  state.speed_ = soro::si::speed{result_speed.sqrt().val_};
  state.dist_ = s_a + v_a * (state.speed_ - v_a) / a_a +
                0.5 * a_a * ((state.speed_ - v_a) / a_a).pow<2>();
  return state;
}
} //namespace b_a_intersection