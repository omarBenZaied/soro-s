#include "doctest/doctest.h"

#include <ranges>

#include "utl/logging.h"
#include "utl/parallel_for.h"

#include "soro/utls/cumulative_timer.h"
#include "soro/utls/print_progress.h"
#include "soro/utls/std_wrapper/contains.h"
#include "soro/utls/std_wrapper/count_if.h"
#include "soro/utls/std_wrapper/any_of.h"

#include "soro/infrastructure/infrastructure.h"

#include "soro/timetable/timetable.h"

#include "soro/runtime/common/use_surcharge.h"
#include "soro/runtime/euler_runtime.h"
#include "soro/runtime/rk4_runtime.h"

#include "test/file_paths.h"

#include "soro/runtime/common/get_intervals.h"
#include "soro/runtime/common/get_next_offset.h"
#include "soro/runtime/common/increase_time.h"
#include "soro/runtime/common/tpe_respecting_travel.h"
#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/physics/rk4/brake.h"
#include "soro/runtime/physics/rk4/detail/get_speed_limit.h"
#include "test/tpe_runtime/tpe_simulation_utls.h"
namespace increase_time_test{
using namespace soro;
using namespace soro::runtime;
using namespace increase_time;
using namespace soro::tpe_simulation;
using namespace soro::test;
TEST_SUITE("increase_time suite"){
  void check_increase_time(vector<tt::train> const& trains,infra::infrastructure const& infra,infra::type_set const& record_types){
    for(auto const& t:trains){
      auto tpe_points = get_tpe_points(t,infra,record_types);
      merge_duplicate_tpe_points(tpe_points);
      auto intervals = split_intervals(get_intervals(t,record_types,infra),tpe_points,t.physics_);
      train_state current;
      current.speed_ = t.start_speed_;
      current.time_ = si::time(t.start_time_.count());
      tt::train::trip const trip(tt::train::trip::id{0}, t.id_, ZERO<absolute_time>);
      double const ARRIVAL_FACTOR = 1.1;
      bool actually_tested = false;
      auto interval = intervals.begin();
      for(auto& tpe_point :tpe_points){
        tpe_point.e_time_ = tpe_point.e_time_*ARRIVAL_FACTOR;
        tpe_point.l_time_ = std::max(tpe_point.l_time_,tpe_point.e_time_);
        train_drive drive;
        std::tie(current,drive) = get_end_state(current,tpe_point,interval,nullptr,t,trip);
        if(drive.phase_types_.size()>=3) {
          actually_tested = true;
          increase_time::increase_time(drive, tpe_point, t.physics_,
                                         intervals.p_, standard_next_offset);
          CHECK_GE(drive.phases_.back().back().time_, tpe_point.e_time_);
          check_drive(drive, 0);
          check_drivable(drive, intervals.p_, t, 0);
        }
        drive.erase_elements(0,drive.phases_.size());
        drive.start_state_ = current;
      }
      utl::verify(actually_tested,"increase time was never used for train with id {}",t.id_);
    }
  }
  TEST_CASE("increase_time hill"){
    infra::infrastructure const infra(HILL_OPTS);
    tt::timetable const tt(HILL_TT_OPTS, infra);
    check_increase_time({tt->trains_[0]},infra,infra::type_set({infra::type::HALT,infra::type::EOTD}));
  }
  TEST_CASE("increase_time intersection"){
    infra::infrastructure const infra(INTER_OPTS);
    tt::timetable const tt(INTER_TT_OPTS, infra);
    check_increase_time({tt->trains_[0]},infra,infra::type_set({infra::type::HALT,infra::type::EOTD}));
  }
  TEST_CASE("increase_time follow"){
    infra::infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    check_increase_time(tt->trains_,infra,infra::type_set({infra::type::HALT,infra::type::EOTD}));
  }
  TEST_CASE("increase_time cross"){
    auto const infra =
        utls::try_deserializing<infra::infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_increase_time(tt->trains_,infra,infra::type_set({infra::type::HALT,infra::type::EOTD}));
  }
  TEST_CASE("check_AHD simple test"){
    train_drive drive;
    train_state one_delta(si::time(1),si::length(1),si::speed(1));
    train_state start_state;
    auto end_accel = start_state+one_delta;
    auto fake_acceleration = {start_state,end_accel};
    one_delta.speed_ = si::speed::zero();
    auto end_cruise = end_accel+one_delta;
    auto fake_cruise= {end_accel,end_cruise};
    one_delta.speed_ = si::speed(-1);
    auto end_brake = end_cruise + one_delta;
    auto fake_brake = {end_cruise,end_brake};
    drive.push_back(fake_acceleration,increase_time::acceleration);
    drive.push_back(fake_cruise,increase_time::cruising);
    drive.push_back(fake_brake,increase_time::braking);
  }
}
}// namespace increase_time_test
