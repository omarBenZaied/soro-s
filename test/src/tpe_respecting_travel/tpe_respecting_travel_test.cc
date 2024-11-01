#pragma once
#include "doctest/doctest.h"

#include <ranges>

#include "utl/parallel_for.h"

#include "soro/utls/std_wrapper/contains.h"

#include "soro/infrastructure/infrastructure.h"

#include "soro/runtime/euler_runtime.h"
#include "soro/runtime/rk4_runtime.h"

#include "soro/runtime/common/tpe_respecting_travel.h"

#include <soro/infrastructure/parsers/iss/parse_track_element.h>
#include <soro/runtime/physics/rk4/detail/get_speed_limit.h>
#include <test/file_paths.h>

#include "soro/runtime/common/train_path_envelope.h"

#include "soro/exceptions/tpe_exceptions.h"

#include "test/tpe_runtime/tpe_simulation_utls.h"
namespace soro::tpe_simulation{
using namespace runtime;
using namespace infra;
using namespace test;
using tpe_changer = tpe_point(tpe_point const&);
using tpe_maker = tpe_points(tt::train const&,infrastructure const&,infra::type_set const&);
TEST_SUITE("tpe respecting travel suite") {
  train_state drive_to_point(train_state state,interval& interval,tt::train const& t,tpe_point const& point) {
    shortest_travel_time shortest_travel_time;
    tt::train::trip const trip(tt::train::trip::id{0}, t.id_, ZERO<absolute_time>);
    signal_time signal_time;
    while(state.dist_!=point.distance_) {
      auto delta = shortest_travel_time.drive(state,nullptr,interval,t,trip,signal_time);
      state+=delta;
      ++interval;
    }
    return state;
  }
  void check_tpe_respecting_simulation(vector<tt::train> const& trains,infra::infrastructure const& infra,infra::type_set const& record_types,tpe_changer tpe_changer,tpe_maker tpe_maker){
    bool did_not_throw = false;
    for(auto const& t : trains) {
      auto tpe_points = tpe_maker(t,infra,record_types);
      for(int i=0;i<tpe_points.size();++i) tpe_points[i] = tpe_changer(tpe_points[i]);
      train_state state;
      state.speed_ = t.start_speed_;
      tt::train::trip const trip(tt::train::trip::id{0}, t.id_, ZERO<absolute_time>);
      vector<train_state> result;
      increase_time::train_drive drive;
      try {
        std::tie(result,drive) = tpe_respecting_simulation(tpe_points,state,nullptr,t,trip,infra,record_types);
        did_not_throw = true;
      }
      catch(tpe_l_time_exception const& e) {
        std::cout<<"Threw l_time exception"<<std::endl;
        auto it = utls::find_if(e.intr_points_,[e](interval_point const& intr_point){return intr_point.distance_ ==e.state_.dist_;});
        interval interval(&*it,&*(it+1));
        auto point_state = drive_to_point(e.state_,interval,t,e.point_);
        utls::sassert(point_state.time_>e.point_.l_time_,"tpe respecting travel threw l_time exception when it shouldnt have");
        continue;
      }
      catch(tpe_speed_exception const& e) {
        std::cout<<"Threw speed exception"<<std::endl;
        utls::sassert(e.state_.dist_.is_zero(),"tpe speed exception after the beginning");
        interval interval(e.intr_points_.data(),e.intr_points_.data()+1);
        auto point_state = drive_to_point(e.state_,interval,t,e.point_);
        utls::sassert(point_state.speed_ < e.point_.v_min_,"tpe respecting travel threw speed exception when it shouldnt have");
        continue;
      }
      std::sort(tpe_points.begin(),tpe_points.end());
      merge_duplicate_tpe_points(tpe_points);
      auto sorted_time = std::ranges::is_sorted(result,[]
        (train_state const& state1,train_state const& state2){return state1.time_ < state2.time_;});
      auto unique_time = std::ranges::adjacent_find(result,[]
        (train_state const& state1,train_state const& state2){return state1.time_ == state2.time_;});
      CHECK(sorted_time);
      CHECK_EQ(unique_time,result.end());

      CHECK_EQ(tpe_points.size(),result.size());
      for(int i =0;i<tpe_points.size();++i) {
        CHECK_EQ(tpe_points[i].distance_,result[i].dist_);

        CHECK_LE(result[i].speed_,tpe_points[i].v_max_);
        CHECK_GE(result[i].speed_,tpe_points[i].v_min_);

        auto predecessor_time = i==0?si::time::zero():result[i-1].time_;
        CHECK_LE(result[i].time_-predecessor_time,tpe_points[i].l_time_);
        CHECK_GE(result[i].time_-predecessor_time,tpe_points[i].e_time_);
      }

      check_drive(drive,0);
      check_drivable(drive,get_intervals(t,record_types,infra).p_,t,0);

      utls::for_each(drive.phases_,[](vector<train_state> const& phase){CHECK_EQ(phase.size(),2);});
    }
    utls::sassert(did_not_throw,"threw for every train");
  }
  void check_slowest_drive(vector<tt::train> const& trains,infrastructure const& infra,type_set const& record_types) {
    for(auto const& t:trains) {
      auto points = get_tpe_points(t,infra,record_types);
      merge_duplicate_tpe_points(points);
      auto intervals = split_intervals(get_intervals(t,record_types,infra),points,t.physics_);
      auto interval = intervals.begin();
      train_state start_state;
      for(int i=0;i<points.size()-1;++i) {
        start_state = drive_to_point(start_state,interval,t,points[i]);
        auto states = slowest_drive(start_state,points[i+1],interval,t.physics_);

        auto sorted_time = std::ranges::is_sorted(states,[](train_state const& state1,train_state const& state2){return state1.time_<state2.time_;});
        utls::sassert(sorted_time,"slowest drive isnt sorted in time");
        utls::sassert(!states.front().time_.is_negative(),"negative times");
        auto unique_time = std::ranges::adjacent_find(states,[](train_state const& state1,train_state const& state2){return state1.time_==state2.time_;});
        utls::sassert(unique_time==states.end(),"slowest drive isnt unique in time");

        auto sorted_dist = std::ranges::is_sorted(states,[](train_state const& state1,train_state const& state2){return state1.dist_<state2.dist_;});
        utls::sassert(sorted_dist,"slowest drive isnt sorted in distance");
        auto unique_dist = std::ranges::adjacent_find(states,[](train_state const& state1,train_state const& state2){return state1.dist_==state2.dist_;});
        utls::sassert(unique_time==states.end(),"slowest drive isnt unique in distance");

        auto it = std::adjacent_find(states.begin(),states.end(),
          [](train_state const& state1,train_state const& state2){return state1.speed_<state2.speed_;});
        if(it==states.end()) {
          auto sorted_speed_ascending = std::ranges::is_sorted(states.begin(),states.end(),
            [](train_state const& state1,train_state const& state2){return state1.speed_<state2.speed_;});
          auto sorted_speed_descending = std::ranges::is_sorted(states.begin(),states.end(),
            [](train_state const& state1,train_state const& state2){return state1.speed_>state2.speed_;});
          utls::sassert(sorted_speed_descending||sorted_speed_ascending,"slowest drive isnt sorted either way");
          continue;
        }
        auto sorted_descending = std::ranges::is_sorted(states.begin(),it+1,
            [](train_state const& state1,train_state const& state2){return state1.speed_>state2.speed_;});
        utls::sassert(sorted_descending,"braking part of slowest drive isnt descending in speed");

        auto sorted_ascending = std::ranges::is_sorted(it+1,states.end(),
            [](train_state const& state1,train_state const& state2){return state1.speed_<state2.speed_;});
        utls::sassert(sorted_ascending,"accelerating part of slowest drive isnt ascending in speed");
      }
    }
  }

  TEST_CASE("slowest drive hill") {
    infrastructure const infra(HILL_OPTS);
    tt::timetable const tt(HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    check_slowest_drive(trains,infra,type_set({type::HALT,type::EOTD}));
  }

  TEST_CASE("slowest drive intersection") {
    infrastructure const infra(INTER_OPTS);
    tt::timetable const tt(INTER_TT_OPTS, infra);
    check_slowest_drive({tt->trains_[0]},infra,type_set({type::HALT,type::EOTD}));
  }

  TEST_CASE("slowest drive follow") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    check_slowest_drive(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }

  TEST_CASE("slowest drive cross") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_slowest_drive(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }

  TEST_CASE("tpe respecting travel hill") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto identity = [](tpe_point const& pt){return pt;};
    check_tpe_respecting_simulation(trains,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points);
  }

  TEST_CASE("tpe respecting travel intersection") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_[0]};
    auto identity = [](tpe_point const& pt){return pt;};
    check_tpe_respecting_simulation(trains,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points);
  }

  TEST_CASE("tpe respecting travel follow") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    auto l_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::infinity();
      return point;
    };
    check_tpe_respecting_simulation(tt->trains_,infra,infra::type_set({infra::type::HALT,infra::type::EOTD}),l_time_increaser,get_tpe_points);
  }

  TEST_CASE("tpe respecting travel cross") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto l_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::infinity();
      return point;
    };
    check_tpe_respecting_simulation(tt->trains_,infra,infra::type_set({infra::type::HALT,infra::type::EOTD}),l_time_increaser,get_tpe_points);
  }

}
}// namespace soro::tpe_simulation
