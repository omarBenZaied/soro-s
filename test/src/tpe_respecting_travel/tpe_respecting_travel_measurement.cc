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
#include <soro/utls/cumulative_timer.h>
#include <test/file_paths.h>
#include <test/tpe_runtime/tpe_arrival_factor.h>

#include "soro/runtime/common/train_path_envelope.h"

#include "soro/exceptions/tpe_exceptions.h"

#include "test/tpe_runtime/tpe_simulation_utls.h"

namespace soro::tpe_simulation{
using namespace runtime;
using namespace infra;
using namespace test;
using tpe_changer = std::function<tpe_point(tpe_point const&)>;
using tpe_maker = tpe_points(tt::train const&,infrastructure const&,infra::type_set const&);
TEST_SUITE("tpe_respecting_travel performance measurement"){
  int constexpr  TRIALS = 100;
  std::chrono::microseconds measure_tpe_performance(tt::train const& t,infra::infrastructure const& infra,type_set const& record_types,tpe_changer const& tpe_changer,tpe_maker const& tpe_maker,int const& trials) {
    std::chrono::microseconds total_microseconds = std::chrono::microseconds::zero();
    train_state state;
    state.speed_ = t.start_speed_;
    tt::train::trip const trip(tt::train::trip::id{0}, t.id_, ZERO<absolute_time>);
    auto pts = tpe_maker(t,infra,record_types);
    for(int j=1;j<pts.size();++j) pts[j] = tpe_changer(pts[j]);
    for(int i=0;i<trials;++i) {
      tpe_points tpe_points_copy{pts};
      auto start = std::chrono::high_resolution_clock::now();
      try {
        tpe_respecting_simulation(tpe_points_copy,state,nullptr,t,trip,infra,record_types);
      }
      catch (std::runtime_error const&) {}
      auto end = std::chrono::high_resolution_clock::now();
      total_microseconds+=std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    }
    return total_microseconds/trials;
  }
  TEST_CASE("tpe respecting travel time measurement hill normal") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto identity = [](tpe_point const& pt){return pt;};
    for(auto const& t:trains) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points,TRIALS);
      std::cout << "Time for hill, normal condition, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for hill, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, hill, normal condition, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement intersection normal") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto identity = [](tpe_point const& pt){return pt;};
    for(auto const& t:trains) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points,TRIALS);
      std::cout << "Time for intersection, normal condition, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for intersection, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, intersection, normal condition, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement follow normal") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    auto identity = [](tpe_point const& pt){return pt;};
    for(auto const& t:tt->trains_) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points,TRIALS);
      std::cout << "Time for follow, normal condition, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for follow, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, follow, normal condition, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement cross normal") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto identity = [](tpe_point const& pt){return pt;};
    for(auto const& t:tt->trains_) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),identity,get_tpe_points,TRIALS);
      std::cout << "Time for cross, normal condition, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for cross, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, cross, normal condition, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement hill decreased l_time") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto l_time_reducer = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::zero();
      return point;
    };
    for(auto const& t:trains) {
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),l_time_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for hill, decreased l_time, id " <<t.id_<<" "<<time<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement intersection decreased l_time") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto l_time_reducer = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::zero();
      return point;
    };
    for(auto const& t:trains) {
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),l_time_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for intersection, decreased l_time, id " <<t.id_<<" "<<time<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel time measurement follow decreased l_time") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    auto l_time_reducer = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::zero();
      return point;
    };
    for(auto const& t:tt->trains_) {
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),l_time_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for follow, decreased l_time, id " <<t.id_<<" "<<time<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel cross time measurement decreased l_time") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto l_time_reducer = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.l_time_ = si::time::zero();
      return point;
    };
    for(auto const& t:tt->trains_) {
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),l_time_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for cross, decreased l_time, id " <<t.id_<<" "<<time<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel hill time measurement increased e_time") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto e_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.e_time_ = pt.e_time_*increase_time::ARRIVAL_FACTOR;
      point.l_time_ = si::time::infinity();
      return point;
    };
    for(auto const& t:trains) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),e_time_increaser,get_tpe_points,TRIALS);
      std::cout << "Time for hill, increased e_time, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for hill, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, hill, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel intersection time measurement increased e_time") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    auto e_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.e_time_ = pt.e_time_*increase_time::ARRIVAL_FACTOR;
      point.l_time_ = si::time::infinity();
      return point;
    };
    for(auto const& t:trains) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),e_time_increaser,get_tpe_points,TRIALS);
      std::cout << "Time for intersection, increased e_time, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for intersection, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, intersection, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel follow time measurement increased e_time") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    auto e_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.e_time_ = pt.e_time_*increase_time::ARRIVAL_FACTOR;
      point.l_time_ = si::time::infinity();
      return point;
    };
    for(auto const& t:tt->trains_) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),e_time_increaser,get_tpe_points,TRIALS);
      std::cout << "Time for follow, increased e_time, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for follow, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, follow, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel cross time measurement increased e_time") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto e_time_increaser = [](tpe_point const& pt) {
      tpe_point point(pt);
      point.e_time_ = pt.e_time_*increase_time::ARRIVAL_FACTOR;
      point.l_time_ = si::time::infinity();
      return point;
    };
    for(auto const& t:tt->trains_) {
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),e_time_increaser,get_tpe_points,TRIALS);
      std::cout << "Time for cross, increased e_time, id " <<t.id_<<" "<<time<< std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for cross, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, cross, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel hill time measurement increased v_min") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        if(pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt()) return pt;
        tpe_point point(pt);
        point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for hill, increased v_min, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for hill, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, hill, increased v_min, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel intersection time measurement increased v_min") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        if(pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt()) return pt;
        tpe_point point(pt);
        point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for intersection, increased v_min, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for intersection, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, intersection, increased v_min, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel follow time measurement increased v_min") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        if(pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt()) return pt;
        tpe_point point(pt);
        point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for follow, increased v_min, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for follow, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, follow, increased v_min, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel cross time measurement increased v_min") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        if(pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt()) return pt;
        tpe_point point(pt);
        point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for cross, increased v_min, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for cross, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, cross, increased v_min, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel hill time measurement decreased v_max") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto max_speed_reducer = [intervals,t](tpe_point const& pt) {
        if(pt.distance_.is_zero()) return pt;
        auto interval_point = std::ranges::find_if(intervals.p_,[pt](struct interval_point const& point){return point.distance_>=pt.distance_;});
        interval interval(&*(interval_point-1),&*interval_point);
        auto deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
        rk4::get_speed_limit get_speed_limit(interval.length(),t.physics_.max_speed(interval.speed_limit()),interval.target_speed(t.physics_),deaccel);
        auto speed = get_speed_limit(pt.distance_-interval.start_distance());
        tpe_point point(pt);
        point.v_max_ = speed*0.9;
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),max_speed_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for hill, train "<<t.id_<< " decreased v_max: " << time << std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for hill, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, hill, decreased v_max, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel intersection time measurement decreased v_max") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto max_speed_reducer = [intervals,t](tpe_point const& pt) {
        if(pt.distance_.is_zero()) return pt;
        auto interval_point = std::ranges::find_if(intervals.p_,[pt](struct interval_point const& point){return point.distance_>=pt.distance_;});
        interval interval(&*(interval_point-1),&*interval_point);
        auto deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
        rk4::get_speed_limit get_speed_limit(interval.length(),t.physics_.max_speed(interval.speed_limit()),interval.target_speed(t.physics_),deaccel);
        auto speed = get_speed_limit(pt.distance_-interval.start_distance());
        tpe_point point(pt);
        point.v_max_ = speed*0.9;
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),max_speed_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for intersection, train "<<t.id_<< " decreased v_max: " << time << std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for intersection, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, intersection, decreased v_max, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel follow time measurement decreased v_max") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto max_speed_reducer = [intervals,t](tpe_point const& pt) {
        if(pt.distance_.is_zero()) return pt;
        auto interval_point = std::ranges::find_if(intervals.p_,[pt](struct interval_point const& point){return point.distance_>=pt.distance_;});
        interval interval(&*(interval_point-1),&*interval_point);
        auto deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
        rk4::get_speed_limit get_speed_limit(interval.length(),t.physics_.max_speed(interval.speed_limit()),interval.target_speed(t.physics_),deaccel);
        auto speed = get_speed_limit(pt.distance_-interval.start_distance());
        tpe_point point(pt);
        point.v_max_ = speed*0.9;
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),max_speed_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for follow, train "<<t.id_<< " decreased v_max: " << time << std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for follow, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, follow, decreased v_max, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }
  TEST_CASE("tpe respecting travel cross time measurement decreased v_max") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto max_speed_reducer = [intervals,t](tpe_point const& pt) {
        if(pt.distance_.is_zero()) return pt;
        auto interval_point = std::ranges::find_if(intervals.p_,[pt](struct interval_point const& point){return point.distance_>=pt.distance_;});
        interval interval(&*(interval_point-1),&*interval_point);
        auto deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
        rk4::get_speed_limit get_speed_limit(interval.length(),t.physics_.max_speed(interval.speed_limit()),interval.target_speed(t.physics_),deaccel);
        auto speed = get_speed_limit(pt.distance_-interval.start_distance());
        tpe_point point(pt);
        point.v_max_ = speed*0.9;
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),max_speed_reducer,get_tpe_points,TRIALS);
      std::cout << "Time for cross, train "<<t.id_<< " decreased v_max: " << time << std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for cross, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, cross, decreased v_max, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel hill time measurement increased v_min increase e_time") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        auto is_halt = pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt();
        tpe_point point(pt);
        point.e_time_ = point.e_time_*increase_time::ARRIVAL_FACTOR;
        point.l_time_ = si::time::infinity();
        if(!is_halt) point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for hill, increased v_min, increased e_time, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for hill, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, hill, increased v_min, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel intersection time measurement increased v_min increase e_time") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        auto is_halt = pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt();
        tpe_point point(pt);
        point.e_time_ = point.e_time_*increase_time::ARRIVAL_FACTOR;
        point.l_time_ = si::time::infinity();
        if(!is_halt) point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for intersection, increased v_min, increased e_time, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for intersection, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, intersection, increased v_min, increased e_time, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel follow time measurement increased v_min increase e_time") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    vector<tt::train> trains{tt->trains_[1]};
    for(auto const& t:trains) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        auto is_halt = pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt();
        tpe_point point(pt);
        point.e_time_ = point.e_time_*increase_time::ARRIVAL_FACTOR;
        point.l_time_ = si::time::infinity();
        if(!is_halt) point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for follow, increased v_min, increased e_time, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for follow train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, follow, increased v_min, increased e_time id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }

  TEST_CASE("tpe respecting travel cross time measurement increased v_min increase e_time") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto intervals = get_intervals(t,type_set({type::HALT,type::EOTD}),infra);
      auto min_speed_increaser = [intervals](tpe_point const& pt) {
        auto point_interval = std::find_if(intervals.begin(),intervals.end(),[pt](interval const& interval) {return interval.end_distance()==pt.distance_;});
        auto is_halt = pt.distance_.is_zero()||point_interval!=intervals.end()&&
          point_interval.sequence_point().has_value()&&point_interval.sequence_point().value()->is_halt();
        tpe_point point(pt);
        point.e_time_ = point.e_time_*increase_time::ARRIVAL_FACTOR;
        point.l_time_ = si::time::infinity();
        if(!is_halt) point.v_min_ = si::speed(5);
        return point;
      };
      reset_duration();
      auto time = measure_tpe_performance(t,infra,type_set({type::HALT,type::EOTD}),min_speed_increaser,get_tpe_points,TRIALS);
      std::cout<<"Time for cross, increased v_min, train" <<t.id_<<" "<<time <<std::endl;
      auto increase_time_time = get_duration();
      auto count = get_count();
      std::cout << "Count for cross, train id "<<t.id_<<" "<<count<<std::endl;
      if(count!=0) std::cout << "Average increase time duration, cross, increased v_min, id "<<t.id_<<" "<<increase_time_time/count<< std::endl;
    }
  }


  std::chrono::microseconds average_shortest_travel_duration(infrastructure const& infra, tt::train const& train,type_set const& type_set) {
    auto duration = std::chrono::microseconds::zero();
    for(int i=0;i<100;++i) {
      auto start = std::chrono::high_resolution_clock::now();
      std::ignore = rk4::runtime_calculation(train,infra,type_set,use_surcharge::no);
      auto end = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    }
    return duration/100;
  }
  TEST_CASE("Hill shortest travel time performance") {
    infrastructure const infra(test::HILL_OPTS);
    tt::timetable const tt(test::HILL_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t:trains) {
      auto time = average_shortest_travel_duration(infra,t,type_set({type::HALT,type::EOTD}));
      std::cout << "Shortest drive hill, train_id: " <<t.id_ << " time: " << time << std::endl;
    }
  }

  TEST_CASE("intersection shortest travel time performance") {
    infrastructure const infra(test::INTER_OPTS);
    tt::timetable const tt(test::INTER_TT_OPTS, infra);
    vector<tt::train> trains{tt->trains_.begin(),tt->trains_.end()-1};
    for(auto const& t: trains) {
      auto time = average_shortest_travel_duration(infra,t,type_set({type::HALT,type::EOTD}));
      std::cout << "Shortest drive intersection, train_id: " <<t.id_ << " time: " << time << std::endl;
    }
  }

  TEST_CASE("follow shortest travel time performance") {
    infrastructure const infra(SMALL_OPTS);
    tt::timetable const tt(FOLLOW_OPTS, infra);
    for(auto const& t:tt->trains_) {
      auto time = average_shortest_travel_duration(infra,t,type_set({type::HALT,type::EOTD}));
      std::cout << "Shortest drive follow, train_id: " <<t.id_ << " time: " << time << std::endl;
    }
  }

  TEST_CASE("cross shortest travel time performance") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", CROSS_OPTS, infra);
    for(auto const& t: tt->trains_) {
      auto time = average_shortest_travel_duration(infra,t,type_set({type::HALT,type::EOTD}));
      std::cout << "Shortest drive cross, train_id: " <<t.id_ << " time: " << time << std::endl;
    }
  }

}
}// namespace soro::tpe_simulation
