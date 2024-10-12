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

#include "soro/runtime/common/train_path_envelope.h"
#include "soro/runtime/common/tpe_respecting_travel.h"
#include "soro/runtime/common/get_intervals.h"
#include "soro/runtime/physics/rk4/brake.h"
#include "soro/runtime/physics/rk4/detail/get_speed_limit.h"
#include "soro/runtime/common/increase_time.h"
#include "soro/runtime/common/phase_checkers.h"

namespace soro::runtime::test {

using namespace utl;
using namespace soro::tt;
using namespace soro::utls;
using namespace soro::infra;
using namespace soro::test;
using namespace soro::runtime::rk4;
using namespace soro::train_path_envelope;
TEST_SUITE("runtime suite") {

  void check_halt_count(train const& t, timestamps const& ts) {
    auto const timestamps_halt_count = utls::count_if(
        ts.times_, [](auto&& stamp) { return stamp.element_->is(type::HALT); });

    auto const train_halt_count = t.total_halts();

    // There should be as many halt events in the timestamps as there are halts
    // in the train run.
    CHECK_EQ(train_halt_count, timestamps_halt_count);
  }

  void check_ascending_timestamps(timestamps const& ts) {
    if (ts.times_.empty()) {
      uLOG(utl::warn) << "Checking ascending timestamps with empty timestamps.";
      return;
    }

    for (auto const [t1, t2] : utl::pairwise(ts.times_)) {
      // all times are valid
      CHECK(valid(t1.arrival_));
      CHECK(valid(t1.departure_));
      CHECK(valid(t2.arrival_));
      CHECK(valid(t2.departure_));

      // arrival must happen before/same time as departure
      CHECK_LE(t1.arrival_, t1.departure_);

      // times must be ordered
      CHECK_LE(t1.departure_, t2.arrival_);
    }
  }

  void check_delays(infrastructure const& infra, timetable const& tt) {
    uLOG(utl::info) << "Checking delays";
    soro::size_t too_early_count = 0;
    soro::size_t delayed_count = 0;
    soro::size_t total_count = 0;

    duration max_delay = duration::zero();
    duration max_too_early = duration::zero();

    auto avg_too_early = duration::zero();
    auto avg_delay = duration::zero();

    for (auto const& train : tt->trains_) {
      auto const timestamps = euler::runtime_calculation(
          train, infra, {type::HALT}, use_surcharge::no);

      if (timestamps.times_.empty()) {
        continue;
      }

      soro::size_t halt_id = 0;
      for (auto const& sp : train.sequence_points_) {
        if (!sp.is_halt()) {
          continue;
        }

        // halts must have arrival/departure times
        CHECK(sp.arrival_.has_value());
        CHECK(sp.departure_.has_value());

        auto const& ts =
            timestamps.times_[timestamps.type_indices_.at(type::HALT)[halt_id]];

        ++total_count;
        if (valid(ts.departure_) && ts.departure_ > *sp.departure_) {
          ++delayed_count;
          auto const delay = ts.departure_ - *sp.departure_;
          max_delay = std::max(max_delay, delay);
          avg_delay += ts.departure_ - *sp.departure_;
        }

        if (valid(ts.arrival_) && ts.arrival_ < *sp.arrival_) {
          ++too_early_count;
          auto const too_early = *sp.arrival_ - ts.arrival_;
          max_too_early = std::max(max_too_early, too_early);
          avg_too_early += *sp.arrival_ - ts.arrival_;
        }

        ++halt_id;
      }
    }

    uLOG(info) << "Total halt timestamps: " << total_count;

    uLOG(info) << "Total delayed timestamps: " << delayed_count;
    uLOG(info) << "Total over punctual timestamps: " << too_early_count;

    if (avg_delay != duration::zero()) {
      uLOG(info) << "AVG delay: " << avg_delay.count() / delayed_count;
    }

    if (avg_too_early != duration::zero()) {
      uLOG(info) << "AVG too early: "
                 << avg_too_early.count() / too_early_count;
    }

    uLOG(info) << "Maximum delay: " << max_delay.count();
    uLOG(info) << "Maximum over punctuality: " << max_too_early.count();
  }

  void check_timestamps_type_indices(timestamps const& ts) {
    for (auto const& [type, indices] : ts.type_indices_) {
      // every index given in type_indices_ must fit the type in the timestamps
      CHECK(utls::all_of(indices, [&ts, &type](auto&& idx) {
        return ts.times_[idx].element_->is(type);
      }));

      // every timestamp type must appear in type_indices_
      CHECK(utls::all_of(ts.times_, [&ts](auto&& t) {
        return ts.type_indices_.contains(t.element_->type());
      }));
    }
  }

  void check_event_existence_in_timestamps(train const& t, timestamps const& ts,
                                           infrastructure const& infra) {
    type_set const event_types(std::views::keys(ts.type_indices_));

    soro::size_t ts_idx = 0;
    for (auto const& rn : t.iterate(infra)) {
      if (rn.omitted() || !event_types.contains(rn.node_->type())) continue;

      bool const same_element =
          ts.times_[ts_idx].element_->get_id() == rn.node_->element_->get_id();

      CHECK_MESSAGE(
          same_element,
          "element from timestamps does not correspond to train run element");

      ++ts_idx;
    }

    CHECK_MESSAGE((ts_idx == ts.times_.size()),
                  "did not check every timestamp");
  }

  void determine_runtime_performance(infrastructure const& infra,
                                     timetable const& tt,
                                     type_set const& record_types) {
    utls::cumulative_timer euler_timer;
    utls::cumulative_timer rk4_timer;

    for (auto const& t : tt->trains_) {
      rk4_timer.start();
      auto const ts =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
      rk4_timer.stop();

      euler_timer.start();
      std::ignore =
          euler::runtime_calculation(t, infra, record_types, use_surcharge::no);
      euler_timer.stop();
    }

    CHECK_EQ(euler_timer.count(), rk4_timer.count());

    uLOG(info) << "Calculated " << rk4_timer.count() << " trains";
    uLOG(info) << "Average RK4 Time: " << rk4_timer.avg_duration()
               << "microseconds";
    uLOG(info) << "Average Euler Time: " << euler_timer.avg_duration()
               << "microseconds";
  }

  void check_runtime_with_events(infrastructure const& infra,
                                 timetable const& tt,
                                 type_set const& record_types) {
    for (auto const& t : tt->trains_) {
      auto const rk4_result =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);

      auto const euler_result =
          euler::runtime_calculation(t, infra, record_types, use_surcharge::no);

      if (record_types.contains(type::HALT)) {
        check_halt_count(t, euler_result);
        check_halt_count(t, rk4_result);
      }

      check_timestamps_type_indices(euler_result);
      check_ascending_timestamps(euler_result);
      check_event_existence_in_timestamps(t, euler_result, infra);

      check_timestamps_type_indices(rk4_result);
      check_ascending_timestamps(rk4_result);
      check_event_existence_in_timestamps(t, rk4_result, infra);
    }
  }

  void check_runtime(infrastructure const& infra, timetable const& tt) {
    auto const only_halt = type_set{type::HALT};
    check_runtime_with_events(infra, tt, only_halt);

    auto const simulation_types =
        type_set{type::RUNTIME_CHECKPOINT_UNDIRECTED, type::RUNTIME_CHECKPOINT,
                 type::APPROACH_SIGNAL, type::MAIN_SIGNAL, type::EOTD};
    check_runtime_with_events(infra, tt, simulation_types);

    check_runtime_with_events(infra, tt, type_set{all_types()});
  }

  void determine_runtime_performance(infrastructure const& infra,
                                     timetable const& tt) {
    uLOG(info) << "runtime performance while recording halts:";
    determine_runtime_performance(infra, tt, {type::HALT});
    uLOG(info) << "runtime performance while recording all types:";
    determine_runtime_performance(infra, tt, type_set{all_types()});
  }

  TEST_CASE("runtime") {
    for (auto const& scenario : soro::test::get_timetable_scenarios()) {
      check_runtime(*scenario->infra_, scenario->timetable_);
      check_delays(*scenario->infra_, scenario->timetable_);

      // when in debug mode skip runtime performance
#if defined(NDEBUG)
      determine_runtime_performance(*scenario->infra_, scenario->timetable_);
#endif
    }
  }

  constexpr auto runs = 100U;

  void test_euler(infrastructure const& infra, timetable const& tt) {
    for (auto i = 0U; i < runs; ++i) {
      for (auto const& t : tt->trains_) {
        if (t.id_ >= 1000) continue;

        auto const record_types = type_set{
            type::RUNTIME_CHECKPOINT_UNDIRECTED, type::RUNTIME_CHECKPOINT,
            type::APPROACH_SIGNAL, type::MAIN_SIGNAL, type::EOTD};

        auto const euler_result = euler::runtime_calculation(
            t, infra, record_types, use_surcharge::no);
      }
    }
  }

  void test_rk4(infrastructure const& infra, timetable const& tt) {
    for (auto i = 0U; i < runs; ++i) {
      for (auto const& t : tt->trains_) {
        if (t.id_ >= 1000) continue;

        auto const record_types = type_set{
            type::RUNTIME_CHECKPOINT_UNDIRECTED, type::RUNTIME_CHECKPOINT,
            type::APPROACH_SIGNAL, type::MAIN_SIGNAL, type::EOTD};

        auto const rk4_result =
            rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
      }
    }
  }

  tpe_points get_tpe_points(train const& t, infrastructure const& infra,
                                                       infra::type_set const& record_types){
    auto timestamps = runtime_calculation(t,infra,record_types,use_surcharge::no);
    tpe_points pts;
    for(auto e:timestamps.times_) {
      tpe_point point(e.dist_, si::time(e.arrival_.count()),
                      si::time(e.departure_.count()), si::speed::zero(),
                      si::speed::infinity());
      pts.push_back(point);
    }
    return pts;
  }
  tpe_points max_speed_reducer(train const& t, infrastructure const& infra,
                               infra::type_set const& record_types){
    auto points = get_tpe_points(t,infra,record_types);
    auto intervals = get_intervals(t,record_types,infra);
    auto interval = intervals.begin();
    for(int i=0;i<points.size();++i){
      while(interval.end_distance()<points[i].distance_) ++interval;
      auto deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
      get_speed_limit get_speed_limit(interval.length(), interval.speed_limit(t.physics_),
                      interval.target_speed(t.physics_), deaccel);
      auto max_speed = get_speed_limit(points[i].distance_-interval.start_distance());
      tpe_point new_point(points[i]);
      new_point.v_max_ = 0.9*max_speed;
      new_point.l_time_ = si::time::infinity();
      points[i] = new_point;
    }
    return points;
  }
  void check_merge_tpe(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types){
    for(auto const& t:trains){
      auto points = get_tpe_points(t,infra,record_types);
      std::sort(points.begin(),points.end());
      soro::tpe_simulation::merge_duplicate_tpe_points(points);
      if(!std::is_sorted(points.begin(),points.end())) throw std::logic_error("points arent sorted after merge");
      auto end = std::unique(points.begin(),points.end(),[](tpe_point const& pt1,tpe_point const& pt2){return pt1.distance_==pt2.distance_;});
      if(points.end()!=end){
        throw std::logic_error("check merge tpe failed");
      }
    }
  }
  void check_interval_iteration(intervals const& intervals){
    auto index = 0;
    for(auto const& interval:intervals){
      CHECK_EQ(interval.p1_,intervals.p_.data()+index);
      CHECK_EQ(interval.p2_,intervals.p_.data()+index+1);
      ++index;
    }
    for(auto i=0;i<intervals.p_.size();++i){
      CHECK_EQ(intervals.p_.data()+i,&intervals.p_[i]);
    }
  }

  void check_split_intervals(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types){
    for(auto const& t: trains){
      auto intervals = get_intervals(t,record_types,infra);
      auto pts = get_tpe_points(t,infra,record_types);
      std::sort(pts.begin(),pts.end());
      soro::tpe_simulation::merge_duplicate_tpe_points(pts);
      auto split_intervals = soro::tpe_simulation::split_intervals(intervals,pts,t.physics_);
      check_interval_iteration(split_intervals);
      if(utls::any_of(split_intervals.p_,[](interval_point const& pt){return pt.limit_.is_negative();}))throw std::logic_error("point has negative limit");
      if(!std::is_sorted(split_intervals.p_.begin(),split_intervals.p_.end(),[](interval_point const& p1,interval_point const& p2){return p1.distance_<p2.distance_;})) throw std::logic_error("Interval points arent ordered");
      for(auto const& interval:split_intervals){
        if(!std::is_sorted(interval.records().begin(),interval.records().end(),[](record const& r1,record const& r2){return r1.dist_<r2.dist_;})){
         throw std::logic_error("records arent sorted");
        }
        if(!interval.records().empty()&&interval.records().back().dist_>interval.end_distance()) throw std::logic_error("records too big");
      }
      for(auto const& pt:pts) {
        auto it = utls::find_if(split_intervals.p_,[pt](interval_point const& int_point){return int_point.distance_==pt.distance_;});
        if(it==split_intervals.p_.end()) throw std::logic_error("split_interval didnt work");
        auto it2 = utls::find_if(intervals.p_,[it](interval_point const& pt1){return pt1.distance_==it->distance_;});
        if(it2==intervals.p_.end()) {
          int count = std::count_if(split_intervals.p_.begin(),split_intervals.p_.end(),[it](interval_point const& ip){return ip.distance_==it->distance_;});
          if(count!=1) throw std::logic_error("split interval created point with same distance as another");
        }
      }
    }
  }

  void check_drive(increase_time::train_drive const& drive,int const& offset){
    if(offset>=drive.phases_.size()) return;
    auto unique_it = std::adjacent_find(drive.phase_types_.begin()+offset,drive.phase_types_.end());
    if(unique_it!=drive.phase_types_.end()) utl::fail("2 neighboring phases {} at offsets {} and {}",*unique_it,unique_it-drive.phase_types_.begin(),unique_it-drive.phase_types_.begin()+1);
    train_state predecessor;
    for(int i=offset;i<drive.phases_.size();++i){
      if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),[](train_state const& st1,train_state const& st2){return st1.dist_<st2.dist_;})){
        utl::fail("states arent ordered in distance");
      }
      if(drive.phases_[i].end()!=std::adjacent_find(drive.phases_[i].begin(),drive.phases_[i].end(),[](train_state const& st1,train_state const& st2){return st1.dist_==st2.dist_;})){
        utl::fail("states with same distance");
      }
      predecessor = offset>0?drive.phases_[offset-1].back():drive.start_state_;
      CHECK_EQ(drive.phases_[offset].front().dist_,predecessor.dist_);
      CHECK_EQ(drive.phases_[offset].front().speed_,predecessor.speed_);
      CHECK_EQ(drive.phases_[offset].front().time_,predecessor.time_);
      switch (drive.phase_types_[i]){
        case increase_time::acceleration:
          if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),
                              [](train_state const& state1,train_state const& state2){
                                return state1.speed_<state2.speed_;
                              })) throw std::logic_error("speed in acceleration is not ascending");
          break;
        case increase_time::cruising:
          CHECK_EQ(drive.phases_[i].size(),2);
          CHECK_EQ(drive.phases_[i].front().speed_,drive.phases_[i].back().speed_);
          if(!drive.phases_[i].front().speed_.is_zero())CHECK_EQ(drive.phases_[i].back().time_-drive.phases_[i].front().time_,increase_time::get_cruise_time(drive.phases_[i].front(),drive.phases_[i].back()));
          break;
        case increase_time::braking:
          if(!std::is_sorted(drive.phases_[i].begin(),drive.phases_[i].end(),
                              [](train_state const& state1,train_state const& state2){
                                return state1.speed_>state2.speed_;
                              })) throw std::logic_error("brake isnt descending in speed");
          break;
        default:throw std::logic_error("Invalid phase type detected");
      }
    }
  }

  void check_acceleration_possible(vector<train_state> const& accel,vector<interval_point> const& intr_point,train const& t){
    for(auto const& state: accel){
      auto it = utls::find_if(intr_point,[state](interval_point const& point){return point.distance_>=state.dist_;});
      utls::sassert(it!=intr_point.end(),"state has too high distance");
      if(it+1==intr_point.end()&&it->distance_==state.dist_) continue;
      interval current_interval  = it->distance_>state.dist_?interval(&*(it-1),&*it):interval(&*it,&*(it+1));
      auto tractive_force = t.physics_.tractive_force(state.speed_);
      auto resistive_force = t.physics_.resistive_force(state.speed_,current_interval.slope());
      if(resistive_force.is_negative()) std::cout<<"resistive force is negative"<<std::endl;
      utls::sassert(tractive_force>resistive_force.abs(),"acceleration not possible");
    }
  }
  void check_braking_possible(vector<train_state> const& brake,vector<interval_point> const& intr_point,train const& t){
    for(auto const& state:brake){
      if(state.dist_ == intr_point.back().distance_)continue;
      auto it = utls::find_if(intr_point,[state](interval_point const& point){return point.distance_>=state.dist_;});
      utls::sassert(it!=intr_point.end(),"state has too high distance");
      interval current_interval  = it->distance_>state.dist_?interval(&*(it-1),&*it):interval(&*it,&*(it+1));
      auto deaccel = t.physics_.braking_deaccel(current_interval.infra_limit(),current_interval.bwp_limit(),current_interval.brake_path_length());
      utls::sassert(deaccel.is_negative(),"positive deaccel");
    }
  }
  void check_cruise_possible(vector<train_state> const& cruise,vector<interval_point> const& intr_point,train const& t){
    auto& start_state = cruise.front();
    auto& end_state = cruise.back();
    auto speed = start_state.speed_;
    auto start_it = utls::find_if(intr_point,[start_state](interval_point const& point){return point.distance_>=start_state.dist_;});
    auto end_it = utls::find_if(intr_point,[end_state](interval_point const& point){return point.distance_>=end_state.dist_;});
    start_it = start_it->distance_ == start_state.dist_ ? start_it : start_it-1;
    while(start_it<end_it-1){
      interval interval(&*start_it,&*(start_it+1));
      auto tractive_force = t.physics_.tractive_force(speed);
      auto resistive_force = t.physics_.resistive_force(speed,start_it->slope_);
      auto braking_deaccel = t.physics_.braking_deaccel(interval.infra_limit(),interval.bwp_limit(),interval.brake_path_length());
      utls::sassert(tractive_force>=resistive_force,"tractive_force too weak, cruising is impossible");
      ++start_it;
    }
  }
  void check_drivable(increase_time::train_drive const& drive,vector<interval_point> const& intr_point,train const& t, int const& offset){
    for(int i=offset;i<drive.phases_.size();++i){
      auto phase = drive.phases_[i];
      switch (drive.phase_types_[i]){
        case increase_time::acceleration:
          check_acceleration_possible(phase,intr_point,t);
          break;
        case increase_time::braking:
          check_braking_possible(phase,intr_point,t);
          break;
        case increase_time::cruising:
          check_cruise_possible(phase,intr_point,t);
          break;
        default:
          throw std::logic_error("invalid type detected");
      }
    }
  }

  void check_get_end_state(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types,
                           std::function<tpe_point(tpe_point const&)> const& tpe_changer,
                           std::function<tpe_points(train const&,infrastructure const&,infra::type_set const&)> const& points_maker=get_tpe_points) {
    for (auto const& t : trains) {
      auto points = points_maker(t, infra, record_types);
      std::sort(points.begin(), points.end(),
                [](tpe_point const& pt1, tpe_point const& pt2) {
                  return pt1 < pt2;
                });
      soro::tpe_simulation::merge_duplicate_tpe_points(points);
      for(int i=0;i<points.size();++i) points[i] = tpe_changer(points[i]);
      auto intervals = get_intervals(t, record_types, infra);
      auto split_intervals = soro::tpe_simulation::split_intervals(
            intervals, points, t.physics_);
        if(t.start_speed_>points.front().v_max_&&points.front().distance_.is_zero()) throw std::logic_error("Train starts too fast");
        train_state state;
        state.speed_ = t.start_speed_;
        state.dist_ = si::length::zero();
        state.time_ = si::time(t.start_time_.count());
        auto interval = split_intervals.begin();
        train::trip const trip(train::trip::id{0}, t.id_, ZERO<absolute_time>);

        for (auto const& pt : points) {
          increase_time::train_drive drive;
          std::tie(state,drive) = soro::tpe_simulation::get_end_state(state, pt, interval,
                                                      nullptr, t, trip);
          CHECK_EQ(state.dist_, pt.distance_);
          CHECK_GE(state.speed_, pt.v_min_);
          CHECK_LE(state.speed_, pt.v_max_);
          CHECK_EQ(state.dist_,interval.start_distance());

          check_drive(drive,0);
          check_drivable(drive,split_intervals.p_,t,0);
        }
      }
    }
    void check_tpe_respecting_travel(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types,
                                     std::function<tpe_point(tpe_point const&)> const& tpe_changer,
                                     std::function<tpe_points(train const&,infrastructure const&,infra::type_set const&)> const& points_maker=get_tpe_points){
      for(auto const& t:trains){
        auto points = points_maker(t,infra,record_types);
        auto intervals = get_intervals(t,record_types,infra);
        for(auto i=0;i<points.size();++i) points[i]=tpe_changer(points[i]);
        train_state initial;
        initial.dist_ = intervals.begin().start_distance();
        initial.time_ = si::time(t.start_time_.count());
        initial.speed_ = t.start_speed_;
        train::trip const trip(train::trip::id{0}, t.id_, ZERO<absolute_time>);
        auto results = soro::tpe_simulation::tpe_respecting_simulation(points,initial,nullptr,t,trip,infra,record_types);
        CHECK_EQ(results.size(),points.size());
        for(int i=0;i<results.size();++i){
          CHECK_EQ(results[i].dist_,points[i].distance_);
          CHECK_LE(results[i].speed_,points[i].v_max_);
          CHECK_GE(results[i].speed_,points[i].v_min_);
        }
      }
    }
    void check_drive_creation(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types){
      shortest_travel_time shortest_travel_time;
      for(auto const& t:trains){
        auto tpe_points = get_tpe_points(t,infra,record_types);
        tpe_simulation::merge_duplicate_tpe_points(tpe_points);
        auto intervals = tpe_simulation::split_intervals(get_intervals(t,record_types,infra),tpe_points,t.physics_);
        auto interval = intervals.begin();
        train_state current;
        current.time_ = si::time(t.start_time_.count());
        current.dist_ = si::length::zero();
        current.speed_ = t.start_speed_;
        increase_time::train_drive drive;
        drive.start_state_ = current;
        signal_time const signal_time;
        train::trip const trip(train::trip::id{0}, t.id_, ZERO<absolute_time>);
        while(interval!=intervals.end()){
          auto drive_size = drive.phase_types_.size();
          auto [delta,delta_drive] = shortest_travel_time.create_drive(current,nullptr,interval,t,trip,signal_time);
          current+=delta;
          drive+=delta_drive;
          CHECK_EQ(current.dist_,interval.end_distance());
          CHECK_EQ(delta_drive.phases_.size(),delta_drive.phase_types_.size());
          if(!drive.phases_.empty()) {
            CHECK_EQ(drive.phases_.back().back().dist_,interval.end_distance());
          }
          CHECK_LE(delta_drive.phases_.size(),3);
          check_drive(drive,drive_size>0?drive_size-1:0);
          ++interval;
        }
      }
    }
    int next_index(increase_time::train_drive const& drive,int const& start_index,increase_time::phase_checker const& checker){
      for(int i=start_index;i<drive.phases_.size()-1;++i){
        auto t1 = drive.phase_types_[i];
        auto t2 = drive.phase_types_[i+1];
        auto t3 = i+2<drive.phases_.size()?drive.phase_types_[i+2]:increase_time::invalid;
        if(checker(t1,t2,t3)) return i;
      }
      return -1;
    }
    void AHD_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,train const& t){
      if(drive.phases_.empty()) return;
      increase_time::set_pt(point);
      int index = next_index(drive,0,increase_time::AHD_checker);
      while(index!=-1){
        bool finished = increase_time::check_AHD(drive,index);
        CHECK_GE(drive.phases_.size(),1);
        check_drive(drive,0);
        utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_AHD returned wrong result {}",finished);
        check_drivable(drive,intr_points,t,0);
        if(finished) return;
        index= next_index(drive,0,increase_time::AHD_checker);
      }
    }
    void AHA_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,train const& t){
      if(drive.phases_.empty()) return;
      increase_time::set_pt(point);
      increase_time::set_intervals(intr_points);
      int index = next_index(drive,0,increase_time::AHA_checker);
      while(index!=-1){
        bool finished = increase_time::check_AHA(drive,t.physics_,index);
        check_drive(drive,index);
        utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_AHA returned wrong result {}",finished);
        check_drivable(drive,intr_points,t,index);
        if(finished) return;
        index=next_index(drive,0,increase_time::AHA_checker);
      }
    }
    void HDH_check(increase_time::train_drive& drive,tpe_point const& point,vector<interval_point> const& intr_points,train const& t){
      if(drive.phases_.empty()) return;
      increase_time::set_pt(point);
      increase_time::set_intervals(intr_points);
      int index = next_index(drive,0,increase_time::HDH_checker);
      while(index!=-1){
        bool finished = increase_time::check_HDH(drive,t.physics_,index);
        check_drive(drive,index);
        utl::verify(finished==(drive.phases_.back().back().time_>=point.e_time_),"check_HDH returned wrong result {}",finished);
        check_drivable(drive,intr_points,t,index);
        if(finished) return;
        index=next_index(drive,0,increase_time::HDH_checker);
      }
    }
    void test_check_function(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types,int function_to_use) {
      shortest_travel_time shortest_travel_time;
      signal_time const signal_time;
      auto const ARRIVAL_FACTOR = 1.1;
      for (auto const& t : trains) {
        auto tpe_points = get_tpe_points(t, infra, record_types);
        tpe_simulation::merge_duplicate_tpe_points(tpe_points);
        auto intervals = tpe_simulation::split_intervals(
            get_intervals(t, record_types, infra), tpe_points, t.physics_);
        train_state current;
        current.time_ = si::time(t.start_time_.count());
        current.dist_ = si::length::zero();
        current.speed_ = t.start_speed_;
        increase_time::train_drive drive;
        drive.start_state_ = current;
        train::trip const trip(train::trip::id{0}, t.id_, ZERO<absolute_time>);
        auto point_index = intervals.begin().length().is_zero() ? 0 : 1;
        for (auto const& interval : intervals) {
          auto [delta, delta_drive] = shortest_travel_time.create_drive(
              current, nullptr, interval, t, trip, signal_time);
          drive += delta_drive;
          current += delta;
          if (interval.end_distance() == tpe_points[point_index].distance_) {
            tpe_points[point_index].e_time_ = tpe_points[point_index].e_time_*ARRIVAL_FACTOR;
            tpe_points[point_index].l_time_ = std::max(tpe_points[point_index].l_time_,tpe_points[point_index].e_time_);
            if(function_to_use==0) AHD_check(drive,tpe_points[point_index],intervals.p_,t);
            if(function_to_use==1) AHA_check(drive,tpe_points[point_index],intervals.p_,t);
            if(function_to_use==2) HDH_check(drive,tpe_points[point_index],intervals.p_,t);
            drive.erase_elements(0, drive.phases_.size());
            drive.start_state_ = current;
            ++point_index;
          }
        }
      }
    }

    auto changer = [](tpe_point const& pt){
      tpe_point point(pt);
      point.l_time_ = si::time::infinity();
      return point;
    };
    auto latest_time_reducer = [](tpe_point const& point){
      tpe_point new_point(point);
      new_point.l_time_ = point.l_time_*0.9;
      if(new_point.e_time_>new_point.l_time_) new_point.e_time_ = new_point.l_time_;
      return new_point;
    };

  TEST_CASE("runtime iss") {
    auto const infra = utls::try_deserializing<infrastructure>(
        "de_iss_runtime.raw", DE_ISS_OPTS);
    auto const tt = utls::try_deserializing<timetable>("de_kss_runtime.raw",
                                                       DE_KSS_OPTS, infra);

    test_euler(infra, tt);
    test_rk4(infra, tt);

    //    check_runtime(infra, tt);
    //    check_delays(infra, tt);
  }
  TEST_CASE("print split intervals"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto train = tt->trains_[0];
    auto tpe_points = get_tpe_points(train,infra,infra::type_set({type::HALT,type::EOTD}));
    tpe_simulation::merge_duplicate_tpe_points(tpe_points);
    auto intervals = get_intervals(train,infra::type_set({type::HALT,type::EOTD}),infra);
    for(auto const& interval: intervals){
      std::cout<<interval.end_distance()<<" "<<interval.target_speed(train.physics_)<<std::endl;
    }
    intervals = tpe_simulation::split_intervals(intervals,tpe_points,train.physics_);
    for(auto const& interval: intervals){
      std::cout<<interval.end_distance()<<" "<<interval.target_speed(train.physics_)<<std::endl;
    }
  }
  TEST_CASE("runtime hill") {
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);

    CHECK_EQ(tt->trains_.size(), 4);

    auto const record_types = infra::type_set{type::HALT};

    auto const duration = [](auto&& times) {
      return (times.back().arrival_ - times.front().departure_).count();
    };

    SUBCASE("sufficient brake weight percentage") {
      auto const& t = tt->trains_[0];
      CHECK_EQ(t.physics_.percentage(), rs::brake_weight_percentage{150});

      auto const euler_no =
          euler::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const euler_yes = euler::runtime_calculation(t, infra, record_types,
                                                        use_surcharge::yes);

      auto const rk4_no =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const rk4_yes =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::yes);

      CHECK_EQ(duration(euler_no.times_), 428);
      CHECK_EQ(duration(rk4_no.times_), 428);

      CHECK_EQ(duration(euler_yes.times_), 440);
      CHECK_EQ(duration(rk4_yes.times_), 440);
    }

    SUBCASE(
        "insufficient brake weight percentage - reduced max speed on slope") {
      auto const& t = tt->trains_[1];
      CHECK_EQ(t.physics_.percentage(), rs::brake_weight_percentage{67});

      auto const euler_no =
          euler::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const euler_yes = euler::runtime_calculation(t, infra, record_types,
                                                        use_surcharge::yes);

      auto const rk4_no =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const rk4_yes =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::yes);

      CHECK_EQ(duration(euler_no.times_), 468);
      CHECK_EQ(duration(rk4_no.times_), 467);

      CHECK_EQ(duration(euler_yes.times_), 481);
      CHECK_EQ(duration(rk4_yes.times_), 481);
    }

    SUBCASE("insufficient brake weight percentage - always lowered max speed") {
      auto const& t = tt->trains_[2];
      CHECK_EQ(t.physics_.percentage(), rs::brake_weight_percentage{40});

      auto const euler_no =
          euler::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const euler_yes = euler::runtime_calculation(t, infra, record_types,
                                                        use_surcharge::yes);

      auto const rk4_no =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
      auto const rk4_yes =
          rk4::runtime_calculation(t, infra, record_types, use_surcharge::yes);

      CHECK_EQ(duration(euler_no.times_), 803);
      CHECK_EQ(duration(rk4_no.times_), 802);

      CHECK_EQ(duration(euler_yes.times_), 826);
      CHECK_EQ(duration(rk4_yes.times_), 826);
    }

    SUBCASE("insufficient brake weight percentage - not possible") {
      auto const& train = tt->trains_[3];
      CHECK_EQ(train.physics_.percentage(), rs::brake_weight_percentage{30});

      CHECK_THROWS(euler::runtime_calculation(train, infra, {type::HALT},
                                              use_surcharge::no));
      CHECK_THROWS(rk4::runtime_calculation(train, infra, {type::HALT},
                                            use_surcharge::no));
    }
  }
  TEST_CASE("runtime intersection") {
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);

    CHECK_EQ(tt->trains_.size(), 4);

    auto const record_types = infra::type_set{type::HALT};

    auto const duration = [](auto&& times) {
      return (times.back().arrival_ - times.front().departure_).count();
    };

    auto const& t = tt->trains_[0];
    CHECK_EQ(t.physics_.percentage(), rs::brake_weight_percentage{150});

    auto const euler_no =
        euler::runtime_calculation(t, infra, record_types, use_surcharge::no);
    auto const euler_yes =
        euler::runtime_calculation(t, infra, record_types, use_surcharge::yes);

    auto const rk4_no =
        rk4::runtime_calculation(t, infra, record_types, use_surcharge::no);
    auto const rk4_yes =
        rk4::runtime_calculation(t, infra, record_types, use_surcharge::yes);

    CHECK_EQ(duration(euler_no.times_), 506);
    CHECK_EQ(duration(rk4_no.times_), 505);

    CHECK_EQ(duration(euler_yes.times_), 522);
    CHECK_EQ(duration(rk4_yes.times_), 520);
  }

  TEST_CASE("runtime follow") {
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);

    check_runtime(infra, tt);
    check_delays(infra, tt);
  }

  TEST_CASE("runtime cross") {
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);

    check_runtime(infra, tt);
    check_delays(infra, tt);
  }
  TEST_CASE("get_end_state hill") {
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    check_get_end_state({tt->trains_[0]}, infra,
                        infra::type_set({type::HALT, type::EOTD}), changer);
    // TODO: der Fall funktioniert, aber ich muss schauen wie ich ihn fange
    // check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),latest_time_reducer);
    auto identity = [](tpe_point const& pt){return pt;};
    check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),identity,max_speed_reducer);
  }
  TEST_CASE("get_end_state intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),changer);
    //TODO: der Fall funktioniert, aber ich muss schauen wie ich ihn fange
    //CHECK_THROWS(check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),latest_time_reducer));
    auto identity = [](tpe_point const& pt){return pt;};
    check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),identity,max_speed_reducer);
  }
  TEST_CASE("get_end_state follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    check_get_end_state(tt->trains_,infra,type_set({type::HALT,type::EOTD}),changer);
    //TODO: siehe oben
    //CHECK_THROWS(check_get_end_state(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),latest_time_reducer));
    auto identity = [](tpe_point const& pt){return pt;};
    check_get_end_state(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),identity,max_speed_reducer);
  }
  TEST_CASE("get_end_state cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_get_end_state(tt->trains_,infra,type_set({type::HALT,type::EOTD}),changer);
    //TODO: siehe oben
    //CHECK_THROWS(check_get_end_state(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),latest_time_reducer));
    auto identity = [](tpe_point const& pt){return pt;};
    check_get_end_state(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),identity,max_speed_reducer);
  }
  TEST_CASE("tpe_respecting_travel hill") {
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    auto e_time_changer = [](tpe_point const& pt){tpe_point point(pt);
      point.e_time_ = point.e_time_*0.9;
      return point;};
    check_tpe_respecting_travel({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),e_time_changer);
  }
  TEST_CASE("tpe_respecting_travel intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    check_tpe_respecting_travel({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),changer);
  }
  TEST_CASE("tpe_respecting_travel follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    auto e_time_changer = [](tpe_point const& pt){tpe_point point(pt);
      point.e_time_ = point.e_time_*0.9;
      point.l_time_ = si::time::infinity();
      return point;};
    check_tpe_respecting_travel(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),e_time_changer);
  }
  TEST_CASE("tpe_resepcting_travel cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto e_time_changer = [](tpe_point const& pt){tpe_point point(pt);
      point.e_time_ = point.e_time_*0.9;
      point.l_time_ = si::time::infinity();
      return point;};
    check_tpe_respecting_travel(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),e_time_changer);
  }
  TEST_CASE("drive creation hill"){
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    check_drive_creation({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("drive creation intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    check_drive_creation({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("drive creation follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    check_drive_creation(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("drive creation cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_drive_creation(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("increase time check methods hill"){
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    auto train = tt->trains_[0];
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),0);
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),1);
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),2);
  }
  TEST_CASE("increase time check methods intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    auto train = tt->trains_[0];
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),0);
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),1);
    test_check_function({train},infra,infra::type_set({type::HALT,type::EOTD}),2);
  }
  TEST_CASE("increase time check methods follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),0);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),1);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),2);
  }
  TEST_CASE("increase time check methods cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),0);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),1);
    test_check_function(tt->trains_,infra,infra::type_set({type::HALT,type::EOTD}),2);
  }
  TEST_CASE("fix_intervals test"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto train = tt->trains_[0];
    auto intervals = get_intervals(train,infra::type_set({type::HALT,type::EOTD}),infra);
    utls::for_each(intervals.p_,[](interval_point const& pt){std::cout<<pt.distance_<<" "<<pt.limit_<<std::endl;});
    std::cout<<std::endl;
    auto points = max_speed_reducer(train,infra,infra::type_set({type::HALT,type::EOTD}));
    tpe_simulation::merge_duplicate_tpe_points(points);
    auto split_intervals = tpe_simulation::split_intervals(intervals,points,train.physics_);
    utls::for_each(intervals.p_,[](interval_point const& pt){std::cout<<pt.distance_<<" "<<pt.limit_<<std::endl;});
    std::cout<<std::endl;
    for(auto const& point:points) {
      auto start_interval = split_intervals.begin();
      auto vector = soro::tpe_simulation::fix_intervals(start_interval,
                                                        point, train.physics_);
      if (utls::any_of(vector, [](interval_point const& pt) {
            return pt.distance_.is_negative();
          })) {
        throw std::logic_error("Negative distance for interval point");
      }
      utls::for_each(vector,[](interval_point const& pt){std::cout<<pt.distance_<<" "<<pt.limit_<<std::endl;});
      std::cout<<std::endl;
    }
  }
  TEST_CASE("max_speed_reducer"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto points = max_speed_reducer(tt->trains_[0],infra,infra::type_set({type::HALT,type::EOTD}));
    for(auto const& pt:points){
      std::cout<<pt.distance_<<" ";
      std::cout<<pt.v_max_<<std::endl;
    }
  }
  TEST_CASE("drive addition simple test"){
    increase_time::train_drive drive;
    if(!drive.phase_types_.empty()||!drive.phases_.empty()) throw std::logic_error("One of the vectors arent empty at the beginning");
    if(!(drive.start_state_.dist_.is_zero()&&drive.start_state_.time_.is_zero()&&drive.start_state_.speed_.is_zero())) throw std::logic_error("state isnt initialited as zero");
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
    auto empty_drive = increase_time::train_drive::empty();
    drive+=empty_drive;
    CHECK_EQ(drive.phases_.size(),3);
    CHECK_EQ(drive.phase_types_.size(),3);
    increase_time::train_drive add_drive;
    add_drive.push_back(fake_acceleration,increase_time::acceleration);
    add_drive.push_back(fake_brake,increase_time::braking);
    drive+=add_drive;
    CHECK_EQ(drive.phases_.size(),5);
    CHECK_EQ(drive.phase_types_.size(),5);
    check_drive(drive,2);
    add_drive.erase_elements(0,2);
    add_drive.push_back({train_state::zero(),train_state::zero()},increase_time::braking);
    drive+=add_drive;
    CHECK_EQ(drive.phase_types_.size(),5);
  }
  TEST_CASE("check_AHD simple test"){
    increase_time::train_drive drive;
    train_state one_delta(si::time(1),si::length(1),si::speed(1));
    train_state start_state;
    start_state.speed_ = si::speed(1);
    auto end_accel = start_state+one_delta;
    auto fake_acceleration = {start_state,end_accel};
    one_delta.speed_ = si::speed::zero();
    auto end_cruise = end_accel+one_delta;
    auto fake_cruise= {end_accel,end_cruise};
    one_delta.speed_ = si::speed(-1);
    auto end_brake = end_cruise + one_delta;
    end_brake.time_ = si::time(2.5);
    auto fake_brake = {end_cruise,end_brake};
    drive.push_back(fake_acceleration,increase_time::acceleration);
    drive.push_back(fake_cruise,increase_time::cruising);
    drive.push_back(fake_brake,increase_time::braking);
    tpe_point point(si::length(3),si::time(3),si::time::infinity(),si::speed::zero(),si::speed::infinity());
    increase_time::set_pt(point);
    bool result = increase_time::check_AHD(drive,0);
    utl::verify(result,"AHD returned false when it shouldnt have");
    CHECK_EQ(drive.phases_.size(),1);
    CHECK_EQ(drive.phases_.size(),drive.phase_types_.size());
    CHECK_EQ(drive.phase_types_.front(),increase_time::cruising);
    CHECK_EQ(drive.phases_.front().size(),2);
    auto state1 = drive.phases_.front().front();
    auto state2 = drive.phases_.front().back();
    CHECK_EQ(state1.speed_,state2.speed_);
    CHECK_EQ(state2.speed_.val_,1);
    CHECK_EQ(state2.dist_.val_,3);

    drive.erase_elements(0,drive.phases_.size());
    train_state mid_accel(si::time(0.5),si::length(0.5),si::speed(1.5));
    end_cruise.dist_= si::length(4);
    end_cruise.time_ = si::time(2);
    train_state mid_brake(si::time(2.5),si::length(4.5),si::speed(1.5));
    end_brake.dist_ = si::length(5);
    end_brake.time_ = si::time(3);
    point.e_time_ = si::time(3+(2.0/3.0));
    increase_time::set_pt(point);
    drive.push_back({start_state,mid_accel,end_accel},increase_time::acceleration);
    drive.push_back({end_accel,end_cruise},increase_time::cruising);
    drive.push_back({end_cruise,mid_brake,end_brake},increase_time::braking);
    result = check_AHD(drive,0);
    utls::sassert(result,"returned false even though it should be true");
    CHECK_EQ(drive.phases_.size(),drive.phase_types_.size());
    CHECK_EQ(drive.phases_.size(),3);
    CHECK_EQ(drive.phase_types_,increase_time::types{increase_time::acceleration,increase_time::cruising,increase_time::braking});
    CHECK_EQ(drive.phases_.front().size(),2);
    CHECK_EQ(drive.phases_[1].size(),2);
    CHECK_EQ(drive.phases_.back().size(),2);
  }
  TEST_CASE("pt test"){
    /*auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);*/
    /*infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    auto points = get_tpe_points(tt->trains_[0],infra,type_set({type::HALT,type::EOTD}));
    auto timestamps = runtime_calculation(tt->trains_[0],infra,type_set({type::HALT,type::EOTD}),use_surcharge::no);
    std::cout<<"Halts"<<std::endl;
    std::for_each(timestamps.type_indices_[type::HALT].begin(),timestamps.type_indices_[type::HALT].end(),[points](auto const& i){std::cout<<points[i].distance_<<std::endl;});
    std::cout<<"EOTD"<<std::endl;
    std::for_each(timestamps.type_indices_[type::EOTD].begin(),timestamps.type_indices_[type::EOTD].end(),[points](auto const& i){std::cout<<points[i].distance_<<std::endl;});*/
    infrastructure const infra2(INTER_OPTS);
    timetable const tt2(INTER_TT_OPTS, infra2);
    auto points2 = get_tpe_points(tt2->trains_[0],infra2,type_set({type::HALT,type::EOTD}));
    auto timestamps2 = runtime_calculation(tt2->trains_[0],infra2,type_set({type::HALT,type::EOTD}),use_surcharge::no);
    std::cout<<"Halts"<<std::endl;
    std::for_each(timestamps2.type_indices_[type::HALT].begin(),timestamps2.type_indices_[type::HALT].end(),[points2](auto const& i){std::cout<<points2[i].distance_<<std::endl;});
    std::cout<<"EOTD"<<std::endl;
    std::for_each(timestamps2.type_indices_[type::EOTD].begin(),timestamps2.type_indices_[type::EOTD].end(),[points2](auto const& i){std::cout<<points2[i].distance_<<std::endl;});
  }
  TEST_CASE("split intervals hill"){
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    check_split_intervals({tt->trains_[0]},infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("split intervals intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    check_split_intervals({tt->trains_[0]},infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("split intervals follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    check_split_intervals(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("split intervals cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_split_intervals(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("merge tpe_points intersection"){
    infrastructure const infra(INTER_OPTS);
    timetable const tt(INTER_TT_OPTS, infra);
    check_merge_tpe({tt->trains_[0]},infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("merge tpe_points follow"){
    infrastructure const infra(SMALL_OPTS);
    timetable const tt(FOLLOW_OPTS, infra);
    check_merge_tpe(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("merge tpe_points hill"){
    infrastructure const infra(HILL_OPTS);
    timetable const tt(HILL_TT_OPTS, infra);
    soro::vector<train> trains = {tt->trains_.begin(),tt->trains_.end()-1};
    std::cout<<trains.size()<<std::endl;
    check_merge_tpe({trains[0]},infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("merge tpe_points cross"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    check_merge_tpe(tt->trains_,infra,type_set({type::HALT,type::EOTD}));
  }
  TEST_CASE("merge tpe_points"){
    auto pt1 = train_path_envelope::tpe_point(si::length::zero(),si::time{100},si::time{200},si::speed{0},si::speed{200});
    auto pt2 = train_path_envelope::tpe_point(si::length{100},si::time{100},si::time{200},si::speed{50},si::speed{300});
    auto pt3 = train_path_envelope::tpe_point(si::length{200},si::time{100},si::time{200},si::speed{2},si::speed{200});
    auto pt4 = train_path_envelope::tpe_point(si::length{250},si::time{100},si::time{200},si::speed{3},si::speed{200});
    train_path_envelope::tpe_points pts = {pt1,pt2,pt3,pt4};
    train_path_envelope::tpe_points pts_check = {pt1,pt2,pt3,pt4};
    soro::tpe_simulation::merge_duplicate_tpe_points(pts);
    if(pts!=pts_check) {
      std::for_each(pts.begin(),pts.end(),[](train_path_envelope::tpe_point pt){std::cout<<pt.distance_<<std::endl;});
      throw std::logic_error("pts was unexpectedly changed");
    }
    pt2.distance_ = si::length::zero();
    pt2.v_max_ = si::speed(300);
    pt2.v_min_ = si::speed(50);
    soro::vector<soro::train_path_envelope::tpe_point> pts2 = {pt1,pt2,pt3,pt4};
    soro::tpe_simulation::merge_duplicate_tpe_points(pts2);
    if(pts2.size()!=3) {
      std::cout<<pts2.size()<<std::endl;
      std::for_each(pts2.begin(),pts2.end(),[](train_path_envelope::tpe_point pt){std::cout<<pt.distance_<<std::endl;});
      throw std::logic_error("pts2 has wrong size when changing pt2");
    }
    CHECK_EQ(pts2[0],train_path_envelope::tpe_point(si::length::zero(),si::time{100},si::time{200},si::speed{50},si::speed{200}));
    CHECK_EQ(pts2[1],pt3);
    CHECK_EQ(pts2[2],pt4);
    soro::vector<soro::train_path_envelope::tpe_point> pts3 = {pt1,pt1,pt1,pt3,pt4};
    soro::tpe_simulation::merge_duplicate_tpe_points(pts3);
    if(pts3.size()!=3){
      std::cout<<pts.size()<<std::endl;
      throw std::logic_error("pts has wrong size with 3 pt1");
    }
    CHECK_EQ(pts3[0],pt1);
    CHECK_EQ(pts3[1],pt3);
    CHECK_EQ(pts3[2],pt4);
  }
  TEST_CASE("sort test tpe"){
    auto pt1 = train_path_envelope::tpe_point(si::length::zero(),si::time{100},si::time{200},si::speed{0},si::speed{200});
    auto pt2 = train_path_envelope::tpe_point(si::length::zero(),si::time{100},si::time{200},si::speed{50},si::speed{300});
    auto pt3 = train_path_envelope::tpe_point(si::length{200},si::time{100},si::time{200},si::speed{2},si::speed{200});
    auto pt4 = train_path_envelope::tpe_point(si::length{250},si::time{100},si::time{200},si::speed{3},si::speed{200});
    //for(int i=0;i<100;++i){

      soro::vector<train_path_envelope::tpe_point> points = {pt4,pt2,pt3,pt1};
      std::sort(points.begin(),points.end(),[](train_path_envelope::tpe_point e1, train_path_envelope::tpe_point e2) { return e1 < e2; });
      std::for_each(points.begin(),points.end(),[](train_path_envelope::tpe_point pt){std::cout<<pt.distance_<<std::endl;});
      if(!std::is_sorted(points.begin(),points.end(),[](train_path_envelope::tpe_point e1, train_path_envelope::tpe_point e2) { return e1 < e2; }))throw std::logic_error("L");
    //}
  }
  TEST_CASE("test split interval"){
    auto const infra =
        utls::try_deserializing<infrastructure>("small_opts.raw", SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<timetable>("cross_opts.raw", CROSS_OPTS, infra);
    auto train = tt->trains_.front();
    auto route_intervals =
        get_intervals(train, soro::infra::type_set::all(), infra);
    auto first_dist = route_intervals.p_.front().distance_;
    auto last_dist = route_intervals.p_.back().distance_;
    auto pt1 = train_path_envelope::tpe_point(first_dist,si::time{100},si::time{200},si::speed{0},si::speed{200});
    auto pt2 = train_path_envelope::tpe_point(first_dist+(last_dist-first_dist)/3,si::time{100},si::time{200},si::speed{50},si::speed{300});
    auto pt3 = train_path_envelope::tpe_point(first_dist+2*(last_dist-first_dist)/3,si::time{100},si::time{200},si::speed{2},si::speed{200});
    auto pt4 = train_path_envelope::tpe_point(last_dist,si::time{100},si::time{200},si::speed{3},si::speed{200});
    soro::vector<train_path_envelope::tpe_point> points = {pt1,pt2,pt3,pt4};
    auto intervals = soro::tpe_simulation::split_intervals(route_intervals,points,train.physics_);
    for(auto pt : points){
      if(!utls::any_of(intervals.p_,[pt](auto int_p){return int_p.distance_==pt.distance_;})){
        throw std::logic_error("Found no point with distance "+std::to_string(pt.distance_.val_));
      }
    }
  }
}
TEST_CASE("Omar Runtime"){
  std::cout<<"Hallo"<<std::endl;
}
TEST_CASE("Test erase"){
  soro::vector<int> numbers = {1,2,3,3,4,5};
  auto it = numbers.begin();
  numbers.erase(it,it+2);
  std::for_each(numbers.begin(),numbers.end(),[](int num){std::cout<<num<<std::endl;});
}
TEST_CASE("Test insert"){
  soro::vector<int> numbers = {1,2,3,4,5};
  auto it = numbers.begin()+3;
  numbers.insert(it,3);
  std::for_each(numbers.begin(),numbers.end(),[](int num){std::cout<<num<<std::endl;});
}
}  // namespace soro::runtime::test
