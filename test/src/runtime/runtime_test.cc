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

  void check_get_end_state(soro::vector<train> const& trains,infrastructure const& infra,infra::type_set const& record_types,
                           std::function<tpe_point(tpe_point const&)> const& tpe_changer,
                           std::function<tpe_points(train const&,infrastructure const&,infra::type_set const&)> const& points_maker=get_tpe_points) {
    for (auto const& t : trains) {
        auto points = points_maker(t, infra, record_types);
        auto intervals = get_intervals(t, record_types, infra);
        std::sort(points.begin(), points.end(),
                  [](tpe_point const& pt1, tpe_point const& pt2) {
                    return pt1 < pt2;
                  });
        soro::tpe_simulation::merge_duplicate_tpe_points(points);
        for(int i=0;i<points.size();++i) points[i] = tpe_changer(points[i]);
        auto split_intervals = soro::tpe_simulation::split_intervals(
            intervals, points, t.physics_);
        if(!std::is_sorted(split_intervals.p_.begin(),split_intervals.p_.end(),[](interval_point const& pt1, interval_point const& pt2){return pt1.distance_<pt2.distance_;})){
          throw std::logic_error("interval points arent sorted");
        }
        if(t.start_speed_>points.front().v_max_&&points.front().distance_==si::length::zero()) throw std::logic_error("Train starts too fast");
        train_state state;
        state.speed_ = t.start_speed_;
        state.dist_ = si::length::zero();
        state.time_ = si::time(t.start_time_.count());
        auto interval = split_intervals.begin();
        train::trip const trip(train::trip::id{0}, t.id_, ZERO<absolute_time>);

        //points.erase(points.begin());

        for (auto const& pt : points) {
          if (pt.distance_ < state.dist_) continue;
          state = soro::tpe_simulation::get_end_state(state, pt, interval,
                                                      nullptr, t, trip);
          CHECK_EQ(state.dist_, pt.distance_);
          CHECK_GE(state.speed_, pt.v_min_);
          CHECK_LE(state.speed_, pt.v_max_);
          CHECK_EQ(state.dist_,interval.start_distance());
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
    check_get_end_state({tt->trains_[0]},infra,infra::type_set({type::HALT,type::EOTD}),identity,max_speed_reducer);
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
      auto vector = soro::tpe_simulation::fix_intervals(split_intervals.begin(),
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
