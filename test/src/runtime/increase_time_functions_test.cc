#include "doctest/doctest.h"
#include "test/tpe_runtime/increase_time_utls.h"

#include "test/file_paths.h"

#include "soro/infrastructure/graph/type.h"

namespace increase_time{

TEST_SUITE("increase_time functions suite"){

  TEST_CASE("DA simple test"){
    train_drive drive;
    train_state start_brake(si::time::zero(),si::length::zero(),si::speed(2));
    train_state end_brake(si::time(1),si::length(1),si::speed(1));
    train_state end_accel(si::time(1.5),si::length(2),si::speed(2));
    drive.push_back({start_brake,end_brake},braking);
    drive.push_back({end_brake,end_accel},acceleration);
    tpe_point point(si::length(2),si::time(2),si::time::infinity(),si::speed::zero(),si::speed::infinity());
    set_pt(point);
    infra::infrastructure const infra(soro::test::HILL_OPTS);
    tt::timetable const tt(soro::test::HILL_TT_OPTS, infra);
    bool finished = check_DA(drive,tt->trains_.front().physics_);
    CHECK_FALSE(!finished);
    CHECK_EQ(drive.phases_.size(),2);
    CHECK_EQ(drive.phase_types_,soro::vector<phase_type>{braking,cruising});
    CHECK_EQ(drive.phases_.back().back().time_.val_,2);

  }

  TEST_CASE("increase time DA HA hill"){
    infra::infrastructure const infra(soro::test::HILL_OPTS);
    tt::timetable const tt(soro::test::HILL_TT_OPTS, infra);
    //This does nothing since HA doesnt appear in these configurations
    test_check_function({tt->trains_[0]},infra,infra::type_set({soro::infra::type::HALT,soro::infra::type::EOTD}),4);
  }

  TEST_CASE("increase time DA HA intersection"){
    infra::infrastructure const infra(soro::test::INTER_OPTS);
    tt::timetable const tt(soro::test::INTER_TT_OPTS, infra);
    test_check_function({tt->trains_[0]},infra,infra::type_set({soro::infra::type::HALT,soro::infra::type::EOTD}),4);
  }

  TEST_CASE("increase time DA HA follow"){
    infra::infrastructure const infra(soro::test::SMALL_OPTS);
    tt::timetable const tt(soro::test::FOLLOW_OPTS, infra);
    //This does nothing since HA doesnt appear in this configuration
    test_check_function(tt->trains_,infra,infra::type_set({soro::infra::type::HALT,soro::infra::type::EOTD}),4);
  }

  TEST_CASE("increase time DA HA cross"){
    auto const infra =
        utls::try_deserializing<infra::infrastructure>("small_opts.raw", soro::test::SMALL_OPTS);
    auto const tt =
        utls::try_deserializing<tt::timetable>("cross_opts.raw", soro::test::CROSS_OPTS, infra);
    test_check_function(tt->trains_,infra,infra::type_set({soro::infra::type::HALT,soro::infra::type::EOTD}),4);
  }

}

}// namespace increase_time
