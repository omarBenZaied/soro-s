//
// Created by omarb on 09.09.2024.
//
#pragma once
#include "soro/runtime/common/train_path_envelope.h"
namespace soro::tpe_simulation {
using namespace soro::train_path_envelope;
void merge_duplicate_tpe_points(soro::train_path_envelope::tpe_points& points);
runtime::intervals split_intervals(runtime::intervals const& intervals, train_path_envelope::tpe_points const& pts,rs::train_physics const& tp);
runtime::train_state get_end_state(runtime::train_state current_state,soro::train_path_envelope::tpe_point const& pt, runtime::interval& interval,
                          runtime::train_safety* train_safety,soro::tt::train const& train,soro::tt::train::trip const& trip);
soro::vector<runtime::interval_point> fix_intervals(runtime::interval& interval,soro::train_path_envelope::tpe_point const& point,rs::train_physics const& tp);
soro::vector<runtime::train_state> tpe_respecting_simulation(
    tpe_points& tpe_points, runtime::train_state const& initial,runtime::train_safety* train_safety,
    soro::tt::train const& train, soro::tt::train::trip const& trip,
    soro::infra::infrastructure const& infra,infra::type_set const& record_types);
} // namespace soro::tpe_simulation