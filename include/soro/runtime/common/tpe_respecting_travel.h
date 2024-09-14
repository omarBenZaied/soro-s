//
// Created by omarb on 09.09.2024.
//
#pragma once
#include "soro/runtime/common/train_path_envelope.h"
namespace soro::tpe_simulation {
void merge_duplicate_tpe_points(soro::train_path_envelope::tpe_points& points);
runtime::intervals split_intervals(runtime::intervals const& intervals, train_path_envelope::tpe_points const& pts,rs::train_physics const& tp);
} // namespace soro::tpe_simulation