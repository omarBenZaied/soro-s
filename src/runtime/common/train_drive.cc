#pragma once
#include "soro/runtime/common/increase_time.h"
namespace increase_time{
using namespace soro::runtime;
void train_drive::trim() {
  for(int i=0;i<phases_.size();++i) {
    auto& phase = phases_[i];
    phase.erase(phase.begin()+1,phase.end()-1);
    phase.shrink_to_fit();
  }
  phases_.shrink_to_fit();
}

void fix_times_cruise(vector<train_state>& phase, train_state const& before) {
  phase[0].time_ = before.time_;
  phase[1].time_ = phase[0].time_ + get_cruise_time(phase[0], phase[1]);
}
void fix_times_non_cruise(vector<train_state>& phase,
                          train_state const& before) {
  auto time_offset = before.time_ - phase[0].time_;
  for (int i = 0; i < phase.size(); ++i) {
    phase[i].time_ += time_offset;
  }
}
void train_drive::fix_times(int const& offset) {
  utls::sassert(phases_.size() == phase_types_.size(),
                "There are not as many types as phases");
  for (int i = offset; i < phase_types_.size(); ++i) {
    auto predecessor = i > 0 ? phases_[i - 1].back() : start_state_;
    switch (phase_types_[i]) {
      case cruising: fix_times_cruise(phases_[i], predecessor); break;
      case braking:
      case acceleration: fix_times_non_cruise(phases_[i], predecessor); break;
      default: throw utl::fail("Invalid phase type detected");
    }
  }
}

void train_drive::merge_phases(const int& offset) {
  utls::sassert(phase_types_[offset] == phase_types_[offset + 1],
                "the two phases to be merged arent equal");
  phase_types_.erase(phase_types_.begin() + offset + 1);
  switch (phase_types_[offset]) {
    case acceleration:
    case braking:
      phases_[offset].insert(phases_[offset].end(),
                             phases_[offset + 1].begin() + 1,
                             phases_[offset + 1].end());
      break;
    case cruising:
      phases_[offset].pop_back();
      phases_[offset].push_back(phases_[offset + 1].back());
      break;
    default: throw utl::fail("Invalid phase type detected");
  }
  phases_.erase(phases_.begin() + offset + 1);
}

void train_drive::fix_phases(int const& start_offset) {
  auto i = start_offset;
  while (i < phase_types_.size() - 1) {
    if (phase_types_[i] == phase_types_[i + 1]) {
      merge_phases(i);
    } else
      ++i;
  }
}

void train_drive::fix_drive(int const& offset) {
  fix_times(offset);
  fix_phases(offset);
}

void train_drive::push_back(vector<train_state> const& phase,
                            phase_type const& type) {
  phases_.push_back(phase);
  phase_types_.push_back(type);
}

void train_drive::fix_distance(int const& offset) {
  for(auto i = offset; i < phases_.size(); ++i) {
    auto predecessor = i > 0 ? phases_[i - 1].back() : start_state_;
    auto dist_offset = predecessor.dist_ - phases_[i][0].dist_;
    for (auto& state : phases_[i]) {
      state.dist_ = dist_offset + state.dist_;
    }
  }
}

train_drive& train_drive::operator+=(const train_drive& other) {
  if (other.phases_.empty()) return *this;
  auto offset = phases_.size();
  insert(offset,other.phases_,other.phase_types_);
  fix_distance(offset);
  fix_drive(offset > 0 ? offset - 1 : 0);
  return *this;
}
void train_drive::erase_elements(int const& offset, const int& to_delete) {
  auto first_phase = phases_.begin() + offset;
  auto first_type = phase_types_.begin() + offset;
  phases_.erase(first_phase, first_phase + to_delete);
  phase_types_.erase(first_type, first_type + to_delete);
}

void train_drive::insert(const int& offset, const phases& new_phases,
                         vector<phase_type> const& types) {
  phases_.insert(phases_.begin() + offset, new_phases.begin(),
                 new_phases.end());
  phase_types_.insert(phase_types_.begin() + offset, types.begin(),
                      types.end());
}

void train_drive::print() {
  std::cout << "Train Drive Print" << std::endl;
  utls::for_each(phase_types_, [](phase_type const& type) {
    std::cout << type << std::endl;
  });
  std::cout << std::endl;
  for (auto const& phase : phases_) {
    utls::for_each(phase, [](train_state const& state) {
      std::cout << state.time_ << ' ' << state.dist_ << ' ' << state.speed_
                << std::endl;
    });
  }
}
}// namespace increase_time