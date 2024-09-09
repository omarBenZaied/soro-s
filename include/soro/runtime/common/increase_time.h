//
// Created by omarb on 04.09.2024.
//
#include "soro/base/soro_types.h"
#include "soro/runtime/common/train_state.h"
#pragma once
enum phase_type{
  acceleration, braking, cruising,invalid
};

struct train_drive{
  soro::vector<soro::vector<soro::runtime::train_state>> phases_;
  soro::vector<phase_type> phase_types_;
};

/*void increase_time(train_drive& drive);

bool check_AHD(train_drive& drive);

bool check_AHA(train_drive& drive);*/

bool check_HDH(train_drive& drive);

bool check_DHA(train_drive& drive);

bool check_HA(train_drive& drive);

bool check_DH(train_drive& drive);

bool check_DA(train_drive& drive);

//bool check_A(train_drive& drive);

//bool check_H(train_drive& drive);

bool check_D(train_drive& drive);