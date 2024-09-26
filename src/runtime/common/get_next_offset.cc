#pragma once
#include "soro/runtime/common/get_next_offset.h"
namespace increase_time{

vector<int> search_indizes(train_drive const& drive,vector<phase_type> const& types){
  utl::verify(!types.empty()&&types.size()<=drive.phase_types_.size(),"types is either too long or empty");
  vector<int>result;
  vector<phase_type>last_seen(drive.phase_types_.end()-types.size(),drive.phase_types_.end());
  for(int i = drive.phase_types_.size()-types.size();i>0;--i){
    if(last_seen==types) result.push_back(i);
    last_seen.pop_back();
    last_seen.insert(last_seen.begin(),drive.phase_types_[i-1]);
  }
  if(last_seen==types) result.push_back(0);
  return result;
}
int standard_next_offset(train_drive const& drive){
  auto indizesAHD = search_indizes(drive,{acceleration,cruising,braking});
  if(!indizesAHD.empty()) return indizesAHD.back();
  auto indizesAD = search_indizes(drive,{acceleration,braking});
  if(!indizesAD.empty()) return indizesAD.back();
  auto indizesAH = search_indizes(drive,{acceleration,cruising});
  if(!indizesAH.empty()) return indizesAH.back();
  auto indizesHD = search_indizes(drive,{cruising,braking});
  if(!indizesHD.empty()) return indizesHD.back();
  return drive.phase_types_.size()-3;
}


} // namespace increase_time
