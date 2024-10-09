#include "increase_time.h"
namespace increase_time{
  using phase_checker = std::function<bool(phase_type const&,phase_type const&,phase_type const&)>;

  inline auto AHD_checker = [](phase_type t1, phase_type t2, phase_type t3) {
    return t1 == acceleration && (t2 == braking || t3 == braking);
  };
  inline auto AHA_checker = [](phase_type t1, phase_type t2, phase_type) {
    return t1 == acceleration && t2 == cruising;
  };
  inline auto HDH_checker = [](phase_type t1, phase_type t2, phase_type) {
    return t1 == cruising && t2 == braking;
  };
  inline auto DHA_checker = [](phase_type t1, phase_type t2, phase_type t3) {
    return t1 == braking && t2 == cruising && t3 == acceleration;
  };

  inline auto HA_checker = [](phase_type t1,phase_type t2,phase_type = invalid){return
                                                           t1==cruising&&t2==acceleration;};
  inline auto DA_checker = [](phase_type
                           t1,phase_type t2,phase_type = invalid){return t1==braking&&t2==acceleration;};
  inline auto DH_checker =
      [](phase_type t1,phase_type t2,phase_type = invalid){return t1==braking&&t2==cruising;};
}// namespace increase_time
