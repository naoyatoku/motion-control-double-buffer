#include "motor_dispatch.hpp"
#include "topological_sort.hpp"

bool build_dispatch_order(std::size_t   n,
                          Motor* const  all_motors[],
                          Motor*        order[]) noexcept {
	if(n > kMaxMotors)	{	return false;	}
	return topological_sort<(int)kMaxMotors>((int)n , all_motors , order);		//循環なら false（order は途中まで書き換わる）
}
