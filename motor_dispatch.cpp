#include "motor_dispatch.hpp"
#include "topo_sort.hpp"

bool build_dispatch_order(MotorDispatchList* list,
                          Motor* const*      all_motors,
                          std::size_t        n) noexcept {
	if(n > kMaxMotors)	{	return false;	}

	Motor* order[kMaxMotors];											//計算する順。（失敗したときに list を壊さないよう、いったんここに受ける）
	if(!topo_sort<(int)kMaxMotors>(all_motors , (int)n , order))	{	return false;	}	//循環がある: list はそのまま

	for(std::size_t i = 0 ; i < n ; ++i)	{	list->motors[i] = order[i];	}
	list->count = n;
	return true;
}
