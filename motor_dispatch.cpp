#include "motor_dispatch.hpp"
#include "topo_sort.hpp"

namespace {

//モーターの配列を、topo_sort が使う番号のグラフとして見せる。
struct MotorGraph {
	Motor* const*	motors;
	int				n;

	int dep_count(int i) const noexcept {	return (int)motors[i]->dependency_count();	}

	//i 番目のモーターの d 本目の依存先の番号。登録されていないモーターなら -1（無視する）
	int dep(int i , int d) const noexcept {
		const Motor* p = motors[i]->dependency((std::size_t)d);
		for(int k = 0 ; k < n ; ++k){
			if(motors[k] == p)	{	return k;	}
		}
		return -1;
	}
};

}	//namespace

bool build_dispatch_order(MotorDispatchList* list,
                          Motor* const*      all_motors,
                          std::size_t        n) noexcept {
	if(n > kMaxMotors)	{	return false;	}

	const MotorGraph g = { all_motors , (int)n };
	int idx[kMaxMotors];												//計算する順の番号。（失敗したときに list を壊さないよう、いったんここに受ける）
	if(!topo_sort<(int)kMaxMotors>(g , (int)n , idx))	{	return false;	}	//循環がある: list はそのまま

	for(std::size_t i = 0 ; i < n ; ++i)	{	list->motors[i] = all_motors[idx[i]];	}
	list->count = n;
	return true;
}
