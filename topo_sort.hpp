#pragma once

//==================================================================================================
//  トポロジカルソート（Kahn 法）
//
//  nodes[0..n-1] を、依存先が先に来る順番に並べて out[0..n-1] に入れる。（B が A に依存するなら A → B の順）
//
//  T（ノードのクラス）には、次の2つの関数があればよい。（基底クラスの継承や virtual は要らない）
//    dependency_count()   : 依存の本数
//    dependency(d)        : d 本目の依存先のポインタ
//  nodes に入っていないノードへの依存は無視する。
//
//  戻り値
//    成功        : true
//    循環がある  : false。（out は途中まで書き換わっているので使わないこと）
//    n > MaxN    : false。
//
//  やり方
//    1. 各ノードが「あと何本待っているか」(in_degree) を数える。
//    2. 待ちが 0 のノードをキューに入れる。
//    3. キューから1つ出して並べ、それを待っていたノードの in_degree を減らす。0 になったらキューに入れる。
//    4. 全部並べられなければ、どこかに循環がある。（循環の中のノードは in_degree が 0 にならない）
//  out の前から順に出すので、out をそのままキューに使う。
//  どのノードもキューに入るのは1回だけなので、キューは n 個で足りる。
//  同じ依存先を2本登録しても、数える時と減らす時で同じ本数を数えるので、正しく並ぶ。
//  計算量はおよそ n^2 × 依存の本数。動的なメモリ確保はしない。
//==================================================================================================

template <int MaxN , class T>
bool topo_sort(T* const* nodes , int n , T** out) noexcept
{
	if( (n < 0) || (n > MaxN) )	{	return false;	}

	//1. あと何本待っているか。（nodes に入っている依存先だけ数える）
	int in_degree[MaxN] = {};
	for(int i = 0 ; i < n ; ++i){
		const int nd = (int)nodes[i]->dependency_count();
		for(int d = 0 ; d < nd ; ++d){
			for(int k = 0 ; k < n ; ++k){
				if(nodes[k] == nodes[i]->dependency(d))	{	++in_degree[i];	break;	}
			}
		}
	}

	//2. 待ちのないノードをキューに入れる
	int head = 0;											//次に出す位置
	int tail = 0;											//次に入れる位置
	for(int i = 0 ; i < n ; ++i){
		if(in_degree[i] == 0)	{	out[tail++] = nodes[i];	}
	}

	//3. 1つずつ出して、それを待っていたノードの待ちを減らす
	while(head < tail){
		const T* m = out[head++];
		for(int i = 0 ; i < n ; ++i){
			const int nd = (int)nodes[i]->dependency_count();
			for(int d = 0 ; d < nd ; ++d){
				if( (nodes[i]->dependency(d) == m) && (--in_degree[i] == 0) )	{	out[tail++] = nodes[i];	}	//m を待っていた本数だけ減らす
			}
		}
	}

	//4. 全部並べられたか
	return tail == n;
}
