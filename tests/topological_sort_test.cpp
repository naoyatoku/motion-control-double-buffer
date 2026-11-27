//topological_sort、build_dispatch_order、MotorSystem のテスト。
//
//  ビルドと実行（リポジトリの一番上で）:
//    g++ -std=c++11 -Wall -Wextra -I. tests/topological_sort_test.cpp motor_dispatch.cpp -o tst.exe && ./tst.exe
//    g++ -std=c++11 -Wall -Wextra -I. -DNDEBUG tests/topological_sort_test.cpp motor_dispatch.cpp -o tst_r.exe && ./tst_r.exe
//  どちらも最後に "all passed"、終了コード 0 が期待値。
//  （登録されていない依存先のテストは、assert で止まるので NDEBUG のときだけ行う）
#include "topological_sort.hpp"
#include "motor_system.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int g_fail = 0;
static void check(bool ok , const char* what)
{
	if(!ok){	++g_fail;	printf("FAIL: %s\n" , what);	}
}

//---- テスト用のノード。Motor を継承しない（関数が2つあれば使えることの確認）----
struct Node {
	char		name;
	int			dep_n;
	const Node*	deps[8];
	int			dependency_count() const	{	return dep_n;	}
	const Node*	dependency(int d) const		{	return deps[d];	}
};

static Node g_node[26];
static void reset_nodes()
{
	for(int i = 0 ; i < 26 ; ++i){	g_node[i].name = (char)('A' + i);	g_node[i].dep_n = 0;	}
}
static Node* N(char c)	{	return &g_node[c - 'A'];	}
//c の依存先を deps（"BC" なら B と C）にする
static void deps(char c , const char* d)
{
	Node* p = N(c);
	p->dep_n = (int)strlen(d);
	for(int i = 0 ; i < p->dep_n ; ++i)	{	p->deps[i] = N(d[i]);	}
}

//input（"FDCBEA" など）の順に nodes を作って並べ、結果を名前の文字列にする。失敗なら "false"
static const char* sort_names(const char* input)
{
	static char result[32];
	Node* nodes[26];
	Node* out[26];
	const int n = (int)strlen(input);
	for(int i = 0 ; i < n ; ++i)	{	nodes[i] = N(input[i]);	}
	if(!topological_sort<26>(n , nodes , out))	{	return "false";	}
	for(int i = 0 ; i < n ; ++i)	{	result[i] = out[i]->name;	}
	result[n] = 0;
	return result;
}
static void expect(const char* title , const char* input , const char* expected)
{
	const char* got = sort_names(input);
	const bool ok = (strcmp(got , expected) == 0);
	printf("  %-34s input=%-8s -> %-8s %s\n" , title , input , got , ok ? "ok" : "NG");
	check(ok , title);
}

//---- 1. 決まった例（手で追った結果と同じか）----
static void test_fixed()
{
	printf("[fixed cases]\n");
	reset_nodes();	deps('A' , "B");	deps('B' , "C");	deps('C' , "D");
	expect("chain A<-B<-C<-D" , "ABCD" , "DCBA");

	reset_nodes();	deps('B' , "A");	deps('C' , "A");	deps('D' , "BC");	deps('F' , "DE");
	expect("diamond (two deps)" , "FDCBEA" , "EACBDF");

	reset_nodes();	deps('A' , "B");	deps('B' , "A");	deps('C' , "A");
	expect("cycle A<->B, C->A, D free" , "ABCD" , "false");

	reset_nodes();	deps('A' , "B");	deps('B' , "A");	deps('C' , "A");	deps('D' , "A");
	expect("cycle, everyone waits" , "ABCD" , "false");

	reset_nodes();	deps('A' , "A");
	expect("self dependency" , "AB" , "false");

	reset_nodes();	deps('A' , "B");	deps('B' , "C");	deps('C' , "A");
	expect("long cycle A->B->C->A" , "ABC" , "false");

	reset_nodes();	deps('B' , "AA");	deps('C' , "BB");
	expect("same dependency twice" , "CBA" , "ABC");

	reset_nodes();
	expect("no dependencies (input order)" , "DCBA" , "DCBA");
	expect("empty" , "" , "");

#ifdef NDEBUG
	reset_nodes();	deps('B' , "G");
	expect("dependency not in nodes (NDEBUG)" , "AB" , "false");
#endif
}

//---- 2. const T* の配列 / n > MaxN ----
static void test_types()
{
	printf("[types]\n");
	reset_nodes();	deps('B' , "A");
	const Node* nodes[] = { N('B') , N('A') };								//const T* の配列
	const Node* out[2];
	const bool ok = topological_sort<4>(2 , nodes , out);					//T = const Node
	check(ok && out[0] == N('A') && out[1] == N('B') , "const T* array");
	printf("  const T* array                     %s\n" , (ok && out[0] == N('A')) ? "ok" : "NG");

	Node* many[5] = { N('A') , N('B') , N('C') , N('D') , N('E') };
	Node* out5[5];
	check(!topological_sort<4>(5 , many , out5) , "n > MaxN returns false");
	printf("  n > MaxN -> false                  %s\n" , topological_sort<4>(5 , many , out5) ? "NG" : "ok");
}

//---- 3. 乱数の依存関係で、順番の決まりが守られているか ----
//  ランダムな順番 perm を決めて、各ノードは perm で自分より前のノードにだけ依存させる（循環なし）。
//  入力の順番も混ぜて並べ、結果が「全ノードを1回ずつ」「依存先が必ず前」になっているかを確かめる。
//  さらに2本のノードがお互いに依存するようにして循環を作り、false になるかを確かめる。
static void test_random()
{
	printf("[random graphs]\n");
	srand(1);
	int dag_ok = 0 , cycle_ok = 0;
	const int RUNS = 20000;
	for(int it = 0 ; it < RUNS ; ++it){
		reset_nodes();
		const int n = 1 + rand() % 20;
		int perm[26];
		for(int i = 0 ; i < n ; ++i)	{	perm[i] = i;	}
		for(int i = n - 1 ; i > 0 ; --i){	const int j = rand() % (i + 1);	const int t = perm[i];	perm[i] = perm[j];	perm[j] = t;	}
		for(int p = 1 ; p < n ; ++p){
			Node& me = g_node[perm[p]];
			const int nd = rand() % 4;
			for(int d = 0 ; d < nd ; ++d)	{	me.deps[me.dep_n++] = &g_node[perm[rand() % p]];	}	//自分より前のどれか（重複もあり）
		}
		Node* nodes[26];
		Node* out[26];
		for(int i = 0 ; i < n ; ++i)	{	nodes[i] = &g_node[i];	}
		for(int i = n - 1 ; i > 0 ; --i){	const int j = rand() % (i + 1);	Node* t = nodes[i];	nodes[i] = nodes[j];	nodes[j] = t;	}

		//循環なし: 並べられて、決まりを守っているか
		bool ok = topological_sort<26>(n , nodes , out);
		if(ok){
			int pos[26];
			for(int i = 0 ; i < 26 ; ++i)	{	pos[i] = -1;	}
			for(int i = 0 ; i < n ; ++i){
				const int id = out[i]->name - 'A';
				if(pos[id] >= 0)	{	ok = false;	}								//同じノードが2回
				pos[id] = i;
			}
			for(int i = 0 ; i < n ; ++i){
				for(int d = 0 ; d < g_node[i].dep_n ; ++d){
					if(pos[g_node[i].deps[d]->name - 'A'] >= pos[i])	{	ok = false;	}	//依存先が後ろにいる
				}
			}
		}
		if(ok)	{	++dag_ok;	}
		else	{	check(false , "random DAG");	}

		//逆向きの依存を足して循環を作る
		//  後ろのノード late が前のノード early に依存し（無ければ足す）、さらに early が late に依存すると循環になる。
		if(n >= 2){
			const int a = 1 + rand() % (n - 1);											//perm で後ろのノード
			const int b = rand() % a;													//perm で前のノード
			Node& early = g_node[perm[b]];
			Node& late  = g_node[perm[a]];
			if(late.dep_n < 8)	{	late.deps[late.dep_n++] = &early;	}				//late → early
			early.deps[early.dep_n++] = &late;											//early → late  で循環
			if(!topological_sort<26>(n , nodes , out))	{	++cycle_ok;	}
			else	{	check(false , "random cycle not detected");	}
		}else{
			++cycle_ok;
		}
	}
	printf("  DAG sorted correctly  %d / %d\n" , dag_ok , RUNS);
	printf("  cycle detected        %d / %d\n" , cycle_ok , RUNS);
}

//---- 4. build_dispatch_order（Motor 用）----
static void test_dispatch()
{
	printf("[build_dispatch_order]\n");
	MotorAxis a , b , c;
	Motor* da[] = { &b };	a.set_deps(da , 1);										//a は b に依存
	Motor* db[] = { &c };	b.set_deps(db , 1);										//b は c に依存
	Motor* all[] = { &a , &b , &c };
	Motor* order[kMaxMotors];
	bool ok = build_dispatch_order(3 , all , order);
	check(ok && order[0] == &c && order[1] == &b && order[2] == &a , "dispatch chain");
	printf("  chain c->b->a                      %s\n" , (ok && order[0] == &c && order[1] == &b && order[2] == &a) ? "ok" : "NG");

	Motor* dc[] = { &a };	c.set_deps(dc , 1);										//c も a に依存 → 循環
	ok = build_dispatch_order(3 , all , order);
	check(!ok , "dispatch cycle");
	printf("  cycle -> false                     %s\n" , !ok ? "ok" : "NG");
}

//---- 5. MotorSystem（ダブルバッファで順番を切り替える）----
//  calculate() が呼ばれた順を記録する軸で、isr_cycle() の順番を確かめる。
static char g_calls[16];
static int  g_ncalls = 0;
struct LoggingAxis : MotorAxis {
	char name;
	void calculate() noexcept override	{	g_calls[g_ncalls++] = name;	}
};
static const char* run_isr(MotorSystem& sys)
{
	g_ncalls = 0;
	sys.isr_cycle();
	g_calls[g_ncalls] = 0;
	return g_calls;
}
static void expect_isr(MotorSystem& sys , const char* title , const char* expected)
{
	const char* got = run_isr(sys);
	const bool ok = (strcmp(got , expected) == 0);
	printf("  %-34s -> %-6s %s\n" , title , got , ok ? "ok" : "NG");
	check(ok , title);
}
static void test_system()
{
	printf("[MotorSystem]\n");
	static MotorSystem sys;
	static LoggingAxis x , y , z;
	x.name = 'X';	y.name = 'Y';	z.name = 'Z';
	sys.add_motor(&z);	sys.add_motor(&y);	sys.add_motor(&x);
	check(sys.init() , "init");
	expect_isr(sys , "no deps (registration order)" , "ZYX");

	check(sys.set_dependency(&z , &y) , "Z follows Y");
	check(sys.set_dependency(&y , &x) , "Y follows X");
	expect_isr(sys , "X -> Y -> Z" , "XYZ");

	check(!sys.set_dependency(&x , &z) , "cycle rejected");
	expect_isr(sys , "after rejected cycle (unchanged)" , "XYZ");

	check(sys.set_dependency(&z , nullptr) , "Z independent");
	expect_isr(sys , "Z independent again" , "ZXY");
}

int main()
{
	test_fixed();
	test_types();
	test_random();
	test_dispatch();
	test_system();
	if(g_fail == 0)	{	printf("all passed\n");	return 0;	}
	printf("%d failed\n" , g_fail);
	return 1;
}
