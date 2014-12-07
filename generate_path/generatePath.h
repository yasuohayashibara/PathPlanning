#include "two_wheel.h"
#include <fstream>
#include <string>
#include <sstream>


class generatePath{

public:
	generatePath(){}
	virtual ~generatePath(){}

	//A*による大域的な検索
	void astarPath();

	//マージンのファイルを作成
	void createMarjin();

	//作成したマージンのファイルを利用して経路の作成
	void pathPlan();

	//評価関数
	//ロボットの外周とマージンとの距離 今は重心位置
	bool func1(std::vector<float> xl, std::vector<float> yl, std::vector<float> xr, std::vector<float> yr, Node our, int t);

	//-πからπ
	float maxPi(float theta){
		while(theta>M_PI) theta -= (2.0*M_PI);
		while(theta<M_PI) theta += (2.0*M_PI);
		return theta;
	}

private:

	std::vector<Node> astarData;//A*による大域的な経路 位置と親ノードと自ノードの番号のみ使用

	two_w tcar;
	Node node;
	std::vector<Node> pathData;//最終的な経路
	std::vector<Node> preNode;
	std::vector<Node> preNode2;

	float xs,ys,thetas;

};
