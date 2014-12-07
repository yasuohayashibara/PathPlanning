/*
 * @file ２輪の動きを計算するクラスのヘッダファイル
 */

#ifndef TWO_WHEEL_H_
#define TWO_WHEEL_H_

#include "car_model.h"

class two_w : public car {

public:
	/*
	 * @brief コンストラクタ
	 */
	two_w(){
		mV=0.0;
		mtread=280.0;	//!トレッド(mm)
		mlr=180.0*0.0001;// 
		mlf=180.0*0.0001;// 
		dt=0.0;
		Vmax=500.0;			//! 最大速度 mm/s
		Vmin=-500.0;
	}
	virtual~two_w(){}

	/*
	 * @brief 車両の移動を計算
	 */
	virtual void equation_of_motion(double vr, double vl, double t, Node pre_node);

	/*
	 * @brief ノードの設定
	 * @param[in] node 設定するノード
	 */
	void Set_Node(Node node) { mnode = node; }
	Node Our_Node() { return mnode; }


	void Set_PreNum(unsigned int pn) { mnode.Set_PreNum(pn); }
	unsigned int Pre_Num() { return mnode.Pre_Num(); }

	void Set_OurNum(unsigned int n) { mnode.Set_OurNum(n); }
	unsigned int Our_Num() { return mnode.Our_Num(); }

	std::vector<Node> xypos;
	
private:
	double mV,mVr,mVl;
	double Vmax,Vmin;
	double mW,mR,mtheta;

	double dt;
	Node mnode;
	double mtread;
	double mlf,mlr;//前後の車輪と重心との距離 mlf+mlr=ホイールベース
};

#endif
