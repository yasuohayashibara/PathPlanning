/*
 * *file 車両モデルのヘッダファイル
 */

#ifndef CAR_MODEL_H_
#define CAR_MODEL_H_

#include "node.h"
#include <vector>

class car{

public:

	/*!
	 * @brief コンストラクタ
	 */
	car(){
		mX=0.0;
		mY=0.0;
		mV=0.0;
		mVr=0.0;
		mVl=0.0;
		mW=0.0;
		mR=0.0;
		mtheta=0.0;
		dt=0.0;
		mtread=0.0;
	}
	
	/*!
	 * @brief デストラクタ
	 */
	virtual~car(){
	}

	/*
	 * @brief ノードの設定
	 * @param[in] node 設定するノード
	 */
	void Set_Node(Node node) { mnode = node; }
	Node Our_Node(){ return mnode; }

	/*
	 * @brief 車両の移動を計算
	 */
	virtual void equation_of_motion(){}

	std::vector<Node> xypos;

	Position Our_Pos() { return mnode.Our_Pos(); }
 	
	void Set_PreNum();
	unsigned int Pre_Num();

	void Set_OurNum();
	unsigned int Our_Num();

	void Set_xy(double x, double y){

		mX = x;
		mY = y;
	}

	double x() { return mX; }
	double y() { return mY; }

	/*
	 * @brief 左右の車輪の速度の設定
	 * @param[in] vr 右の車輪の速度(m/s)
	 * `param[in] vl 左の車輪の速度(m/s)
	 */
	void Set_Vrl(double vr, double vl){
		mVr=vr;
		mVl=vl;
	}

	/*
	 * @brief 角度を-PI～PIの間に変換する関数
	 * @param[in] theta 角度(rad)
	 ` @return theta 変換した角度(rad)
	 */
	double maxPi(double theta){
		while (theta >  M_PI) theta -= (2.0 * M_PI);
		while (theta < -M_PI) theta += (2.0 * M_PI);
		return theta;
	}
	
private:
	double mX,mY;		//! 位置 (m)
	double mV,mVr,mVl;	//! 速度 (m/s)
	double mW;			//! 角速度 (rad/s)
	double mR;			//! 回転半径 (m)
	double mtheta;		//! 回転角度 (rad)

	double dt;			//! サンプリングタイム(s)
	Node mnode;			//! ノード

	double mtread;		//! トレッド画面サイズに合わせるのを忘れない
};

#endif
