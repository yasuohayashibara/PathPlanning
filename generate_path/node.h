/*!
 * @file nodeを記述するためのクラス
 */
#include "position.h"

class Node{

public:
	/*
	 * @brief コンストラクタ
	 */
	Node(){
		mV		 = 0.0;
		mVr		 = 0.0;
		mVl		 = 0.0;
		mtheta	 = 0.0;
		mIz		 = 0.0;
		mb		 = 0.0;
		msteer	 = 0.0;
		mour_num = 0;
		mpre_num = 0;
	}

	/*
	 * @brief デストラクタ
	 */
	virtual~Node(){}

	/*
	 * @brief 位置の設定
	 * @param[in] x x座標(m)
	 * @param[in] y y座標(m)
	 */
	void Set_Pos(double x, double y){
		mpos.SetPosition(x,y);
	}

	/*
	 * @brief 位置の取得
	 * @return 位置
	 */
	Position Our_Pos() const { return mpos; }

	/*
	 * @brief 角度の設定
	 * @param[in] thera 角度(rad)
	 */
	void Set_theta(double theta) { mtheta = theta; }
	double Theta() const { return mtheta; }

	/*
	 * @brief ステアリングの角度の設定
	 * @param[in] steer ステアリングの角度(rad)
	 */
	void Set_Steer(double steer) { msteer = steer; }
	double Steer() { return msteer; }

	/*
	 * @brief 前のnode番号の設定
	 * @param[in] pre 前のノード番号
	 */
	void Set_PreNum(unsigned int pre) { mpre_num = pre; }
	unsigned int Pre_Num()  { return mpre_num; }

	/*
	 * @brief このノードの番号を設定
	 * @param[in] num ノード番号
	 */
	void Set_OurNum(unsigned int num) { mour_num = num; } 

	/*
	 * @brief このノードの番号を戻す
	 * @return ノード番号
	 */
	unsigned int Our_Num()  { return mour_num; }

	/*
	 * @brief 速度の設定
	 * @param[in] r 右車輪の速度(m/s)
	 * @param[in] l 左車輪の速度(m/s)
	 */
	void Set_Vel(double r, double l) {
		mVr = r;
		mVl = l;
	}

	/*
	 * @brief 右車輪の速度の取得
	 * @return 右車輪の速度(m/s)
	 */
	double Vr() { return mVr; }

	/*
	 * @brief 左車輪の速度の取得
	 * @return 左車輪の速度(m/s)
	 */
	double Vl() { return mVl; }

private:
	double mV;
	double mVr;				//! 右車輪の速度(m/s)
	double mVl;				//! 左車輪の速度(m/s)
	double mtheta;			//! 角度(rad)
	double mIz;				//! 慣性モーメント ４輪で運動方程式たてた時に使う
	double mb;
	double msteer;			//! ステアリングの角度(rad)
	Position mpos;			//! ロボットの位置(m)
	unsigned int mour_num;	//! ノード番号
	unsigned int mpre_num;	//! 一つ前のノード番号
};
