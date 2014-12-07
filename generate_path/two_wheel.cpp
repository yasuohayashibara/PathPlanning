#include "two_wheel.h"

/*
 * @brief 動きを計算する
 * @param[in] vr 右車輪の速度の変化(m/s/step)
 * @param[in] vl 左車輪の速度の変化(m/s/step)
 * @param[in] t サンプリングタイム(s)
 * @param[in] pre_node 一つ前のノード
 */
void two_w::equation_of_motion(double vr, double vl, double dt, Node pre_node){

 	mVr = pre_node.Vr() + vr;
 	mVl = pre_node.Vl() + vl;

	if( mVr>Vmax ) mVr = Vmax;
	if( mVr<Vmin ) mVr = Vmin;
	if( mVl>Vmax ) mVl = Vmax;
	if( mVl<Vmin ) mVl = Vmin;

	dt = 1.0;

	if((mVr-mVl)==0){
		
		mR = 0;
		mW = 0;
		mV = (mVr + mVl)/2;
	}
	else{
	
		mR = (mtread/2)*(mVr + mVl)/(mVr - mVl);	// 回転半径(m)
		mW = (mVr - mVl)/mtread;			// 回転角度(rad/s)
		mV = (mVr + mVl)/2;			        // 速度(m/s)
	}
	       
	mtheta = maxPi(mW*dt + pre_node.Theta());	// 新しい角度

	Set_xy( mV*cos(mtheta)*dt + pre_node.Our_Pos().x(), mV*sin(mtheta)*dt + pre_node.Our_Pos().y() );

	mnode.Set_Vel(mVr, mVl);
	mnode.Set_theta(mtheta);
	mnode.Set_Pos(x(), y());
	
    Set_Node(mnode);
}
