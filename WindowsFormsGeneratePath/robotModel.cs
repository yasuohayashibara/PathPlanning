using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeneratePath
{
    /// <summary>
    /// ロボットの移動モデル
    /// </summary>
    class robotModel
    {
        public double x;        // x座標の位置(m)
        public double y;        // y座標の位置(m)
        public double the;      // 角度(rad)
        public double xd;       // 前後方向の速度(m/s)　ロボット座標系
        public double yd;       // 左右方向の速度(m/s)　ロボット座標系
        public double thed;     // 角速度(rad/s)　ロボット座標系

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public robotModel()
        {
            Initialize();
        }

        /// <summary>
        /// 初期化
        /// </summary>
        public virtual void Initialize()
        {
            setPosition(0, 0, 0);
            setVelocity(0, 0, 0);
        }

        /// <summary>
        /// 位置の設定
        /// </summary>
        /// <param name="x">x座標の位置(m)</param>
        /// <param name="y">y座標の位置(m)</param>
        /// <param name="the">角度(rad)</param>
        public void setPosition(double x, double y, double the)
        {
            this.x = x;
            this.y = y;
            this.the = the;
        }

        /// <summary>
        /// 速度の設定
        /// </summary>
        /// <param name="xd">前後方向の速度(m/s)　ロボット座標系</param>
        /// <param name="yd">左右方向の速度(m/s)　ロボット座標系</param>
        /// <param name="thed">角速度(rad/s)　ロボット座標系</param>
        public void setVelocity(double xd, double yd, double thed)
        {
            this.xd = xd;
            this.yd = yd;
            this.thed = thed;
        }

        /// <summary>
        /// 次の位置を計算
        /// </summary>
        /// <param name="dt">サンプリングタイム(s)</param>
        public void calculatePosition(double dt)
        {
            double xdt = xd * Math.Cos(the) - yd * Math.Sin(the);
            double ydt = xd * Math.Sin(the) + yd * Math.Cos(the);
            x += xdt * dt;
            y += ydt * dt;
            the += thed * dt;
        }
    }

}
