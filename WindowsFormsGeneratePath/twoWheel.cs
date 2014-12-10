using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeneratePath
{
    class twoWheel : robotModel
    {
        public double velocity_left;                // 左車輪の速度(m/s)
        public double velocity_right;               // 右車輪の速度(m/s)
        public double acceleration;                 // 左車輪の加速度(m/s^2)
        public const double TREAD = 0.280;          // トレッド(m)
        public const double MAX_VELOCITY = 1.0;     // 最大速度(m/s)
        public const double MAX_ACCELERATION = 0.5; // 最大加速度(m/s^2)
        
        /// <summary>
        /// コンストラクタ
        /// </summary>
        public twoWheel()
        {
            Initialize();
        }

        /// <summary>
        /// コピーコンストラクタ
        /// </summary>
        /// <param name="source">コピー元のコンストラクタ</param>
        public twoWheel(twoWheel source) : base(source)
        {
            this.velocity_right = source.velocity_right;
            this.velocity_left = source.velocity_left;
            this.acceleration = source.acceleration;
        }

        /// <summary>
        /// 初期化
        /// </summary>
        public override void Initialize()
        {
            velocity_left = 0;
            velocity_right = 0;
            acceleration = MAX_ACCELERATION;
            base.Initialize();
        }

        /// <summary>
        /// 加速度の設定
        /// </summary>
        /// <param name="acceleration">加速度(m/s^2)</param>
        public void setAcceleration(double acceleration)
        {
            this.acceleration = Math.Min(acceleration, MAX_ACCELERATION);
        }

        /// <summary>
        /// 絶対値で制限する
        /// </summary>
        /// <param name="value">値</param>
        /// <param name="max">最大値</param>
        /// <returns>絶対値を制限した値</returns>
        private double limitAbs(double value, double max)
        {
            if (value > 0)
            {
                value = Math.Min(value, max);
            }
            else
            {
                value = Math.Max(value, -max);
            }
            return value;
        }
        
        /// <summary>
        /// 位置のアップデート
        /// </summary>
        /// <param name="velocity_left">左ホイールの速度(m/s)</param>
        /// <param name="velocity_right">右ホイールの速度(m/s)</param>
        /// <param name="dt">サンプリングタイム(m/s)</param>
        public void calculatePosition(double velocity_left, double velocity_right, double dt)
        {
            velocity_left = limitAbs(velocity_left, MAX_VELOCITY);
            velocity_right = limitAbs(velocity_right, MAX_VELOCITY);

            double leftVelocityDiff = velocity_left - this.velocity_left;       // 左車輪の速度差
            double rightVelocityDiff = velocity_right - this.velocity_right;    // 右車輪の速度差
            double velocityDiffMax = acceleration * dt;                         // 速度差の最大値

            leftVelocityDiff = limitAbs(leftVelocityDiff, velocityDiffMax);     // 最大加速度を超えないように値を調整
            rightVelocityDiff = limitAbs(rightVelocityDiff, velocityDiffMax);   // 最大加速度を超えないように値を調整
            this.velocity_left += leftVelocityDiff;
            this.velocity_right += rightVelocityDiff;
            xd = (this.velocity_right + this.velocity_left) / 2;                // 前後方向の速度(m/s)　ロボット座標系
            yd = 0;                                                             // 左右方向の速度(m/s)　ロボット座標系
            thed = (this.velocity_right - this.velocity_left) / TREAD;          // 角速度(rad/s) ロボット座標系
            
            calculatePosition(dt);                                              // dt秒後の位置を計算する
        }
    }
}