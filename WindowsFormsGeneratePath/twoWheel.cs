using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeneratePath
{
    class twoWheel : robotModel
    {
        public double velocity;             // 速度(m/s)
        public double steering;             // ステアリングの角度(rad)
        public const double TREAD = 0.280;  // トレッド(m)
        
        /// <summary>
        /// コンストラクタ
        /// </summary>
        public twoWheel()
        {
            Initialize();
        }

        /// <summary>
        /// 初期化
        /// </summary>
        public override void Initialize()
        {
            velocity = 0;
            steering = 0;
            base.Initialize();
        }

        public 
    }
}