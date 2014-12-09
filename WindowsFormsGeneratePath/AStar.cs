using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GeneratePath;

namespace GeneratePath
{
    class AStar
    {
        byte[] map;
        int width;
        int height;
        double unit;        // pixel一辺あたりの長さ(m)
        double start_x;
        double start_y;
        double goal_x;
        double goal_y;
        List<Pos> path;

        public AStar()
        {
            Initialize();
        }

        public void Initialize()
        {
            map = new byte[1];
            width = 1;
            height = 1;
            start_x = 0;
            start_y = 0;
            goal_x = 0;
            goal_y = 0;
            path = new List<Pos>();
        }

        /// <summary>
        /// 地図のセット
        /// </summary>
        /// <param name="map">地図 (0:障害物なし，1:障害物あり)</param>
        /// <param name="width">幅</param>
        /// <param name="height">高さ</param>
        public void setMap(byte[] map, int width, int height)
        {
            this.width = width;
            this.height = height;
            this.map = new byte[width * height];
            for (int i = 0; i < width * height; i++)
            {
                this.map[i] = map[i];
            }
        }

        /// <summary>
        /// スタート位置の設定
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        /// <returns></returns>
        public bool setStartPosition(double x, double y)
        {
            if ((x > width) || (y > height))
            {
                return false;
            }
            start_x = x;
            start_y = y;
            return true;
        }

        /// <summary>
        /// ゴール位置の設定
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        /// <returns></returns>
        public bool setGoalPosition(int x, int y)
        {
            if ((x > width) || (y > height))
            {
                return false;
            }
            goal_x = x;
            goal_y = y;
            return true;
        }

        /// <summary>
        /// 距離の計算
        /// </summary>
        /// <param name="x">x座標の差</param>
        /// <param name="y">y座標の差</param>
        /// <returns>距離</returns>
        private double len(double x, double y)
        {
            return (Math.Sqrt(x * x + y * y));
        }
    }
}
