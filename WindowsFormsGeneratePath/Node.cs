using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GeneratePath
{
    class Node
    {
        public int our_num;         // このノードの番号
        public int pre_num;         // 一つ前のノードの番号
        public double cost;         // スタートノードからの最小コストの推定値
        public double goal_cost;    // ゴールノードまでの最小コストの推定値
        public twoWheel robot;             // メガローバ

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public Node(double x, double y, double the, double velocity_left, double velocity_right, int our_num, int pre_num)
        {
            robot = new twoWheel();
            robot.setPosition(x, y, the);
            robot.velocity_left = velocity_left;
            robot.velocity_right = velocity_right;
            this.our_num = our_num;
            this.pre_num = pre_num;
        }

        /// <summary>
        /// ゴールまでの最小コストの推定値を計算する
        /// </summary>
        /// <param name="goal_x">ゴールのx座標</param>
        /// <param name="goal_y">ゴールのy座標</param>
        /// <returns>ゴールまでの最小コストの推定値</returns>
        public double calculateGoalCost(double goal_x, double goal_y)
        {
//            goal_cost = Math.Abs(goal_x - robot.x) + Math.Abs(goal_y - robot.y);
            double xd = goal_x - robot.x;
            double yd = goal_y - robot.y;
            double goal_the = Math.Atan2(yd, xd);
            double thed = goal_the - robot.the;
//            thed *= 5;
            goal_cost = Math.Sqrt(xd * xd + yd * yd + thed * thed);
            return goal_cost;
        }

        /// <summary>
        /// ヒューリスティックの取得
        /// </summary>
        /// <returns>ヒューリスティック</returns>
        public double getHeuristic()
        {
            return cost + goal_cost;
        }

        /// <summary>
        /// 距離の計算
        /// </summary>
        /// <param name="x">比較する位置のx座標(m)</param>
        /// <param name="y">比較する位置のx座標(m)</param>
        /// <returns>距離(m)</returns>
        public double getLength(double x, double y)
        {
            double dx = x - robot.x;
            double dy = y - robot.y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
