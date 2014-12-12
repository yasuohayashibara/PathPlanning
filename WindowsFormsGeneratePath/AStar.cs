using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using GeneratePath;
using System.Diagnostics;

namespace GeneratePath
{        
    class AStar
    {
        byte[] map;         // 地図データ
        int width;          // 地図データの幅
        int height;         // 地図データの高さ
        double unit;        // pixel一辺あたりの長さ(m)
        double max_x;       // x方向の値の最大値(m)
        double max_y;       // y方向の値の最大値(m)
        double start_x;     // x方向のスタート位置(m)
        double start_y;     // y方向のスタート位置(m)
        double start_the;   // スタート時の角度(rad)
        double goal_x;      // x方向のゴール位置(m)
        double goal_y;      // y方向のゴール位置(m)
        double goal_the;    // ゴール時の角度(rad)
        List<PosVel> path;    // 生成された経路

        double goal_radius;         // ゴールに到着したと判定する半径(m)
        double left_velocity_d;     // 左ホイールの速度変更値(m/s/step)
        double right_velocity_d;    // 右ホイールの速度変更値(m/s/step)
        double sampling_time;       // サンプリングタイム(s/step)
        
        /// <summary>
        /// コンストラクタ
        /// </summary>
        public AStar()
        {
            Initialize();
        }

        /// <summary>
        /// 初期化
        /// </summary>
        public void Initialize()
        {
            map = new byte[1];
            width = 1;
            height = 1;
            start_x = 0;
            start_y = 0;
            goal_x = 0;
            goal_y = 0;
            path = new List<PosVel>();

            goal_radius = 0.1;
            sampling_time = 1.0;
            left_velocity_d = 0.05 * sampling_time;
            right_velocity_d = 0.05 * sampling_time;
        }

        /// <summary>
        /// 地図のセット
        /// </summary>
        /// <param name="map">地図 (0:障害物なし，1:障害物あり)</param>
        /// <param name="width">幅</param>
        /// <param name="height">高さ</param>
        public void setMap(byte[] map, int width, int height, double unit)
        {
            this.width = width;
            this.height = height;
            this.map = new byte[width * height];
            this.unit = unit;
            this.max_x = width * unit;
            this.max_y = height * unit;
            for (int i = 0; i < width * height; i++)
            {
                this.map[i] = map[i];
            }
        }

        /// <summary>
        /// 最大角度を-PI～PIに制限
        /// </summary>
        /// <param name="value">角度(rad)</param>
        /// <returns>-PI～PIに制限した角度(rad)</returns>
        private double maxPI(double value)
        {
            while (value > Math.PI) value -= Math.PI * 2;
            while (value < -Math.PI) value += Math.PI * 2;
            return value;
        }

        /// <summary>
        /// スタート位置の設定
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        /// <returns></returns>
        public bool setStartPosition(double x, double y, double the)
        {
            if ((x > max_x) || (y > max_y))
            {
                return false;
            }
            start_x = x;
            start_y = y;
            start_the = maxPI(the);
            return true;
        }

        /// <summary>
        /// ゴール位置の設定
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        /// <returns></returns>
        public bool setGoalPosition(double x, double y, double the)
        {
            if ((x > max_x) || (y > max_y))
            {
                return false;
            }
            goal_x = x;
            goal_y = y;
            goal_the = maxPI(the);
            return true;
        }

        public void setParameter(double goal_radius, double left_velocity_d, double right_velocity_d, double sampling_time)
        {
            this.goal_radius = goal_radius;
            this.left_velocity_d = left_velocity_d;
            this.right_velocity_d = right_velocity_d;
            this.sampling_time = sampling_time;
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

        /// <summary>
        /// (x,y)の場所にノードが存在するかをチェックする関数
        /// </summary>
        /// <param name="list">ノードのリスト</param>
        /// <param name="x">x座標</param>
        /// <param name="y">y座標</param>
        /// <param name="n">ノードの番号</param>
        /// <returns>ノードが存在するかどうか(0:存在しない，1:存在する)</returns>
        private bool checkExist(List<Node> list, double x, double y, double the, double velocity_left, double velocity_right, out int n)
        {
            n = -1;
            const double thre_xy = 0.05;        // 同じカテゴリかどうかのしきい値 (m)
            const double thre_the = 3;          // (rad)
            const double thre_velocity = 0.025;       // (m/s)

            for (int i = 0; i < list.Count; i++)
            {
                twoWheel robot = list[i].robot;
                if ((Math.Abs(robot.x - x) < thre_xy) && (Math.Abs(robot.y - y) < thre_xy) && (Math.Abs(robot.the - the) < thre_the) &&
                    (Math.Abs(robot.velocity_left - velocity_left) < thre_velocity) && (Math.Abs(robot.velocity_right - velocity_right) < thre_velocity))
                {
                    n = i;
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 最小のヒューリスティックの番号を戻す
        /// </summary>
        /// <param name="node">ノードのリスト</param>
        /// <returns>最小のヒューリスティックの番号</returns>
        private int minHeuristic(List<Node> node)
        {
            if (node.Count == 0) return -1;                     // ノードが0個の場合は-1を戻す．
            int min_no = 0;
            double min_heuristic = node[0].getHeuristic();      // ヒューリスティックの取得
            for (int i = 1; i < node.Count; i++)
            {
                double heuristic = node[i].getHeuristic();
                if (heuristic < min_heuristic)
                {
                    min_heuristic = heuristic;
                    min_no = i;
                }
            }
            return min_no;
        }


        private int getMapData(double x, double y)
        {
            int xm = (int)(x / unit);
            int ym = (int)(y / unit);
            return map[ym * width + xm];
        }

        /// <summary>
        /// our_numがnumberであるノードのインデックスを戻す
        /// </summary>
        /// <param name="node">探索するノード</param>
        /// <param name="number">our_numの値</param>
        /// <returns>インデックス</returns>
        private int getIndex(List<Node> node, int number)
        {
            for (int i = 0; i < node.Count; i++)
            {
                if (node[i].our_num == number) return i;
            }
            return -1;
        }

        /// <summary>
        /// A Start Search による経路の計算
        /// </summary>
        /// <returns>true:経路探索成功，false:失敗</returns>
        public bool calculatePath()
        {
            const double GOAL_AREA = 0.1;
            const double MAX_VELOCITY = 1.0;
            List<Node> openList = new List<Node>();             // 計算中のノードを格納するためのリスト
            List<Node> closeList = new List<Node>();            // 計算済みのノードを格納しておくリスト
            // スタートノードとゴールノードを設定
            Node start = new Node(start_x, start_y, start_the, 0, 0, 0, -1);
            Node goal = new Node(goal_x, goal_y, goal_the, 0, 0, 0, 0);
            start.calculateGoalCost(goal_x, goal_y);            // ゴールまでのコストの推定値を計算
            start.cost = 0;                                     // スタートからのコストは0
            openList.Add(start);                                // スタートノードをOpenリストに追加
            Node current = new Node(0, 0, 0, 0, 0, 0, 0);       // 現在，取り出して計算しているノード
            int node_no = 1;
            while (true)
            {
                if (openList.Count == 0) return false;          // ゴールに辿り着くルートが無い
                int min_no = minHeuristic(openList);            // 最小のヒューリスティックのノード番号を取得
                current = openList[min_no];                     // そのノードを取り出す
                openList.RemoveAt(min_no);                      // Openリストから削除して，
                closeList.Add(current);                         // Closeリストに追加する

                // ゴールに到着した場合ループを抜ける
                if (current.getLength(goal_x, goal_y) < GOAL_AREA) break;

                for (double right = -right_velocity_d; right <= right_velocity_d; right += right_velocity_d)
                {
                    for (double left = -left_velocity_d; left <= left_velocity_d; left += left_velocity_d)
                    {
                        double velocity_left = current.robot.velocity_left + left;
                        if (velocity_left > MAX_VELOCITY) continue;
                        double velocity_right = current.robot.velocity_right + right;
                        if (velocity_right > MAX_VELOCITY) continue;
                        if ((velocity_left + velocity_right) <= 0) continue;    // バックをしない
                        
                        twoWheel robot = new twoWheel(current.robot);
                        robot.calculatePosition(velocity_left, velocity_right, sampling_time);
                        if ((robot.x >= max_x) || (robot.x < 0) || (robot.y >= max_y) || (robot.y < 0)) continue;
                        if (getMapData(robot.x, robot.y) != 0) continue;
                        Debug.Write(current.our_num.ToString() + ", " + robot.x.ToString() + ", " + robot.y.ToString() + ", " + robot.velocity_left.ToString() + ", " + robot.velocity_right.ToString() + ", " + current.cost.ToString() + ", " + current.goal_cost.ToString() + "\n");

                        Node node = new Node(robot.x, robot.y, robot.the, robot.velocity_left, robot.velocity_right, 0, current.our_num);
                        double xd = robot.x - current.robot.x;
                        double yd = robot.y - current.robot.y;
                        double fd = current.cost + node.calculateGoalCost(goal_x, goal_y) + 0.5 * 0.1;
//                            + Math.Sqrt(xd * xd + yd * yd) * 2;
                        int n;
                        if (checkExist(openList, robot.x, robot.y, robot.the, robot.velocity_left, robot.velocity_right, out n))
                        {
                            if (fd < openList[n].getHeuristic())
                            {
//                                Debug.Write("openList\n");
                                openList[n].cost = fd - node.goal_cost;
                                openList[n].pre_num = current.our_num;
                            }
                        }
                        else if (checkExist(closeList, robot.x, robot.y, robot.the, robot.velocity_left, robot.velocity_right, out n))
                        {
                            if (fd < closeList[n].getHeuristic())
                            {
//                                Debug.Write("closeList\n");
                                Node node0 = closeList[n];
                                node0.cost = fd - node.goal_cost;
                                node0.pre_num = current.our_num;
                                openList.Add(node0);
                                closeList.RemoveAt(n);
                            }
                        }
                        else
                        {
                            Node node1 = new Node(robot.x, robot.y, robot.the, robot.velocity_left, robot.velocity_right, node_no++, current.our_num);
                            node1.calculateGoalCost(goal_x, goal_y);
                            node1.cost = fd - node1.goal_cost;
                            openList.Add(node1);
//                            Debug.Write(node_no.ToString() + "\n");
                        }
                    }
                }
            }
            while (true)
            {
                PosVel pos = new PosVel();
                twoWheel robot = current.robot;
                pos.x = robot.x;
                pos.y = robot.y;
                pos.the = robot.the;
                pos.velocity_left = robot.velocity_left;
                pos.velocity_right = robot.velocity_right;
                path.Add(pos);
                int n = current.pre_num;
                if (n == -1) break;
                int index = getIndex(closeList, current.pre_num);
                current = closeList[index];
            }
            return true;
        }

        public List<PosVel> getPath()
        {
            return path;
        }
    }
}
