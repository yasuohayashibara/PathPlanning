using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
using GeneratePath;

namespace AStarSearch
{
    /// <summary>
    /// ノードのクラス
    /// </summary>
    class Node
    {
        public int our_num;         // このノードの番号
        public int pre_num;         // 一つ前のノードの番号
        public int x;               // x座標
        public int y;               // y座標
        public double cost;         // スタートノードからの最小コストの推定値
        public double goal_cost;    // ゴールノードまでの最小コストの推定値

        /// <summary>
        /// コンストラクタ
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        /// <param name="our_num">このノードの番号</param>
        /// <param name="pre_num">一つ前のノードの番号</param>
        public Node(int x, int y, int our_num, int pre_num)
        {
            setPosition(x, y);
            this.our_num = our_num;
            this.pre_num = pre_num;
        }
        
        /// <summary>
        /// 位置の設定
        /// </summary>
        /// <param name="x">x座標の値</param>
        /// <param name="y">y座標の値</param>
        public void setPosition(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        /// <summary>
        /// ゴールまでの最小コストの推定値を計算する
        /// </summary>
        /// <param name="goal_x">ゴールのx座標</param>
        /// <param name="goal_y">ゴールのy座標</param>
        /// <returns>ゴールまでの最小コストの推定値</returns>
        public double calculateGoalCost(int goal_x, int goal_y)
        {
            goal_cost = Math.Abs(goal_x - x) + Math.Abs(goal_y - y);
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
    }

    /// <summary>
    /// A Star Searchを行うクラス
    /// </summary>
    class AStar
    {
        byte[] map;
        int width;
        int height;
        int start_x;
        int start_y;
        int goal_x;
        int goal_y;
        List<Pos> path;

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
        public bool setStartPosition(int x, int y)
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

        /// <summary>
        /// (x,y)の場所にノードが存在するかをチェックする関数
        /// </summary>
        /// <param name="list">ノードのリスト</param>
        /// <param name="x">x座標</param>
        /// <param name="y">y座標</param>
        /// <param name="n">ノードの番号</param>
        /// <returns>ノードが存在するかどうか(0:存在しない，1:存在する)</returns>
        private bool checkExist(List<Node> list, int x, int y, out int n)
        {
            n = -1;
            for (int i = 0; i < list.Count; i++)
            {
                if ((list[i].x == x) && (list[i].y == y))
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
            List<Node> openList = new List<Node>();             // 計算中のノードを格納するためのリスト
            List<Node> closeList = new List<Node>();            // 計算済みのノードを格納しておくリスト
            Node start = new Node(start_x, start_y, 0, -1);     // スタートノードを設定
            Node goal = new Node(goal_x, goal_y, 0, 0);         // ゴールノードを設定
            start.calculateGoalCost(goal_x, goal_y);            // ゴールまでのコストの推定値を計算
            start.cost = 0;                                     // スタートからのコストは0
            openList.Add(start);                                // スタートノードをOpenリストに追加
            Node current = new Node(0, 0, 0, 0);                // 現在，取り出して計算しているノード
            int node_no = 1;
            while (true)
            {
                if (openList.Count == 0) return false;          // ゴールに辿り着くルートが無い
                int min_no = minHeuristic(openList);            // 最小のヒューリスティックのノード番号を取得
                current = openList[min_no];                     // そのノードを取り出す
                openList.RemoveAt(min_no);                      // Openリストから削除して，
                closeList.Add(current);                         // Closeリストに追加する

                // ゴールに到着した場合ループを抜ける
                if ((current.x == goal.x) && (current.y == goal.y)) break;

                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        if ((x != 0)&&(y != 0)) continue;
                        int xc = current.x + x;
                        int yc = current.y + y;
                        if ((xc >= width) || (xc < 0) || (yc >= height) || (yc < 0)) continue;
                        if (map[yc * width + xc] != 0) continue;    // 障害物があれば次へ

                        Node node = new Node(xc, yc, 0, current.our_num);
                        double fd = current.cost + node.calculateGoalCost(goal_x, goal_y) + len(x,y);
                        int n;
                        if (checkExist(openList, xc, yc, out n))
                        {
                            if (fd < openList[n].getHeuristic())
                            {
                                openList[n].cost = fd - node.goal_cost;
                                openList[n].pre_num = current.our_num;
                            }
                        }
                        else if (checkExist(closeList, xc, yc, out n))
                        {
                            if (fd < closeList[n].getHeuristic())
                            {
                                Node node0 = closeList[n];
                                node0.cost = fd - node.goal_cost;
                                node0.pre_num = current.our_num;
                                openList.Add(node0);
                                closeList.RemoveAt(n);
                            }
                        }
                        else
                        {
                            Node node1 = new Node(xc, yc, node_no++, current.our_num);
                            node1.calculateGoalCost(goal_x, goal_y);
                            node1.cost = fd - node1.goal_cost;
                            openList.Add(node1);
                        }
                    }
                }
            }
            while (true)
            {
                Pos pos = new Pos();
                pos.x = current.x;
                pos.y = current.y;
                path.Add(pos);
                int n = current.pre_num;
                if (n == -1) break;
                current = closeList[getIndex(closeList, current.pre_num)];
            }
            return true;
        }

        public List<Pos> getPath()
        {
            return path;
        }
    }
}
