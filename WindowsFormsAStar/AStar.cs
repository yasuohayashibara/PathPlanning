using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GenaratePath
{
    /// <summary>
    /// ノードのクラス
    /// </summary>
    class Node
    {
        public int our_num;     // このノードの番号
        public int pre_num;     // 一つ前のノードの番号
        public int x;           // x座標
        public int y;           // y座標

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
            for (int n = 0; n < width * height; n++)
            {
                this.map[n] = map[n];
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

        public bool calculatePath()
        {
            List<Node> openList = new List<Node>();
            List<Node> closeList = new List<Node>();
            Node start = new Node(start_x, start_y, 0, -1);
            Node goal = new Node(goal_x, goal_y, 0, 0);
            Node current = new Node(0, 0, 0, 0);
            double min_cost = width * height;
            openList.Add(start);
            int openListNo = -1;
            double heuristic = width * height;
            while (true)
            {
                if (openList.Count == 0) return false;      // ゴールに辿り着くルートが無い
                for (int i = 0; i < openList.Count; i++)
                {
                    heuristic = len(goal.x - openList[i].x, goal.y - openList[i].y);
                    if (heuristic < min_cost)
                    {
                        openListNo = i;
                        min_cost = heuristic;
                        current = openList[i];
                    }
                }
                closeList.Add(current);

                // ゴールに到着した場合ループを抜ける
                if ((current.x == goal.x) && (current.y == goal.y)) break;

                openList.RemoveAt(openListNo);

                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        int xc = current.x + x;
                        int yc = current.y + y;
                        int n;
                        if (map[yc * width + xc] != 0) continue;    // 障害物があれば次へ
                        if (checkExist(openList, xc, yc, out n))
                        {
                            double h = len(goal.x - xc, goal.y - yc);
                            if (h < heuristic)
                            {
                                openList[n].pre_num = current.our_num;
                            }
                        }
                        else if (checkExist(closeList, xc, yc, out n))
                        {
                            double h = len(goal.x - xc, goal.y - yc);
                            if (h < heuristic)
                            {
                                Node node = closeList[n];
                                node.pre_num = current.our_num;
                                openList.Add(node);
                                closeList.RemoveAt(n);
                            }
                        }
                        else
                        {
                            Node node = new Node(xc, yc, c
                        }
                    }
                }



            }
            return true;
        }
    }
}
