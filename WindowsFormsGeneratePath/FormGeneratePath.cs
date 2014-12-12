using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using GeneratePath;
using System.Diagnostics;

namespace WindowsFormsGeneratePath
{
    public partial class FormGeneratePath : Form
    {
        Map map = new Map(0.05);
        AStar astar = new AStar();

        public FormGeneratePath()
        {
            InitializeComponent();
            map.loadMap("..\\..\\..\\map_sample.bmp", 0.5);
            astar.setMap(map.getData(), map.width, map.height, 0.05);
            astar.setStartPosition(0, 2.5, 0);
            astar.setGoalPosition(4.5, 2.5, 0);
            if (astar.calculatePath())
            {
                List<PosVel> path = astar.getPath();
                map.setPath(astar.getPath());
                for (int i = 0; i < path.Count; i++)
                {
                    Debug.Write("(" + path[i].x.ToString() + " , " + path[i].y.ToString() + " , " + path[i].the.ToString() + " , " + path[i].velocity_left.ToString() + " , " + path[i].velocity_right.ToString() + ")\n");
                }
            }
            Bitmap bmp = map.getBitmap(100, 100);
            pictureBoxMap.Image = bmp;
            bmp.Save("test.bmp");
        }
    }
}
