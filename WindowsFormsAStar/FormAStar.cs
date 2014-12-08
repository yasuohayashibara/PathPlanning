using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Diagnostics;
using GenaratePath;
using System.Drawing.Drawing2D;

namespace WindowsFormsAStar
{
    public partial class FormAStar : Form
    {
        Map map = new Map();
        AStar astar = new AStar();

        public FormAStar()
        {
            InitializeComponent();
            map.loadMap("..\\..\\..\\map.bmp", 0.01);
            astar.setMap(map.getData(), map.width, map.height);
            astar.setStartPosition(0, 5);
            astar.setGoalPosition(map.width - 1, map.height - 1);
            if (astar.calculatePath())
            {
                List<Pos> path = astar.getPath();
                map.setPath(astar.getPath());
            }
            Bitmap bmp = map.getBitmap(100,100);
            pictureBoxMap.Image = bmp;
            bmp.Save("test.bmp");
        }
    }
}
