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
        bool isNeedCalculate = true;

        public FormGeneratePath()
        {
            InitializeComponent();
            map.loadMap("..\\..\\..\\map_sample.bmp", 0.5);
        }

        private void FormGeneratePath_Paint(object sender, PaintEventArgs e)
        {
            if (isNeedCalculate)
            {
                astar.Initialize();
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
                isNeedCalculate = false;
            }
            Bitmap bmp = map.getBitmap(100, 100);
            pictureBoxMap.Image = bmp;
            bmp.Save("test.bmp");
        }

        private void pictureBoxMap_MouseDown(object sender, MouseEventArgs e)
        {
            const int WIDTH = 10;
            byte value = 1;
            if (e.Button == MouseButtons.Right) value = 0; 
            int x = e.X * map.width / pictureBoxMap.Width / WIDTH;
            int y = e.Y * map.height / pictureBoxMap.Height / WIDTH;
            for (int i = 0; i < WIDTH; i++)
            {
                for (int j = 0; j < WIDTH; j++)
                {
                    map.setValue(x * WIDTH + j, y * WIDTH + i, value);
                }
            }
            Invalidate();
        }

        private void pictureBoxMap_MouseUp(object sender, MouseEventArgs e)
        {
            isNeedCalculate = true;
            Invalidate();
        }

        private void pictureBoxMap_MouseMove(object sender, MouseEventArgs e)
        {
            const int WIDTH = 10;
            byte value = 1;
            if ((e.Button != MouseButtons.Left) && (e.Button != MouseButtons.Right)) return;
            if (e.Button == MouseButtons.Right) value = 0;
            int x = e.X * map.width / pictureBoxMap.Width / WIDTH;
            int y = e.Y * map.height / pictureBoxMap.Height / WIDTH;
            for (int i = 0; i < WIDTH; i++)
            {
                for (int j = 0; j < WIDTH; j++)
                {
                    map.setValue(x * WIDTH + j, y * WIDTH + i, value);
                }
            }
            Invalidate();
        }
    }
}
