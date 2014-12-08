using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using GenaratePath;

namespace WindowsFormsAStar
{
    public partial class FormAStar : Form
    {
        Map map = new Map();

        public FormAStar()
        {
            InitializeComponent();
            map.loadMap("..\\..\\..\\map.bmp", 0.1);
            Bitmap bmp = map.getBitmap();
            pictureBoxMap.Image = bmp;
            bmp.Save("test.bmp");
        }
    }
}
