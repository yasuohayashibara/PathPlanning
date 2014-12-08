﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Diagnostics;

namespace GenaratePath
{
    class Map
    {
        byte[] map;         // 地図データ（0:障害物なし or 1：障害物あり）
        int width;          // 幅（データの個数）
        int height;         // 高さ（データの個数）
        double unit;        // グリッドの一辺の距離(m)

        /// <summary>
        /// コンストラクタ
        /// </summary>
        public Map()
        {
            Initialize();
        }

        /// <summary>
        /// グリッドの一辺の距離を設定するコンストラクタ
        /// </summary>
        /// <param name="unit">グリッドの一辺の距離(m)</param>
        public Map(double unit)
        {
            Initialize();
            this.unit = unit;
        }

        /// <summary>
        /// 初期化
        /// </summary>
        public void Initialize()
        {
            map = new byte[1];
            width = 1;
            height = 1;
            unit = 0.01;            // 10mmをデフォルトの単位とする．
        }

        /// <summary>
        /// ビットマップファイルから地図を生成する．
        /// </summary>
        /// <param name="filename">ビットマップファイル名</param>
        public void loadMap(string filename, double pixelUnit)
        {
            try
            {
                Bitmap bmp = (Bitmap)Image.FromFile(filename);
                byte[] image = BitmapToByteArray(bmp);
                double verticalLength = pixelUnit * bmp.Width;
                double horizontalLength = pixelUnit * bmp.Height;
                width = (int)(pixelUnit / unit * bmp.Width);
                height = (int)(pixelUnit / unit * bmp.Height);
                map = new byte[width * height];
                int num = 0;
                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        int xi = (int)(unit / pixelUnit * x);
                        int yi = (int)(unit / pixelUnit * y);
                        int n = yi * bmp.Width + xi;
                        if (image[n << 2] == 0)
                        {
                            map[num] = 1;
                        }
                        else
                        {
                            map[num] = 0;
                        }
                        num ++;
                    }
                }
            }
            catch (Exception) { }
        }

        /// <summary>
        /// 地図ファイルからビットマップを生成する
        /// </summary>
        /// <returns></returns>
        public Bitmap getBitmap()
        {
            Bitmap bmp = new Bitmap(width, height);
            byte[] image = new byte[width * height * 4];
            int n = 0;
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    if (map[n] == 1)
                    {
                        image[(n << 2) + 0] = 0;
                        image[(n << 2) + 1] = 0;
                        image[(n << 2) + 2] = 0;
                    }
                    else
                    {
                        image[(n << 2) + 0] = 255;
                        image[(n << 2) + 1] = 255;
                        image[(n << 2) + 2] = 255;
                    }
                    image[(n << 2) + 3] = 255;
                    n++;
                }
            }
            ByteArrayToBitmap(image, bmp);
            return bmp;
        }

        public int getValue(int x, int y)
        {
            return map[y * width + y];
        }

        /// <summary>
        /// Bitmapをbyte[]に変換する
        /// </summary>
        /// <param name="bitmap">変換元のBitmap</param>
        /// <returns>1 pixel = 4 byte (+3:A, +2:R, +1:G, +0:B) に変換したbyte配列</returns>
        public static byte[] BitmapToByteArray(Bitmap bmp)
        {
            Rectangle rect = new Rectangle(0, 0, bmp.Width, bmp.Height);
            System.Drawing.Imaging.BitmapData bmpData =
                bmp.LockBits(rect, System.Drawing.Imaging.ImageLockMode.ReadWrite,
                PixelFormat.Format32bppArgb);

            // Bitmapの先頭アドレスを取得
            IntPtr ptr = bmpData.Scan0;

            // 32bppArgbフォーマットで値を格納
            int bytes = bmp.Width * bmp.Height * 4;
            byte[] rgbValues = new byte[bytes];

            // Bitmapをbyte[]へコピー
            Marshal.Copy(ptr, rgbValues, 0, bytes);

            bmp.UnlockBits(bmpData);
            return rgbValues;
        }

        /// <summary>
        /// byte[]をBitmapに変換する
        /// </summary>
        /// <param name="byteArray">1 pixel = 4 byte (+3:A, +2:R, +1:G, +0:B) に変換したbyte配列</param>
        /// <param name="bmp">変換先のBitmap</param>
        /// <returns>変換先のBitmap</returns>
        public static Bitmap ByteArrayToBitmap(byte[] rgbValues, Bitmap bmp)
        {
            Rectangle rect = new Rectangle(0, 0, bmp.Width, bmp.Height);
            System.Drawing.Imaging.BitmapData bmpData =
                bmp.LockBits(rect, System.Drawing.Imaging.ImageLockMode.ReadWrite,
                PixelFormat.Format32bppArgb);

            // Bitmapの先頭アドレスを取得
            IntPtr ptr = bmpData.Scan0;

            // Bitmapへコピー
            Marshal.Copy(rgbValues, 0, ptr, rgbValues.Length);

            bmp.UnlockBits(bmpData);

            return bmp;
        }
    }
}