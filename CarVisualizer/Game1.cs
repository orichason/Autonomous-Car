using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Intrinsics.X86;
using MonoGame.Extended;
using MonoGame.Extended.Shapes;
using MonoGame.Extended.Screens;
using System.Threading;

namespace CarVisualizer
{
    public class Game1 : Game
    {
        private GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        private Vector2 center;
        private List<(float, ushort)> points = new();
        private float pixelsPerMeter = 120f;
        private int head = 0;
        private int chunkSize = 10;
        private int trailLength = 600;
        private const double intervalMs = 10;
        private double accumulatorMs = 0;
        private List<(float, ushort)> chunkList = new();



        private const string path = "C:\\Users\\orich\\OneDrive\\Desktop\\PlaybackLog.bin";


        void LoadLog(string path)
        {
            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                using (BinaryReader reader = new BinaryReader(fs))
                {
                    while (fs.Position + 6 < fs.Length) //reads until file is fully read
                    {
                        float angle = reader.ReadSingle();
                        ushort distance = reader.ReadUInt16();
                        points.Add((angle, distance));
                    }
                }
            }
        }
        public Game1()
        {
            _graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
        }

        protected override void Initialize()
        {
            // TODO: Add your initialization logic here

            if (!File.Exists(path))
            {
                throw new Exception("File doesn't exist");
            }

            base.Initialize();
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);
            center = new(GraphicsDevice.Viewport.Width / 2f, GraphicsDevice.Viewport.Height /2f);

            LoadLog(path);

            using (StreamWriter writer = new("C:\\Users\\orich\\OneDrive\\Desktop\\points_dump.txt"))
            {
                foreach (var (angle, distance) in points)
                {
                    writer.WriteLine($"Angle={angle}, Distance={distance}");
                }
            }

            // TODO: use this.Content to load your game content here
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            accumulatorMs += gameTime.ElapsedGameTime.TotalMilliseconds;

            // TODO: Add your update logic here

            while (accumulatorMs >= intervalMs && head < points.Count)
            {
                int end = Math.Min(head + chunkSize, points.Count);

                for (int i = head; i < end; i++)
                    chunkList.Add(points[i]);

                head = end;
                accumulatorMs -= intervalMs;


                int overflow = chunkList.Count - trailLength;
                if(overflow > 0)
                {
                    chunkList.RemoveRange(0, overflow);
                }

            }


            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.Black);
            _spriteBatch.Begin();

            const float dotSize = 3f;


            foreach (var (angle, distance) in chunkList)
            {
                float meters = distance * 0.001f;
                float radius = meters * pixelsPerMeter;

                float rad = MathHelper.ToRadians(angle);
                float x = center.X + radius * (float)Math.Cos(rad);
                float y = center.Y - radius * (float)Math.Sin(rad);

                _spriteBatch.FillRectangle(new RectangleF(
                   x - dotSize * 0.5f, y - dotSize * 0.5f, dotSize, dotSize), Color.White);
            }

            _spriteBatch.FillRectangle(new RectangleF(center.X - 6, center.Y - 6, 12, 12), Color.Orange);


            // TODO: Add your drawing code here

            _spriteBatch.End();
            base.Draw(gameTime);
        }
    }
}
