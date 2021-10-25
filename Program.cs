using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InformedRRTStar
{
    class Program
    {
        static void Main(string[] args)
        {
            double[,] obstacleList = new double[,]
            {
                { 5, 5, 0.5 },
                { 9, 6, 1 },
                { 7, 5, 1 },
                { 1, 5, 1 },
                { 3, 6, 1 },
                { 7, 9, 1 }
            };


            InformedRRTStar rrt = new InformedRRTStar(new double[] { 0, 0 }, new double[] { 5, 10 },
                obstacleList, new double[] { -2, 15 });

            List<Node> path = rrt.informed_rrt_star_search();
            //Console.WriteLine("Path: {0}", string.Join(", ", path));

            Console.Write("Path: ");
            for (int i = path.Count - 1; i >= 0; i--)
            {
                Console.Write($"({path[i].x}, {path[i].y}), ");
            }
            Console.WriteLine();

            Console.WriteLine("Distance between start to end: {0}", InformedRRTStar.get_path_len(path));
            Console.WriteLine("Done!!");
            Console.ReadKey();

        }
    }
}
