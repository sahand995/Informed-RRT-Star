using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InformedRRTStar
{
    public class InformedRRTStar
    {
        public Node start;
        public Node goal;
        public double minRand;
        public double maxRand;
        public double expandDis;
        public int goalSampleRate;
        public int maxIter;
        public double[,] obstacleList;
        public List<Node> node_List;
        Random random = new Random();

        public InformedRRTStar(double[] start_pos, double[] goal_pos, double[,] obstacleList, double[] randArea,
            double expandDis = 0.5, int goalSampleRate = 10, int maxIter = 200)
        {
            start = new Node(start_pos[0], start_pos[1]);
            goal = new Node(goal_pos[0], goal_pos[1]);
            minRand = randArea[0];
            maxRand = randArea[1];
            this.expandDis = expandDis;
            this.goalSampleRate = goalSampleRate;
            this.maxIter = maxIter;
            this.obstacleList = obstacleList;
            node_List = new List<Node>();
        }

        public List<Node> informed_rrt_star_search()
        {
            node_List.Add(start);
            double cBest = double.PositiveInfinity;
            //var solutionSet = new HashSet<Node>();
            List<Node> path = new List<Node>();
            double cMin = Math.Sqrt(Math.Pow(start.x - goal.x, 2) + Math.Pow(start.y - goal.y, 2));

            double[,] xCenter = { { (start.x + goal.x) / 2.0 },
                                  { (start.y + goal.y) / 2.0 },
                                  { 0 } };

            double[,] a1 = { { (goal.x - start.x) / cMin }, 
                             { (goal.y - start.y) / cMin },
                             { 0 } };

            double etheta = Math.Atan2(a1[1, 0] , a1[0, 0]);

            double[,] id1_t = new double[1, 3] { {1, 0, 0} };

            double[,] M = new double[3, 3];                         //np.dot(a1, idt_1)
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    M[i, j] = 0;
                    for (int k = 0; k < 1; k++)
                    {
                        M[i, j] += a1[i, k] * id1_t[k, j];
                    }
                }
            }

            //U, S, Vh
            //C

            double[,] C = { { M[0,0], -1 * M[1,0], 0 },
                            { M[1,0],  M[0,0], 0 },
                            { 0, 0, 1 } };

            for (int i = 0; i < maxIter; i++)
            {
                double[] rnd = informed_sample(cBest, cMin, xCenter, C);
                int nind = get_nearest_list_index(node_List, rnd);
                Node nearestNode = node_List[nind];
                double theta = Math.Atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x);
                Node newNode = get_new_node(theta, nind, nearestNode);
                double d = line_cost(nearestNode, newNode);
                bool noCollision = check_collision(nearestNode, theta, d);

                if (noCollision)
                {
                    List<int> nearInds = find_near_nodes(newNode);
                    newNode = choose_parent(newNode, nearInds);
                    node_List.Add(newNode);
                    Console.WriteLine($"{node_List.Count - 1} - ({newNode.x}, {newNode.y}),  {newNode.parent}");
                    rewire(newNode, nearInds);

                    if (is_near_goal(newNode))
                    {
                        if (check_segment_collision(newNode.x, newNode.y, goal.x, goal.y))
                        {
                            //solutionSet.Add(newNode);
                            int lastIndex = node_List.Count - 1;
                            List<Node> tempPath = get_final_course(lastIndex);
                            double tempPathLen = get_path_len(tempPath);

                            if (tempPathLen < cBest)
                            {
                                path = tempPath;
                                cBest = tempPathLen;
                            }
                        }
                    }
                }
            }

            return path;
        }

        #region [- choose_parent(Node newNode, List<int> nearInds) -]
        public Node choose_parent(Node newNode, List<int> nearInds)
        {
            if (nearInds.Count == 0)
            {
                return newNode;
            }

            List<double> dList = new List<double>();

            for (int i = 0; i < nearInds.Count; i++)
            {
                double dx = newNode.x - node_List[nearInds[i]].x;
                double dy = newNode.y - node_List[nearInds[i]].y;
                double d = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2));
                double theta = Math.Atan2(dy, dx);

                if (check_collision(node_List[nearInds[i]], theta, d))
                {
                    dList.Add(node_List[nearInds[i]].cost + d);
                }
                else
                {
                    dList.Add(double.PositiveInfinity);
                }
            }

            double minCost = dList.Min();
            int minInd = nearInds[dList.IndexOf(minCost)];

            if (minCost == double.PositiveInfinity)
            {
                return newNode;
            }

            newNode.cost = minCost;
            newNode.parent = minInd;

            return newNode;
        } 
        #endregion

        #region [- find_near_nodes(Node newNode) -]
        public List<int> find_near_nodes(Node newNode)
        {
            int nnode = node_List.Count;
            double r = 50 * Math.Sqrt(Math.Log(nnode) / nnode);

            List<double> dlist = new List<double>();
            List<int> nearinds = new List<int>();

            foreach (var item in node_List)
            {
                dlist.Add(Math.Pow(item.x - newNode.x, 2) + Math.Pow(item.y - newNode.y, 2));
            }

            for (int i = 0; i < dlist.Count; i++)
            {
                if (dlist[i] <= Math.Pow(r, 2))
                {
                    nearinds.Add(i);
                }
            }

            return nearinds;
        } 
        #endregion

        #region [- informed_sample(double cMax, double cMin, double[,] xCenter, double[,] C) -]
        public double[] informed_sample(double cMax, double cMin, double[,] xCenter, double[,] C)
        {
            if (cMax < double.PositiveInfinity)
            {
                double[] r = {cMax / 2.0,
                              Math.Sqrt(Math.Pow(cMax,2) - Math.Pow(cMin,2)) / 2.0,
                              Math.Sqrt(Math.Pow(cMax,2) - Math.Pow(cMin,2)) / 2.0};

                double[,] L = new double[3, 3];
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (i == 0 && j == 0)
                        {
                            L[i, j] = r[0];
                        }
                        else if (i == 1 && j == 1)
                        {
                            L[i, j] = r[1];
                        }
                        else if (i == 2 && j == 2)
                        {
                            L[i, j] = r[2];
                        }
                        else
                        {
                            L[i, j] = 0;
                        }
                    }
                }

                double[,] xBall = sample_unit_ball();

                double[,] multiply_C_L = new double[3, 3];
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        multiply_C_L[i, j] = 0;
                        for (int k = 0; k < 3; k++)
                        {
                            multiply_C_L[i, j] += C[i, k] * L[k, j];
                        }
                    }
                }

                double[,] multiply_multiply_C_L_xBall = new double[3, 1];
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 1; j++)
                    {
                        multiply_multiply_C_L_xBall[i, j] = 0;
                        for (int k = 0; k < 3; k++)
                        {
                            multiply_multiply_C_L_xBall[i, j] += multiply_C_L[i, k] * xBall[k, j];
                        }
                    }
                }

                double[,] rnd = new double[3, 1];
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 1; j++)
                    {
                        rnd[i, j] = multiply_multiply_C_L_xBall[i, j] + xCenter[i, j];
                    }
                }

                return new double[2] { rnd[0, 0], rnd[1, 0] };
            }

            else
            {
                return sample_free_space();
            }
        } 
        #endregion

        #region [- sample_unit_ball() -]
        public static double[,] sample_unit_ball()
        {            
            Random random = new Random();

            double a = random.NextDouble();
            double b = random.NextDouble();

            if (b < a)
            {
                double temp = a;
                a = b;
                b = temp;
            }

            double[] sample = { b * Math.Cos(2 * Math.PI * (a / b)), b * Math.Sin(2 * Math.PI * (a / b)) };

            return new double[3, 1] { { sample[0] },
                                      { sample[1] },
                                      { 0 } };
        }
        #endregion

        #region [- sample_free_space() -]
        public double[] sample_free_space()
        {
            double[] rnd;

            if (random.Next(0, 100) > goalSampleRate)
            {
                rnd = new double[2] { random.NextDouble() * (maxRand - minRand) + minRand,
                                      random.NextDouble() * (maxRand - minRand) + minRand };
            }
            else
            {
                rnd = new double[2] { goal.x, goal.y };
            }

            return rnd;
        }
        #endregion

        #region [- get_path_len(List<Node> path) -]
        public static double get_path_len(List<Node> path)
        {
            double pathLen = 0;
            for (int i = 1; i < path.Count; i++)
            {
                double node1_x = path[i].x;
                double node1_y = path[i].y;
                double node2_x = path[i - 1].x;
                double node2_y = path[i - 1].y;

                pathLen += Math.Sqrt(Math.Pow(node1_x - node2_x, 2) + Math.Pow(node1_y - node2_y, 2));
            }

            return pathLen;
        } 
        #endregion

        #region [- line_cost(Node node1, Node node2) -]
        public static double line_cost(Node node1, Node node2)
        {
            return Math.Sqrt(Math.Pow(node1.x - node2.x, 2) + Math.Pow(node1.y - node2.y, 2));
        } 
        #endregion

        #region [- get_nearest_list_index(Node[] nodes, double[] rnd) -]
        public static int get_nearest_list_index(List<Node> nodes, double[] rnd)
        {
            List<double> dList = new List<double>();

            foreach (var item in nodes)
            {
                dList.Add(Math.Pow(item.x - rnd[0], 2) + Math.Pow(item.y - rnd[1], 2));
            }

            int minIndex = dList.IndexOf(dList.Min());
            return minIndex;
        }
        #endregion

        #region [- get_new_node(double theta, int nind, Node nearestnode) -]
        public Node get_new_node(double theta, int nind, Node nearestnode)
        {
            Node newNode = nearestnode.DeepCopy();
            newNode.x += expandDis * Math.Cos(theta);
            newNode.y += expandDis * Math.Sin(theta);
            newNode.cost += expandDis;
            newNode.parent = nind;
            return newNode;
        }
        #endregion

        #region [- is_near_goal(Node node) -]
        public bool is_near_goal(Node node)
        {
            double d = line_cost(node, goal);

            if (d < expandDis)
            {
                return true;
            }

            return false;
        } 
        #endregion

        #region [- rewire(Node newNode, List<int> nearInds) -]
        public void rewire(Node newNode, List<int> nearInds)
        {
            int n_node = node_List.Count;

            for (int i = 0; i < nearInds.Count; i++)
            {
                Node nearNode = node_List[nearInds[i]];

                double d = Math.Sqrt(Math.Pow(nearNode.x - newNode.x, 2) + Math.Pow(nearNode.y - newNode.y, 2));
                double scost = newNode.cost + d;

                if (nearNode.cost > scost)
                {
                    double theta = Math.Atan2(newNode.y - nearNode.y, newNode.x - nearNode.x);

                    if (check_collision(nearNode, theta, d))
                    {
                        nearNode.parent = n_node - 1;
                        nearNode.cost = scost;
                    }
                }
            }
        } 
        #endregion

        #region [- distance_squared_point_to_segment(double[] v, double[] w, double[] p) -]
        public static double distance_squared_point_to_segment(double[] v, double[] w, double[] p)
        {
            
            double[] p_v = new double[2] { p[0] - v[0], p[1] - v[1] };          //p-v

            if (v[0] == w[0] && v[1] == w[1])
            {
                return Math.Pow(p_v[0], 2) + Math.Pow(p_v[1], 2);               //(p-v).dot(p-v)
            }
            else
            {
                double[] w_v = new double[2] { w[0] - v[0], w[1] - v[1] };      //w-v
                double l2 = Math.Pow(w_v[0], 2) + Math.Pow(w_v[1], 2);          //(w-v).dot(w-v)

                double temp = ((p_v[0] * w_v[0]) + (p_v[1] * w_v[1])) / l2;     //(p-v).dot(w-v) / l2

                double t = Math.Max(0, Math.Min(1, temp));                      // 0 < t < 1

                double[] projection = new double[2] { (w_v[0] * t) + v[0], (w_v[1] * t) + v[1] };

                double[] p_projection = new double[2] { p[0] - projection[0], p[1] - projection[1] };   //p_projection

                return Math.Pow(p_projection[0], 2) + Math.Pow(p_projection[1], 2);
            }
        }
        #endregion

        #region [- check_segment_collision(double x1, double y1, double x2, double y2) -]
        public bool check_segment_collision(double x1, double y1, double x2, double y2)
        {
            for (int i = 0; i < obstacleList.Length / 3; i++)
            {
                double dd = distance_squared_point_to_segment(new double[] { x1, y1 }, new double[] { x2, y2 },
                         new double[] { obstacleList[i, 0], obstacleList[i, 1] });

                if (dd <= Math.Pow(obstacleList[i, 2], 2))
                {
                    return false;
                }
            }

            return true;
        } 
        #endregion

        #region [- check_collision(Node nearNode, double theta, double d) -]
        public bool check_collision(Node nearNode, double theta, double d)
        {
            Node tmpNode = nearNode.DeepCopy();
            double endx = tmpNode.x + Math.Cos(theta) * d;
            double endy = tmpNode.y + Math.Sin(theta) * d;

            return check_segment_collision(tmpNode.x, tmpNode.y, endx, endy);
        }
        #endregion

        #region [- get_final_course(int lastIndex) -]
        public List<Node> get_final_course(int lastIndex)
        {
            List<Node> path = new List<Node>();
            path.Add(goal);

            while (node_List[lastIndex].parent != -1)
            {
                Node node = node_List[lastIndex];
                path.Add(node);
                lastIndex = node.parent;
            }

            path.Add(start);
            return path;
        }
        #endregion



        //public void draw_graph(double xCenter = 0,
        //    double cBest = 0, double cMin = 0, double etheta = 0, double rnd = 0)
        //{

        //}

        //public static void plot_ellipse(double xCenter, double cBest, double cMin, double etheta)
        //{

        //}

    }
}
