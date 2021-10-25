using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InformedRRTStar
{
    public class Node
    {
        public double x;
        public double y;
        public double cost;
        public int parent;

        public Node(double x, double y)
        {
            this.x = x;
            this.y = y;
            cost = 0.0;
            parent = -1;
        }

        public Node DeepCopy()
        {
            Node temp = (Node)this.MemberwiseClone();
            temp.x = x;
            temp.y = y;
            temp.cost = cost;
            temp.parent = parent;
            return temp;
        }
    }
}
