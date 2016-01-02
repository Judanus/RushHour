using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RushHourSolver
{
    static class Program
    {
        static int targetVehicle, goal;
        static int numHorVec;
        static byte[] vehicleLocs;
        static byte[] vehicleLengths;
        static char[] vehicleNames;
        static byte[][] vehicleRows;
        static byte[][] vehicleCols;
        static byte[] vehicleStartPos;
        static Trie visited;
        static Solution foundSolution;
        static bool solveMode;
        
        static void Main(string[] args)
        {
            ReadInput();
            Queue<Tuple<byte[], Solution>> q = new Queue<Tuple<byte[], Solution>>();
            foundSolution = new NoSolution();
            q.Enqueue(Tuple.Create(vehicleStartPos, (Solution)new EmptySolution()));
            AddNode(vehicleStartPos);
                        
            while (q.Count > 0)
            {
                Tuple<byte[], Solution> currentState = q.Dequeue();
                foreach (Tuple<byte[], Solution> next in Sucessors(currentState))
                {
                    if (next.Item1[targetVehicle] == goal)
                    {
                        q.Clear();
                        foundSolution = next.Item2;
                        break;
                    }
                    if (!AddNode(next.Item1))
                        q.Enqueue(next);
                }
            }
            Console.WriteLine(foundSolution);
            Console.ReadLine();
        }

        private static IEnumerable<Tuple<byte[], Solution>> Sucessors(Tuple<byte[], Solution> state)
        {
            byte[] currentState = state.Item1;
            Solution solution = state.Item2;
            for (byte v = 0; v < vehicleNames.Length; v++)
            {
                byte[] sameDir;
                byte[][] diffDir;
                int size;
                char dir;
                if (v < numHorVec)
                {
                    size = vehicleCols.GetLength(0);
                    sameDir = vehicleRows[vehicleLocs[v]];
                    diffDir = vehicleCols;
                    dir = 'r';
                }
                else
                {
                    size = vehicleRows.GetLength(0);
                    sameDir = vehicleCols[vehicleLocs[v]];
                    diffDir = vehicleRows;
                    dir = 'd';
                }
                for (byte i = 1; i <= size - (currentState[v] + vehicleLengths[v]); i++)
                {
                    bool good = true;
                    foreach (int j in sameDir) if (j != v && currentState[j] == currentState[v] + i + vehicleLengths[v] - 1) good = false;
                    foreach (int j in diffDir[currentState[v] + vehicleLengths[v] + i - 1]) if (currentState[j] <= vehicleLocs[v] && currentState[j] + vehicleLengths[j] - 1 >= vehicleLocs[v]) good = false;
                    if (!good) break;
                    byte[] newState = (byte[])currentState.Clone();
                    newState[v] += i;
                    if (AddNode(newState))
                        yield return Tuple.Create(newState, solution.appendMove(v < numHorVec, true, i, v));
                }
                if (dir == 'r')
                    dir = 'l';
                else
                    dir = 'u';
                for (byte i = 1; i <= currentState[v]; i++)
                {
                    bool good = true;
                    foreach (int j in sameDir) if (j != v && currentState[j] + vehicleLengths[j] - 1 == currentState[v] - i) good = false;
                    foreach (int j in diffDir[currentState[v] - i]) if (currentState[j] <= vehicleLocs[v] && currentState[j] + vehicleLengths[j] - 1 >= vehicleLocs[v]) good = false;
                    if (!good) break;
                    byte[] newState = (byte[])currentState.Clone();
                    newState[v] -= i;
                    if (AddNode(newState))
                        yield return Tuple.Create(newState, solution.appendMove(v < numHorVec, false, i, v));
                }
            }
            yield break;
        }

        static void ReadInput()
        {
            solveMode = Console.ReadLine() == "1";
            string[] cf = Console.ReadLine().Split();
            int cols = int.Parse(cf[0]); int rows = int.Parse(cf[1]);
            cf = Console.ReadLine().Split();
            int targetX = int.Parse(cf[0]);
            int targetY = int.Parse(cf[1]);
            vehicleNames = new char[0];
            vehicleLengths = new byte[0];
            vehicleStartPos = new byte[0];
            vehicleLocs = new byte[0];
            numHorVec = 0;
            byte vehicleId = 0;

            string[] field = new string[rows + 1];
            vehicleRows = new byte[rows][];
            for (byte i = 0; i < rows; i++)
            {
                vehicleRows[i] = new byte[0];
                field[i] = Console.ReadLine() + ".";
                byte runStart = 0; char runChar = '.';
                for (byte j = 0; j < field[i].Length; j++)
                {
                    if (field[i][j] != runChar)
                    {
                        if (runChar != '.' && j - runStart > 1)
                        {
                            vehicleNames = ArrayAdd(runChar, vehicleNames);
                            if (runChar == 'x') targetVehicle = vehicleId;
                            vehicleLengths = ArrayAdd((byte)(j - runStart), vehicleLengths);
                            vehicleStartPos = ArrayAdd(runStart, vehicleStartPos);
                            vehicleRows[i] = ArrayAdd(vehicleId, vehicleRows[i]);
                            vehicleLocs = ArrayAdd(i, vehicleLocs);
                            vehicleId++;
                            numHorVec++;
                        }
                        runStart = j;
                        runChar = field[i][j];
                    }
                }
            }
            if (cols != field[0].Length - 1)
                throw new Exception("This testcase is bad!");
            field[rows] = new String('.', cols);
            vehicleCols = new byte[cols][];
            for (byte i = 0; i < cols; i++)
            {
                vehicleCols[i] = new byte[0];
                byte runStart = 0; char runChar = '.';
                for (byte j = 0; j < rows + 1; j++)
                {
                    if (field[j][i] != runChar)
                    {
                        if (runChar != '.' && j - runStart > 1)
                        {
                            vehicleNames = ArrayAdd(runChar, vehicleNames);
                            if (runChar == 'x') targetVehicle = vehicleId;
                            vehicleLengths = ArrayAdd((byte)(j - runStart), vehicleLengths);
                            vehicleStartPos = ArrayAdd(runStart, vehicleStartPos);
                            vehicleCols[i] = ArrayAdd(vehicleId, vehicleCols[i]);
                            vehicleLocs = ArrayAdd(i, vehicleLocs);
                            vehicleId++;
                        }
                        runStart = j;
                        runChar = field[j][i];
                    }
                }
            }
            if (numHorVec == 0)
                visited = new Trie(rows - vehicleLengths[0] + 1);
            else
                visited = new Trie(cols - vehicleLengths[0] + 1);

            if (targetVehicle < numHorVec)
            {
                goal = targetX;
                if (vehicleLocs[targetVehicle] != targetY)
                    throw new Exception("This testcase is bad!");
            }
            else
            {
                goal = targetY;
                if (vehicleLocs[targetVehicle] != targetX)
                    throw new Exception("This testcase is bad!");
            }
        }

        static T[] ArrayAdd<T>(T i, T[] array)
        {
            T[] newArray = new T[array.Length + 1];
            Array.Copy(array, newArray, array.Length);
            newArray[array.Length] = i;
            return newArray;
        }

        static bool AddNode(byte[] node)
        {
            Trie cur = visited;
            for (byte i = 0; i < node.Length; i++)
            {
                if (cur.Leaves[node[i]] == null)
                {
                    Trie newT;
                    if (i == node.Length - 1)
                        newT = new Trie(0);
                    else if (i < numHorVec)
                        newT = new Trie(vehicleCols.GetLength(0) - vehicleLengths[i + 1] + 1);
                    else
                        newT = new Trie(vehicleRows.GetLength(0) - vehicleLengths[i + 1] + 1);
                    if (cur.Leaves[node[i]] == null)
                    {
                        cur.Leaves[node[i]] = newT;
                        if (i == node.Length - 1) return true;
                        cur = newT;
                    }
                    else
                        cur = cur.Leaves[node[i]];
                }
                else
                    cur = cur.Leaves[node[i]];
            }
            return false;
        }

        class Trie
        {
            public Trie[] Leaves { get; protected set; }

            public Trie(int s)
            {
                Leaves = new Trie[s];
            }
        }

        class Solution
        {
            public Solution Parent { get; protected set; }
            public bool Direction { get; protected set; }
            public bool Forward { get; protected set; }
            public byte Amount { get; protected set; }
            public byte Vehicle { get; protected set; }
            public int Count { get; protected set; }
            public Solution() { }
            public Solution appendMove(bool direction, bool forward, byte amount, byte vehicle)
            {
                if (!solveMode)
                    return new Solution() { Count = this.Count + 1 };
                return new Solution()
                {
                    Parent = this,
                    Direction = direction,
                    Forward = forward,
                    Amount = amount,
                    Vehicle = vehicle
                };
            }

            public override string ToString()
            {
                return MakeString().Trim();
            }

            protected virtual string MakeString()
            {
                if (!solveMode)
                    return Count.ToString();
                else
                    return Parent.MakeString() + vehicleNames[Vehicle] + (Direction ? (Forward ? 'r' : 'l') : (Forward ? 'd' : 'u')) + Amount.ToString() + " ";
            }
        }

        class EmptySolution : Solution
        {
            protected override string MakeString()
            {
                return "";
            }

            public override string ToString()
            {
                return solveMode ? "" : base.ToString();
            }
        }

        class NoSolution : Solution
        {
            public override string ToString()
            {
                return solveMode ? "Geen oplossing gevonden" : "-1";
            }
        }
    }
}
