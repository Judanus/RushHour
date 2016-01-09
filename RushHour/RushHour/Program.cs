using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using System.IO;

namespace RushHourSolver
{
    static class Program
    {
        static void Main(string[] args)
        {
            //Console.SetIn(File.OpenText("input.txt"));
            Worker worker = new Worker();
            worker.doWork();
            Console.WriteLine(worker._foundSolution/*+"solution"*/);
            //Console.SetIn(new StreamReader(Console.OpenStandardInput()));
            //Console.ReadLine();
        }

		class Worker
		{
			static int _targetVehicle, _goal;
			static int _numHorVec;
			static byte[] _vehicleLocs;
			static byte[] _vehicleLengths;
			static char[] _vehicleNames;
			static byte[][] _vehicleRows;
			static byte[][] _vehicleCols;
			static byte[] _vehicleStartPos;
			static Trie _visited;
			public Solution _foundSolution;
			static bool _solveMode;
            private Object lockSetSolution = new Object();
			
			public bool _solutionFound = false;
			private ConcurrentQueue<Tuple<byte[], Solution>> _q;

			static int _endDepth = int.MaxValue;
			public int _jobs = 0;

			public Worker()
			{
				ReadInput ();
				// Create a thread-safe queue that is accessible as a class member
				_q = new ConcurrentQueue<Tuple<byte[], Solution>> ();
			}

			//Logic that gets handles per thread
			public void ThreadPoolCallback (Object threadPoolContext)
			{
				Tuple<byte[], Solution> state = (Tuple<byte[], Solution>) threadPoolContext;
				foreach (Tuple<byte[], Solution> next in Sucessors (state))
				{
                    //Console.WriteLine(next.Item2.ToString());
					if (next.Item1[_targetVehicle] == _goal)
                    {
                        //Console.WriteLine(next.Item2.ToString()+"nylocked");
                        //lock zetten om de 2 regels hieronder, de bool is wel atomic, maar _foundSolution niet.
						_solutionFound = true;
                        lock (lockSetSolution)
                        {
                            //Console.WriteLine(next.Item2.Depth);
                            if (_endDepth > next.Item2.Depth)
                            {
                                _foundSolution = next.Item2;
                                //Console.WriteLine(_foundSolution.ToString()+"locked");
                                _endDepth = next.Item2.Depth;
                            }
                        }
						break;
					}
                    if (!AddNode(next.Item1))
                    {
                        _q.Enqueue(next);
                    }
				}
                Interlocked.Decrement(ref _jobs);
			}

			internal void doWork ()
			{
                _foundSolution = new NoSolution();
                _q.Enqueue(Tuple.Create(_vehicleStartPos, (Solution)new EmptySolution()));
                AddNode(_vehicleStartPos);

				// Do BFS
				while ((_q.Count > 0 || _jobs > 0) && !_solutionFound)
				{
                    while (_q.Count == 0 && _jobs!=0) { }
                    if (_q.Count != 0)
                    {
                        Interlocked.Increment(ref _jobs);
                        Tuple<byte[], Solution> currentState;
                        if (_q.TryDequeue(out currentState))
                        {
                            //Console.WriteLine(currentState.Item2.ToString());
                            ThreadPool.QueueUserWorkItem(ThreadPoolCallback, currentState);
                        }
                    }
				}
				// Alle threads klaar && queue is leeg
				// queue.dequeue en check of depth < current solution depth => in thread pool
				// als alle threads klaar en qqueue is leeg 
				while (_q.Count > 0 || _jobs > 0)
                {
                    while (_q.Count == 0 && _jobs != 0) { }
					// blijf de queue leeg werken, en alleen nieuwe jobs toevoegen als depth < currentSolutionDepth
                    if (_q.Count != 0)
                    {
                        Tuple<byte[], Solution> currentState;
                        if (_q.TryDequeue(out currentState))
                        {
                            if (currentState.Item2.Depth < _endDepth-1) //_endDepth needs to be atomic, because the check can happen here while it is being changed in a thread. It's an int, and according to documentation those are supposed to be atomic
                            {
                                Interlocked.Increment(ref _jobs);
                                ThreadPool.QueueUserWorkItem(ThreadPoolCallback, currentState);
                            }
                        }
                    }
				}
                if (_q.Count != 0 || _jobs != 0) { Console.WriteLine("Something wrong2"); }
			}

			private static IEnumerable<Tuple<byte[], Solution>> Sucessors (Tuple<byte[], Solution> state)
			{
				byte[] currentState = state.Item1;
				Solution solution = state.Item2;
				for (byte v = 0; v < _vehicleNames.Length; v++)
				{
					byte[] sameDir;
					byte[][] diffDir;
					int size;
					char dir;
					if (v < _numHorVec)
					{
						size = _vehicleCols.GetLength (0);
						sameDir = _vehicleRows[_vehicleLocs[v]];
						diffDir = _vehicleCols;
						dir = 'r';
					} else
					{
						size = _vehicleRows.GetLength (0);
						sameDir = _vehicleCols[_vehicleLocs[v]];
						diffDir = _vehicleRows;
						dir = 'd';
					}
					for (byte i = 1; i <= size - (currentState[v] + _vehicleLengths[v]); i++)
					{
						bool good = true;
						foreach (int j in sameDir) if (j != v && currentState[j] == currentState[v] + i + _vehicleLengths[v] - 1) good = false;
						foreach (int j in diffDir[currentState[v] + _vehicleLengths[v] + i - 1]) if (currentState[j] <= _vehicleLocs[v] && currentState[j] + _vehicleLengths[j] - 1 >= _vehicleLocs[v]) good = false;
						if (!good) break;
						byte[] newState = (byte[]) currentState.Clone ();
						newState[v] += i;
						if (AddNode (newState))
							yield return Tuple.Create (newState, solution.appendMove (v < _numHorVec, true, i, v));
					}
					if (dir == 'r')
						dir = 'l';
					else
						dir = 'u';
					for (byte i = 1; i <= currentState[v]; i++)
					{
						bool good = true;
						foreach (int j in sameDir) if (j != v && currentState[j] + _vehicleLengths[j] - 1 == currentState[v] - i) good = false;
						foreach (int j in diffDir[currentState[v] - i]) if (currentState[j] <= _vehicleLocs[v] && currentState[j] + _vehicleLengths[j] - 1 >= _vehicleLocs[v]) good = false;
						if (!good) break;
						byte[] newState = (byte[]) currentState.Clone ();
						newState[v] -= i;
						if (AddNode (newState))
							yield return Tuple.Create (newState, solution.appendMove (v < _numHorVec, false, i, v));
					}
				}
				yield break;
			}

			static void ReadInput ()
			{
				_solveMode = Console.ReadLine () == "1";
				string[] cf = Console.ReadLine ().Split ();
				int cols = int.Parse (cf[0]); int rows = int.Parse (cf[1]);
				cf = Console.ReadLine ().Split ();
				int targetX = int.Parse (cf[0]);
				int targetY = int.Parse (cf[1]);
				_vehicleNames = new char[0];
				_vehicleLengths = new byte[0];
				_vehicleStartPos = new byte[0];
				_vehicleLocs = new byte[0];
				_numHorVec = 0;
				byte vehicleId = 0;

				string[] field = new string[rows + 1];
				_vehicleRows = new byte[rows][];
				for (byte i = 0; i < rows; i++)
				{
					_vehicleRows[i] = new byte[0];
					field[i] = Console.ReadLine () + ".";
					byte runStart = 0; char runChar = '.';
					for (byte j = 0; j < field[i].Length; j++)
					{
						if (field[i][j] != runChar)
						{
							if (runChar != '.' && j - runStart > 1)
							{
								_vehicleNames = ArrayAdd (runChar, _vehicleNames);
								if (runChar == 'x') _targetVehicle = vehicleId;
								_vehicleLengths = ArrayAdd ((byte) (j - runStart), _vehicleLengths);
								_vehicleStartPos = ArrayAdd (runStart, _vehicleStartPos);
								_vehicleRows[i] = ArrayAdd (vehicleId, _vehicleRows[i]);
								_vehicleLocs = ArrayAdd (i, _vehicleLocs);
								vehicleId++;
								_numHorVec++;
							}
							runStart = j;
							runChar = field[i][j];
						}
					}
				}
				if (cols != field[0].Length - 1)
					throw new Exception ("This testcase is bad!");
				field[rows] = new String ('.', cols);
				_vehicleCols = new byte[cols][];
				for (byte i = 0; i < cols; i++)
				{
					_vehicleCols[i] = new byte[0];
					byte runStart = 0; char runChar = '.';
					for (byte j = 0; j < rows + 1; j++)
					{
						if (field[j][i] != runChar)
						{
							if (runChar != '.' && j - runStart > 1)
							{
								_vehicleNames = ArrayAdd (runChar, _vehicleNames);
								if (runChar == 'x') _targetVehicle = vehicleId;
								_vehicleLengths = ArrayAdd ((byte) (j - runStart), _vehicleLengths);
								_vehicleStartPos = ArrayAdd (runStart, _vehicleStartPos);
								_vehicleCols[i] = ArrayAdd (vehicleId, _vehicleCols[i]);
								_vehicleLocs = ArrayAdd (i, _vehicleLocs);
								vehicleId++;
							}
							runStart = j;
							runChar = field[j][i];
						}
					}
				}
				if (_numHorVec == 0)
					_visited = new Trie (rows - _vehicleLengths[0] + 1);
				else
					_visited = new Trie (cols - _vehicleLengths[0] + 1);

				if (_targetVehicle < _numHorVec)
				{
					_goal = targetX;
					if (_vehicleLocs[_targetVehicle] != targetY)
						throw new Exception ("This testcase is bad!");
				} else
				{
					_goal = targetY;
					if (_vehicleLocs[_targetVehicle] != targetX)
						throw new Exception ("This testcase is bad!");
				}
			}

			static T[] ArrayAdd<T> (T i, T[] array)
			{
				T[] newArray = new T[array.Length + 1];
				Array.Copy (array, newArray, array.Length);
				newArray[array.Length] = i;
				return newArray;
			}

			static bool AddNode (byte[] node)
			{
				Trie cur = _visited;
				for (byte i = 0; i < node.Length; i++)
				{
					if (cur.Leaves[node[i]] == null)
					{
						Trie newT;
						if (i == node.Length - 1)
							newT = new Trie (0);
						else if (i < _numHorVec)
							newT = new Trie (_vehicleCols.GetLength (0) - _vehicleLengths[i + 1] + 1);
						else
							newT = new Trie (_vehicleRows.GetLength (0) - _vehicleLengths[i + 1] + 1);
						if (cur.Leaves[node[i]] == null)
						{
							cur.Leaves[node[i]] = newT;
							if (i == node.Length - 1) return true;
							cur = newT;
						} else
							cur = cur.Leaves[node[i]];
					} else
						cur = cur.Leaves[node[i]];
				}
				return false;
			}

			internal bool getSolveMode ()
			{
				return _solveMode;
			}

			internal char[] _getVehicleNames ()
			{
				return _vehicleNames;
			}

			class Trie
			{
				public Trie[] Leaves { get; protected set; }

				public Trie (int s)
				{
					Leaves = new Trie[s];
				}
			}

			public class Solution
			{
				public Solution Parent { get; protected set; }
				public bool Direction { get; protected set; }
				public bool Forward { get; protected set; }
				public byte Amount { get; protected set; }
				public byte Vehicle { get; protected set; }
				public int Count { get; protected set; }
                public int Depth { get; protected set; }
                public Solution() { }
				public Solution appendMove (bool direction, bool forward, byte amount, byte vehicle)
				{
					if (!_solveMode)
						return new Solution () { Count = this.Count + 1, Depth = Depth + 1 };
					return new Solution ()
					{
						Parent = this,
						Direction = direction,
						Forward = forward,
						Amount = amount,
						Vehicle = vehicle,
                        Depth = Depth + 1
					};
				}

				public override string ToString ()
				{
					return MakeString ().Trim ();
				}

				protected virtual string MakeString ()
				{
					if (!_solveMode)
						return Count.ToString ();
					else
						return Parent.MakeString () + _vehicleNames[Vehicle] + (Direction ? (Forward ? 'r' : 'l') : (Forward ? 'd' : 'u')) + Amount.ToString () + " ";
				}
			}

			class EmptySolution : Solution
			{
                public EmptySolution ()
                {
                    Depth = 0;
                }
				protected override string MakeString ()
				{
					return "";
				}

				public override string ToString ()
				{
					return _solveMode ? "" : base.ToString ();
				}
			}

			class NoSolution : Solution
			{
				public override string ToString ()
				{
					return _solveMode ? "Geen oplossing gevonden" : "-1";
				}
			}
		}
    }
}
