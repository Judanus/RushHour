﻿using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;

namespace RushHourSolver
{
    static class Program
    {
        static void Main(string[] args)
        {
			Worker worker = new Worker ();
			worker.doWork ();
			Console.WriteLine (worker._foundSolution);
            Console.ReadLine();
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
			
			public bool _solutionFound = false;
			private ConcurrentQueue<Tuple<byte[], Solution>> _q;

			static int _endDepth = 0;
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
					if (next.Item1[_targetVehicle] == _goal)
					{
						_solutionFound = true;
						_foundSolution = next.Item2;
						break;
					}
					if (!AddNode (next.Item1))
					{
						_q.Enqueue (state);
					}
				}
				_jobs--;
			}

			internal void doWork ()
			{
				// Do BFS
				while (_q.Count > 0 && !_solutionFound)
				{
					_jobs++;
					Tuple<byte[], Solution> currentState;
					if (_q.TryDequeue (out currentState))
					{
						ThreadPool.QueueUserWorkItem (ThreadPoolCallback, currentState);
					}
				}

				// Alle threads klaar && queue is leeg
				// queue.dequeue en check of depth < current solution depth => in thread pool
				// als alle threads klaar en qqueue is leeg 
				while (_q.Count > 0 || _jobs > 0)
				{
					// blijf de queue leeg werken, en alleen nieuwe jobs toevoegen als depth < currentSolutionDepth
					Tuple<byte[], Solution> currentState;
					if (_q.TryDequeue (out currentState))
					{
						if (currentState.Item2.Count < _endDepth)
						{
							_jobs++;
							ThreadPool.QueueUserWorkItem (ThreadPoolCallback, currentState);
						}
					}
				}
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

			class Solution
			{
				public Solution Parent { get; protected set; }
				public bool Direction { get; protected set; }
				public bool Forward { get; protected set; }
				public byte Amount { get; protected set; }
				public byte Vehicle { get; protected set; }
				public int Count { get; protected set; }
				public Solution () { }
				public Solution appendMove (bool direction, bool forward, byte amount, byte vehicle)
				{
					if (!_solveMode)
						return new Solution () { Count = this.Count + 1 };
					return new Solution ()
					{
						Parent = this,
						Direction = direction,
						Forward = forward,
						Amount = amount,
						Vehicle = vehicle
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
