using System;

namespace FlightControl.App.Matrix
{
	public class DoubleMatrix
	{
		private readonly int _n;
		private readonly int _m;
		private readonly double[] _data;

		public DoubleMatrix(int m, int n)
		{
			_n = n;
			_m = m;
			_data = new double[n * m];
		}

		public double this[int i, int j]
		{
			get { return _data[i + j * _m]; }
			set { _data[i + j * _m] = value; }
		}

		public static DoubleMatrix Diagonal(int n, double value)
		{
			var res = new DoubleMatrix(n, n);
			for (var i = 0; i < n; i++)
			{
				res[i, i] = value;
			}
			return res;
		}

		public static DoubleMatrix Diagonal(int n, DoubleVector data)
		{
			var res = new DoubleMatrix(n, n);
			for (var i = 0; i < n; i++)
			{
				res[i, i] = data[i];
			}
			return res;
		}

		public void SetRow(int i, DoubleVector row)
		{
			for (var j = 0; j < row.Count; j++)
			{
				this[i, j] = row[j];
			}
		}

		public void SetRow(int i, double[] row)
		{
			for (var j = 0; j < row.Length; j++)
			{
				this[i, j] = row[j];
			}
		}

		public DoubleMatrix Transpose()
		{
			var res = new DoubleMatrix(_n, _m);
			for (var i = 0; i < _m; i++)
			{
				for (var j = 0; j < _n; j++)
				{
					res[j, i] = this[i, j];
				}
			}
			return res;
		}

		public static DoubleVector operator *(DoubleMatrix a, DoubleVector b)
		{
			if (a._n != b.Count)
				throw new Exception("Dimension mismatch");

			var res = new DoubleVector(a._m);
			for (var i = 0; i < a._m; i++)
			{
				var sum = 0.0;
				for (var j = 0; j < a._n; j++)
				{
					sum += a[i, j] * b[j];
				}
				res[i] = sum;
			}
			return res;
		}

		public static DoubleMatrix operator *(DoubleMatrix a, DoubleMatrix b)
		{
			if (a._n != b._m)
				throw new Exception("Dimension mismatch");

			var res = new DoubleMatrix(a._m, b._n);
			for (var i = 0; i < a._m; i++)
			{
				for (var j = 0; j < a._n; j++)
				{
					var sum = 0.0;
					for (var k = 0; k < a._n; k++)
					{
						sum += a[i, k] * b[k, j];
					}
					res[i, j] = sum;
				}

			}
			return res;
		}

		public static DoubleMatrix operator +(DoubleMatrix a, DoubleMatrix b)
		{
			if (a._m != b._m && a._n != b._n)
				throw new Exception("Dimension mismatch");

			var res = new DoubleMatrix(a._m, b._n);
			for (var i = 0; i < a._m; i++)
			{
				for (var j = 0; j < a._n; j++)
				{
					res[i, j] = a[i, j] + b[i, j];
				}

			}
			return res;
		}

		public static DoubleMatrix operator -(DoubleMatrix a, DoubleMatrix b)
		{
			if (a._m != b._m && a._n != b._n)
				throw new Exception("Dimension mismatch");

			var res = new DoubleMatrix(a._m, b._n);
			for (var i = 0; i < a._m; i++)
			{
				for (var j = 0; j < a._n; j++)
				{
					res[i, j] = a[i, j] - b[i, j];
				}

			}
			return res;
		}

		private double[] Row(int i)
		{
			var res = new double[_n];
			for (int j = 0; j < _n; j++)
			{
				res[j] = this[i, j];
			}
			return res;
		}

		private DoubleMatrix Duplicate()
		{
			var res = new DoubleMatrix(_m, _n);
			for (var i = 0; i < _m; i++)
			{
				for (var j = 0; j < _n; j++)
				{
					res[i, j] = this[i, j];
				}
			}
			return res;
		}

		public DoubleMatrix Inverse4()
		{
			var inv = new DoubleMatrix(4, 4);

			inv[0, 0] = this[1, 1] * this[2, 2] * this[3, 3] -
					 this[1, 1] * this[2, 3] * this[3, 2] -
					 this[2, 1] * this[1, 2] * this[3, 3] +
					 this[2, 1] * this[1, 3] * this[3, 2] +
					 this[3, 1] * this[1, 2] * this[2, 3] -
					 this[3, 1] * this[1, 3] * this[2, 2];

			inv[1, 0] = -this[1, 0] * this[2, 2] * this[3, 3] +
					  this[1, 0] * this[2, 3] * this[3, 2] +
					  this[2, 0] * this[1, 2] * this[3, 3] -
					  this[2, 0] * this[1, 3] * this[3, 2] -
					  this[3, 0] * this[1, 2] * this[2, 3] +
					  this[3, 0] * this[1, 3] * this[2, 2];

			inv[2, 0] = this[1, 0] * this[2, 1] * this[3, 3] -
					 this[1, 0] * this[2, 3] * this[3, 1] -
					 this[2, 0] * this[1, 1] * this[3, 3] +
					 this[2, 0] * this[1, 3] * this[3, 1] +
					 this[3, 0] * this[1, 1] * this[2, 3] -
					 this[3, 0] * this[1, 3] * this[2, 1];

			inv[3, 0] = -this[1, 0] * this[2, 1] * this[3, 2] +
					   this[1, 0] * this[2, 2] * this[3, 1] +
					   this[2, 0] * this[1, 1] * this[3, 2] -
					   this[2, 0] * this[1, 2] * this[3, 1] -
					   this[3, 0] * this[1, 1] * this[2, 2] +
					   this[3, 0] * this[1, 2] * this[2, 1];

			inv[1, 0] = -this[1, 0] * this[2, 2] * this[3, 3] +
					  this[1, 0] * this[2, 3] * this[3, 2] +
					  this[2, 1] * this[2, 0] * this[3, 3] -
					  this[2, 1] * this[3, 0] * this[3, 2] -
					  this[3, 1] * this[2, 0] * this[2, 3] +
					  this[3, 1] * this[3, 0] * this[2, 2];

			inv[1, 1] = this[0, 0] * this[2, 2] * this[3, 3] -
					 this[0, 0] * this[2, 3] * this[3, 2] -
					 this[2, 0] * this[2, 0] * this[3, 3] +
					 this[2, 0] * this[3, 0] * this[3, 2] +
					 this[3, 0] * this[2, 0] * this[2, 3] -
					 this[3, 0] * this[3, 0] * this[2, 2];

			inv[2, 1] = -this[0, 0] * this[2, 1] * this[3, 3] +
					  this[0, 0] * this[2, 3] * this[3, 1] +
					  this[2, 0] * this[1, 0] * this[3, 3] -
					  this[2, 0] * this[3, 0] * this[3, 1] -
					  this[3, 0] * this[1, 0] * this[2, 3] +
					  this[3, 0] * this[3, 0] * this[2, 1];

			inv[3, 1] = this[0, 0] * this[2, 1] * this[3, 2] -
					  this[0, 0] * this[2, 2] * this[3, 1] -
					  this[2, 0] * this[1, 0] * this[3, 2] +
					  this[2, 0] * this[2, 0] * this[3, 1] +
					  this[3, 0] * this[1, 0] * this[2, 2] -
					  this[3, 0] * this[2, 0] * this[2, 1];

			inv[2, 0] = this[1, 0] * this[1, 2] * this[3, 3] -
					 this[1, 0] * this[1, 3] * this[3, 2] -
					 this[1, 1] * this[2, 0] * this[3, 3] +
					 this[1, 1] * this[3, 0] * this[3, 2] +
					 this[3, 1] * this[2, 0] * this[1, 3] -
					 this[3, 1] * this[3, 0] * this[1, 2];

			inv[1, 2] = -this[0, 0] * this[1, 2] * this[3, 3] +
					  this[0, 0] * this[1, 3] * this[3, 2] +
					  this[1, 0] * this[2, 0] * this[3, 3] -
					  this[1, 0] * this[3, 0] * this[3, 2] -
					  this[3, 0] * this[2, 0] * this[1, 3] +
					  this[3, 0] * this[3, 0] * this[1, 2];

			inv[2, 2] = this[0, 0] * this[1, 1] * this[3, 3] -
					  this[0, 0] * this[1, 3] * this[3, 1] -
					  this[1, 0] * this[1, 0] * this[3, 3] +
					  this[1, 0] * this[3, 0] * this[3, 1] +
					  this[3, 0] * this[1, 0] * this[1, 3] -
					  this[3, 0] * this[3, 0] * this[1, 1];

			inv[3, 2] = -this[0, 0] * this[1, 1] * this[3, 2] +
					   this[0, 0] * this[1, 2] * this[3, 1] +
					   this[1, 0] * this[1, 0] * this[3, 2] -
					   this[1, 0] * this[2, 0] * this[3, 1] -
					   this[3, 0] * this[1, 0] * this[1, 2] +
					   this[3, 0] * this[2, 0] * this[1, 1];

			inv[3, 0] = -this[1, 0] * this[1, 2] * this[2, 3] +
					  this[1, 0] * this[1, 3] * this[2, 2] +
					  this[1, 1] * this[2, 0] * this[2, 3] -
					  this[1, 1] * this[3, 0] * this[2, 2] -
					  this[2, 1] * this[2, 0] * this[1, 3] +
					  this[2, 1] * this[3, 0] * this[1, 2];

			inv[1, 3] = this[0, 0] * this[1, 2] * this[2, 3] -
					 this[0, 0] * this[1, 3] * this[2, 2] -
					 this[1, 0] * this[2, 0] * this[2, 3] +
					 this[1, 0] * this[3, 0] * this[2, 2] +
					 this[2, 0] * this[2, 0] * this[1, 3] -
					 this[2, 0] * this[3, 0] * this[1, 2];

			inv[2, 3] = -this[0, 0] * this[1, 1] * this[2, 3] +
					   this[0, 0] * this[1, 3] * this[2, 1] +
					   this[1, 0] * this[1, 0] * this[2, 3] -
					   this[1, 0] * this[3, 0] * this[2, 1] -
					   this[2, 0] * this[1, 0] * this[1, 3] +
					   this[2, 0] * this[3, 0] * this[1, 1];

			inv[3, 3] = this[0, 0] * this[1, 1] * this[2, 2] -
					  this[0, 0] * this[1, 2] * this[2, 1] -
					  this[1, 0] * this[1, 0] * this[2, 2] +
					  this[1, 0] * this[2, 0] * this[2, 1] +
					  this[2, 0] * this[1, 0] * this[1, 2] -
					  this[2, 0] * this[2, 0] * this[1, 1];

			var det = this[0, 0] * inv[0, 0] + this[1, 0] * inv[1, 0] + this[2, 0] * inv[2, 0] + this[3, 0] * inv[3, 0];
			det = 1.0 / det;

			for (var i = 0; i < this._m; i++)
			{
				for (var j = 0; j < this._n; j++)
				{
					inv[i, j] *= det;
				}
			}
			return inv;
		}
	}

	public class DoubleVector
	{
		private readonly int _n;
		private readonly double[] _data;

		public DoubleVector(int n)
		{
			_n = n;
			_data = new double[n];
		}

		public DoubleVector(double[] data)
		{
			_n = data.Length;
			_data = data;
		}

		public double this[int i]
		{
			get { return _data[i]; }
			set { _data[i] = value; }
		}

		public int Count { get { return _n; } }

		public static DoubleVector operator -(DoubleVector a, DoubleVector b)
		{
			if (a.Count != b.Count)
				throw new Exception("Dimension mismatch");

			var res = new DoubleVector(a.Count);
			for (var i = 0; i < a.Count; i++)
			{
				res[i] = a[i] - b[i];
			}
			return res;
		}

		public static DoubleVector operator +(DoubleVector a, DoubleVector b)
		{
			if (a.Count != b.Count)
				throw new Exception("Dimension mismatch");

			var res = new DoubleVector(a.Count);
			for (var i = 0; i < a.Count; i++)
			{
				res[i] = a[i] + b[i];
			}
			return res;
		}

		public DoubleVector Normalize2()
		{
			var norm = 0.0;
			for (var i = 0; i < Count; i++)
			{
				norm += this[i] * this[i];
			}
			norm = System.Math.Sqrt(norm);
			var res = new DoubleVector(Count);
			for (var i = 0; i < Count; i++)
			{
				res[i] = this[i] / norm;
			}
			return res;
		}
	}
}
