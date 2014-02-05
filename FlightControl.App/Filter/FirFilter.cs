namespace FlightControl.App.Filter
{
	public class FirFilter : IFilter
	{
		private readonly double[] _b;
		private double[] _buffer;
		private int _bufferPos = 0;
		private uint _pos = 0;

		public FirFilter(double[] b)
		{
			_b = b;
			_buffer = new double[b.Length];
		}

		public double Input(double input)
		{
			_buffer[_bufferPos] = input;
			var res = 0.0;
			for (var i = 0; i < System.Math.Min(_pos, _b.Length); i++)
			{
				res += _b[i] * _buffer[(_bufferPos - i + _buffer.Length) % _buffer.Length];
			}

			_bufferPos = ++_bufferPos % _buffer.Length;
			_pos++;

			return res;
		}

		public static double[] CalculateFilter(double fa, double fb, int m, double fs)
		{
			// http://www.arc.id.au/FilterDesign.html
			if (m % 2 == 0) m++;
			var res = new double[m + 1];
			res[0] = 2 * (fb - fa) / fs;
			for (var j = 1; j <= m; j++)
			{
				res[j] = (System.Math.Sin(2 * System.Math.PI * j * fb / fs) - System.Math.Sin(2 * System.Math.PI * j * fa / fs)) / (System.Math.PI * j);
			}
			return res;
		}
	}
}
