namespace FlightControl.App.Filter
{
	public class AverageFilter : IFilter
	{
		private double[] _buffer;
		private int _bufferPos = 0;
		private uint _pos = 0;

		public AverageFilter(int n)
		{
			_buffer = new double[n];
		}

		public double Input(double input)
		{
			_buffer[_bufferPos] = input;
			var res = 0.0;
			for (var i = 0; i < System.Math.Min(_pos, _buffer.Length); i++)
			{
				res += _buffer[(_bufferPos - i + _buffer.Length) % _buffer.Length];
			}

			_bufferPos = ++_bufferPos % _buffer.Length;
			_pos++;

			return res / System.Math.Min(_pos, _buffer.Length);
		}
	}
}