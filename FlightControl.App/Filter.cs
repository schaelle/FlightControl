using System;
using Microsoft.SPOT;

namespace FlightControl.App
{
	public class Filter
	{
		private readonly double[] _b;
		private double[] _buffer;
		private int _bufferPos = 0;
		private uint _pos = 0;

		public Filter(double[] b)
		{
			_b = b;
			_buffer = new double[b.Length];
		}

		public double Input(double input)
		{
			_buffer[_bufferPos] = input;
			var res = 0.0;
			for (var i = 0; i < System.Math.Min(_pos,_b.Length); i++)
			{
				res += _b[i] * _buffer[(_bufferPos - i + _buffer.Length) % _buffer.Length];
			}

			_bufferPos = ++_bufferPos % _buffer.Length;
			_pos++;

			return res;
		}
	}
}
