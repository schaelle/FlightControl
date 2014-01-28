using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Generic;

namespace FlightControl.Imu.Android
{
	public class AndroidImuSensor : IImuSensor
	{
		private readonly Socket _socket;
		private readonly byte[] _buffer = new byte[65000];
		private EndPoint _endpoint = new IPEndPoint(IPAddress.Any, 0);
		private StreamWriter _stream;
		private readonly KalmanFilter _filter;
		public event OnMeasurementUpdatedHandler OnMeasurementUpdated;
		private double _lastTime = -1;
		private Vector<double> _gyroOld;

		public AndroidImuSensor(int port)
		{
			_filter = new KalmanFilter();

			_socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
			_socket.Bind(new IPEndPoint(IPAddress.Any, 5555));

			_socket.BeginReceiveMessageFrom(_buffer, 0, _buffer.Length, SocketFlags.None, ref _endpoint, ReceiveMessage, null);
			_stream = new StreamWriter("c:/imu.txt", false);
		}

		private void ReceiveMessage(IAsyncResult ar)
		{
			var flags = SocketFlags.None;
			IPPacketInformation information;
			var count = _socket.EndReceiveMessageFrom(ar, ref flags, ref _endpoint, out information);

			var data = Encoding.Default.GetString(_buffer, 0, count);
			var doubleData = data.Split(new[] { "," }, StringSplitOptions.None).Select(double.Parse).ToArray();
			if (doubleData.Length >= 10 && _lastTime != -1)
			{
				var acc = Convert(doubleData, 2, 3);
				var gyro = Convert(doubleData, 6, 3);
				var mag = Convert(doubleData, 10, 3);
				if (_gyroOld != null)
				{
					var gyroFilter = (_gyroOld + gyro) / 2;
					var filterData = _filter.Update(doubleData[0] - _lastTime, acc, gyroFilter, mag).ToArray();
					_stream.WriteLine(string.Join("\t", filterData));
					_stream.Flush();
				}
				_gyroOld = gyro;
			}
			_lastTime = doubleData[0];

			_socket.BeginReceiveMessageFrom(_buffer, 0, _buffer.Length, SocketFlags.None, ref _endpoint, ReceiveMessage, null);
		}

		private static Vector<double> Convert(double[] data, int index, int len)
		{
			var tmp = new double[len];
			Array.Copy(data, index, tmp, 0, len);
			return new DenseVector(tmp);
		}
	}

	public static class QuaternionExtension
	{
		public static Vector<double> ToQuaternion(this Vector<double> vector)
		{
			var subVector = new DenseVector(4);
			subVector.SetSubVector(1, 3, vector);
			return subVector;
		}

		public static Vector<double> ToAngles(this Vector<double> q)
		{
			var a1 = Math.Atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 180 / Math.PI;
			var a2 = Math.Asin(2 * (q[0] * q[2] + q[3] * q[1])) * 180 / Math.PI;
			var a3 = Math.Atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * 180 / Math.PI;
			return new DenseVector(new[] { a1, a2, a3 });
		}

		public static Matrix<double> RotationMatrix(this Vector<double> a)
		{
			var tmp = a / 180 * Math.PI;

			var res = new DenseMatrix(3, 3);
			res[0, 0] = Math.Cos(tmp[1]) * Math.Cos(tmp[2]);
			res[0, 1] = Math.Cos(tmp[0]) * Math.Sin(tmp[2]) + Math.Sin(tmp[0]) * Math.Sin(tmp[1]) * Math.Cos(tmp[2]);
			res[0, 2] = Math.Sin(tmp[0]) * Math.Sin(tmp[2]) - Math.Cos(tmp[0]) * Math.Sin(tmp[1]) * Math.Cos(tmp[2]);
			res[1, 0] = -Math.Cos(tmp[1]) * Math.Sin(tmp[2]);
			res[1, 1] = Math.Cos(tmp[0]) * Math.Cos(tmp[2]) - Math.Sin(tmp[0]) * Math.Sin(tmp[1]) * Math.Sin(tmp[2]);
			res[1, 2] = Math.Sin(tmp[0]) * Math.Cos(tmp[2]) + Math.Cos(tmp[0]) * Math.Sin(tmp[1]) * Math.Sin(tmp[2]);
			res[2, 0] = Math.Sin(tmp[1]);
			res[2, 1] = -Math.Sin(tmp[0]) * Math.Cos(tmp[1]);
			res[2, 2] = Math.Cos(tmp[0]) * Math.Cos(tmp[1]);
			return res;
		}
		public static Matrix<double> RotationQuaternion(this Vector<double> q)
		{
			var a = q[1];
			var b = q[2];
			var c = q[3];
			var d = q[0];

			var r = new DenseMatrix(3, 3);
			r[0, 0] = d * d + a * a - b * b - c * c;
			r[0, 1] = 2 * (a * b - c * d);
			r[0, 2] = 2 * (a * c + b * d);
			r[1, 0] = 2 * (a * b + c * d);
			r[1, 1] = d * d + b * b - a * a - c * c;
			r[1, 2] = 2 * (b * c - a * d);
			r[2, 0] = 2 * (a * c - b * d);
			r[2, 1] = 2 * (b * c + a * d);
			r[2, 2] = d * d + c * c - b * b - a * a;

			return r;
		}
	}

	public class FilterButterworth
	{
		private readonly double _a1, _a2, _a3, _b1, _b2;

		/// <summary>
		/// Array of input values, latest are in front
		/// </summary>
		private readonly double[] _inputHistory = new double[2];

		/// <summary>
		/// Array of output values, latest are in front
		/// </summary>
		private readonly double[] _outputHistory = new double[3];

		public FilterButterworth(double frequency, PassType passType, double resonance)
		{
			switch (passType)
			{
				case PassType.Lowpass:
					var c = 1.0 / Math.Tan(frequency);
					_a1 = 1.0 / (1.0 + resonance * c + c * c);
					_a2 = 2 * _a1;
					_a3 = _a1;
					_b1 = 2.0 * (1.0 - c * c) * _a1;
					_b2 = (1.0 - resonance * c + c * c) * _a1;
					break;
				case PassType.Highpass:
					c = Math.Tan(frequency);
					_a1 = 1.0 / (1.0f + resonance * c + c * c);
					_a2 = -2f * _a1;
					_a3 = _a1;
					_b1 = 2.0 * (c * c - 1.0) * _a1;
					_b2 = (1.0 - resonance * c + c * c) * _a1;
					break;
			}
		}

		public enum PassType
		{
			Highpass,
			Lowpass,
		}

		public void Update(double newInput)
		{
			var newOutput = _a1 * newInput + _a2 * this._inputHistory[0] + _a3 * this._inputHistory[1] - _b1 * this._outputHistory[0] - _b2 * this._outputHistory[1];

			_inputHistory[1] = _inputHistory[0];
			_inputHistory[0] = newInput;

			_outputHistory[2] = _outputHistory[1];
			_outputHistory[1] = _outputHistory[0];
			_outputHistory[0] = newOutput;
		}

		public double Value
		{
			get { return _outputHistory[0]; }
		}
	}
}