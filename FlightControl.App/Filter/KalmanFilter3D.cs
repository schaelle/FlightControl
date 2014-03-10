using System;
using FlightControl.App.Matrix;
using Microsoft.SPOT;

namespace FlightControl.App.Filter
{
	public class KalmanFilter3D
	{

		private DoubleVector _var;
		private DoubleVector _qUpdate;
		private DoubleVector _qOsserv;
		private DoubleMatrix _q;
		private DoubleMatrix _pUpdate;
		private DoubleMatrix _h;
		private DoubleMatrix _r;

		public KalmanFilter3D()
		{
			var var = .02;
			_var = new DoubleVector(new[] { System.Math.Pow(var / 180 * System.Math.PI, 2), System.Math.Pow(var / 180 * System.Math.PI, 2), System.Math.Pow(var / 180 * System.Math.PI, 2) });
			_qUpdate = new DoubleVector(4);
			_qUpdate[0] = 1;
			_qOsserv = new DoubleVector(4);
			_qOsserv[0] = 1;

			var sigmaRValue = 0.01;
			var sigmaR = new DoubleVector(new[] { sigmaRValue, sigmaRValue, sigmaRValue, sigmaRValue });
			_pUpdate = DoubleMatrix.Diagonal(4, .01);
			_h = DoubleMatrix.Diagonal(4, 1);
			_r = DoubleMatrix.Diagonal(4, sigmaR);

			var q1 = new DoubleVector(new[] {
				_var[0] + _var[1] - _var[2],
				-_var[0] + _var[1] + _var[2], 
				-_var[0] - _var[1] + _var[2], 
				_var[0] - _var[1] - _var[2]});
			var q2 = new DoubleVector(new[] {
				-_var[0] + _var[1] - _var[2],
				_var[0] + _var[1] + _var[2], 
				_var[0] - _var[1] - _var[2], 
				-_var[0] - _var[1] + _var[2]});
			var q3 = new DoubleVector(new[] {
				-_var[0] - _var[1] + _var[2],
				_var[0] - _var[1] - _var[2], 
				_var[0] + _var[1] + _var[2], 
				-_var[0] + _var[1] - _var[2]});
			var q4 = new DoubleVector(new[] {
				_var[0] - _var[1] - _var[2],
				-_var[0] + _var[1] - _var[2], 
				-_var[0] + _var[1] - _var[2], 
				_var[0] + _var[1] + _var[2]});

			_q = new DoubleMatrix(4, 4);
			_q.SetRow(0, q1);
			_q.SetRow(1, q2);
			_q.SetRow(2, q3);
			_q.SetRow(3, q4);
		}


		public DoubleVector Update(double dt, DoubleVector gyro, DoubleVector qObserved)
		{
			qObserved = qObserved.Normalize2();

			var f1 = new[] { 1, -dt / 2 * gyro[0], -dt / 2 * gyro[1], -dt / 2 * gyro[2] };
			var f2 = new[] { dt / 2 * gyro[0], 1, dt / 2 * gyro[2], -dt / 2 * gyro[1] };
			var f3 = new[] { dt / 2 * gyro[1], -dt / 2 * gyro[2], 1, dt / 2 * gyro[0] };
			var f4 = new[] { -dt / 2 * gyro[2], dt / 2 * gyro[1], -dt / 2 * gyro[0], 1 };
			var f = new DoubleMatrix(4, 4);
			f.SetRow(0, f1);
			f.SetRow(1, f2);
			f.SetRow(2, f3);
			f.SetRow(3, f4);

			var qPredicted = f * _qUpdate;
			var pPredicted = f * _pUpdate * f.Transpose() + _q;
			var k = pPredicted * _h.Transpose() * (_h * pPredicted * _h.Transpose() + _r).Inverse4();
			_qUpdate = (qPredicted + k * (qObserved - _h * qPredicted)).Normalize2();
			_pUpdate = (DoubleMatrix.Diagonal(4, 1) - k * _h) * pPredicted;

			return _qUpdate;
		}
	}
}
