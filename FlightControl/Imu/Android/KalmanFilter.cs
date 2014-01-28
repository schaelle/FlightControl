using System;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Generic;

namespace FlightControl.Imu.Android
{
	public class KalmanFilter
	{
		private DenseVector _var;
		private Vector<double> _qUpdate;
		private Vector<double> _qOsserv;
		private DenseMatrix _q;
		private Matrix<double> _pUpdate;
		private DiagonalMatrix _h;
		private DiagonalMatrix _r;
		private FilterButterworth[] _accFilter = new FilterButterworth[3];
		private FilterButterworth[] _magFilter = new FilterButterworth[3];

		public KalmanFilter()
		{
			for (int i = 0; i < 3; i++)
			{
				_accFilter[i] = new FilterButterworth(0.00075, FilterButterworth.PassType.Lowpass, 0);
				_magFilter[i] = new FilterButterworth(0.06, FilterButterworth.PassType.Lowpass, 0.06 * 2);
			}

			var var = Math.Sqrt(.0003);
			_var = new DenseVector(new[] { Math.Pow(var / 180 * Math.PI, 2), Math.Pow(var / 180 * Math.PI, 2), Math.Pow(var / 180 * Math.PI, 2) });
			_qUpdate = new DenseVector(4);
			_qUpdate[0] = 1;
			_qOsserv = new DenseVector(4);
			_qOsserv[0] = 1;

			var sigmaR = new DenseVector(new[] { 0.01, 0.01, 0.01, 0.01 });
			_pUpdate = new DiagonalMatrix(4, 4, .01);
			_h = new DiagonalMatrix(4, 4, 1);
			_r = new DiagonalMatrix(4, 4, sigmaR.ToArray());

			var q1 = new DenseVector(new[] {
				_var[0] + _var[1] - _var[2],
				-_var[0] + _var[1] + _var[2], 
				-_var[0] - _var[1] + _var[2], 
				_var[0] - _var[1] - _var[2]});
			var q2 = new DenseVector(new[] {
				-_var[0] + _var[1] - _var[2],
				_var[0] + _var[1] + _var[2], 
				_var[0] - _var[1] - _var[2], 
				-_var[0] - _var[1] + _var[2]});
			var q3 = new DenseVector(new[] {
				-_var[0] - _var[1] + _var[2],
				_var[0] - _var[1] - _var[2], 
				_var[0] + _var[1] + _var[2], 
				-_var[0] + _var[1] - _var[2]});
			var q4 = new DenseVector(new[] {
				_var[0] - _var[1] - _var[2],
				-_var[0] + _var[1] - _var[2], 
				-_var[0] + _var[1] - _var[2], 
				_var[0] + _var[1] + _var[2]});

			_q = new DenseMatrix(4, 4);
			_q.SetRow(0, q1);
			_q.SetRow(1, q2);
			_q.SetRow(2, q3);
			_q.SetRow(3, q4);
		}


		public Vector<double> Update(double dt, Vector<double> acc, Vector<double> gyro, Vector<double> mag)
		{
			mag = mag.Normalize(2);
			gyro = gyro / 180 * Math.PI;

			for (int i = 0; i < 3; i++)
			{
				_accFilter[i].Update(acc[i]);
				//acc[i] = _accFilter[i].Value;
				_magFilter[i].Update(mag[i]);
				//mag[i] = _magFilter[i].Value;
			}

			mag = mag.Normalize(2);

			_qOsserv = GaussNewton(acc.Normalize(2), mag, _qUpdate);

			var f1 = new DenseVector(new[] { 1, -dt / 2 * gyro[0], -dt / 2 * gyro[1], -dt / 2 * gyro[2] });
			var f2 = new DenseVector(new[] { dt / 2 * gyro[0], 1, dt / 2 * gyro[2], -dt / 2 * gyro[1] });
			var f3 = new DenseVector(new[] { dt / 2 * gyro[1], -dt / 2 * gyro[2], 1, dt / 2 * gyro[0] });
			var f4 = new DenseVector(new[] { -dt / 2 * gyro[2], dt / 2 * gyro[1], -dt / 2 * gyro[0], 1 });
			var f = new DenseMatrix(4, 4);
			f.SetRow(0, f1);
			f.SetRow(1, f2);
			f.SetRow(2, f3);
			f.SetRow(3, f4);

			var qPredict = f * _qUpdate;
			var pPredicted = f * _pUpdate * f.Transpose() + _q;
			var k = pPredicted * _h.Transpose() * (_h * pPredicted * _h.Transpose() + _r).Inverse();
			_qUpdate = (qPredict + k * (_qOsserv - _h * qPredict)).Normalize(2);
			_pUpdate = (new DiagonalMatrix(4, 4, 1) - k * _h) * pPredicted;

			var angles = _qUpdate.ToAngles();
			//var angles = _qOsserv.ToAngles();
			var rot = _qUpdate.RotationQuaternion();
			var accRot = rot * acc;
			Console.WriteLine(accRot);
			return accRot;
		}

		private Vector<double> GaussNewton(Vector<double> acc, Vector<double> mag, Vector<double> q)
		{
			var a = q[1];
			var b = q[2];
			var c = q[3];
			var d = q[0];

			Vector<double> nk = new DenseVector(new[] { a, b, c, d });
			double rate;
			do
			{
				var m = mag.Normalize(2);
				var qConj = new DenseVector(new[] { q[0], -q[1], -q[2], -q[3] });
				var hTemp = QuaternionProduct(q, m.ToQuaternion());
				var h = QuaternionProduct(hTemp, qConj);
				var bMag = new DenseVector(new[] { Math.Sqrt(h[1] * h[1] + h[2] * h[2]), 0, h[3] }).Normalize(2);
				var jnk = ComputeJacobian(a, b, c, d, acc[0], acc[1], acc[2], mag[0], mag[1], mag[2]);
				var M = ComputeMMatrix(a, b, c, d);

				var ye = new DenseVector(6);
				ye[2] = 1;
				ye.SetSubVector(3, 3, bMag);
				var yb = new DenseVector(6);
				yb.SetSubVector(0, 3, acc);
				yb.SetSubVector(3, 3, mag);

				//Gauss-Newton step
				var n = (nk - ((jnk.Transpose() * jnk).Inverse() * jnk.Transpose() * (ye - M * yb))).Normalize(2);
				a = n[0];
				b = n[1];
				c = n[2];
				d = n[3];
				nk = n;
				var qNew = new DenseVector(new[] { d, a, b, c });
				rate = (q - qNew).Norm(2);
				q = qNew;
			} while (rate > 1E-12);

			return q;
		}

		private Matrix ComputeMMatrix(double a, double b, double c, double d)
		{
			//Compute the rotation transformation matrix based on quaternions
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

			var m = new DenseMatrix(6, 6);
			m.SetSubMatrix(0, 3, 0, 3, r);
			m.SetSubMatrix(3, 3, 3, 3, r);
			return m;
		}

		private Matrix ComputeJacobian(double a, double b, double c, double d, double Ax, double Ay, double Az, double Mx, double My, double Mz)
		{
			var res = new DenseMatrix(6, 4);

			res[0, 0] = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
			res[0, 1] = (-2 * b * Ax + 2 * a * Ay + 2 * d * Az);
			res[0, 2] = (-2 * c * Ax - 2 * d * Ay + 2 * a * Az);
			res[0, 3] = (2 * d * Ax - 2 * c * Ay + 2 * b * Az);

			res[1, 0] = (2 * b * Ax - 2 * a * Ay - 2 * d * Az);
			res[1, 1] = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
			res[1, 2] = (2 * d * Ax - 2 * c * Ay + 2 * b * Az);
			res[1, 3] = (2 * c * Ax + 2 * d * Ay - 2 * a * Az);

			res[2, 0] = (2 * c * Ax + 2 * d * Ay - 2 * a * Az);
			res[2, 1] = (-2 * d * Ax + 2 * c * Ay - 2 * b * Az);
			res[2, 2] = (2 * a * Ax + 2 * b * Ay + 2 * c * Az);
			res[2, 3] = (-2 * b * Ax + 2 * a * Ay + 2 * d * Az);

			res[3, 0] = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
			res[3, 1] = (-2 * b * Mx + 2 * a * My + 2 * Mz * d);
			res[3, 2] = (-2 * c * Mx - 2 * d * My + 2 * a * Mz);
			res[3, 3] = (2 * d * Mx - 2 * c * My + 2 * b * Mz);

			res[4, 0] = (2 * b * Mx - 2 * a * My - 2 * d * Mz);
			res[4, 1] = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
			res[4, 2] = (2 * d * Mx - 2 * c * My + 2 * b * Mz);
			res[4, 3] = (2 * c * Mx + 2 * d * My - 2 * a * Mz);

			res[5, 0] = (2 * c * Mx + 2 * d * My - 2 * a * Mz);
			res[5, 1] = (-2 * d * Mx + 2 * c * My - 2 * b * Mz);
			res[5, 2] = (2 * a * Mx + 2 * b * My + 2 * c * Mz);
			res[5, 3] = (-2 * b * Mx + 2 * a * My + 2 * d * Mz);

			return -res;
		}

		private static Vector<double> QuaternionProduct(Vector<double> a, Vector<double> b)
		{
			var p1 = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
			var p2 = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
			var p3 = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
			var p4 = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
			return new DenseVector(new[] { p1, p2, p3, p4 });
		}
	}
}