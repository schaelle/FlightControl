using System;

namespace FlightControl.App
{
	public class Quaternion
	{
		public double W;
		public double X;
		public double Y;
		public double Z;

		public void Normalize()
		{
			var norm = System.Math.Sqrt(W * W + X * X + Y * Y + Z * Z);
			if (Math.Abs(norm) < 1E-12) return;
			W = W / norm;
			X = X / norm;
			Y = Y / norm;
			Z = Z / norm;
		}
	}
}