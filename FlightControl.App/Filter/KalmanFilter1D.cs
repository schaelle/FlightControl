namespace FlightControl.App.Filter
{
	public class KalmanFilter1D
	{
		/* variables */
		private readonly double[] _k = new double[2]; // Kalman gain - This is a 2x1 matrix
		private readonly double[] _p = new double[4]; // Error covariance matrix - This is a 2x2 matrix
		// 0,0 = 0 0,1 = 1 1,0 = 2 1,1 =3
		private double _s; // Estimate error - 1x1 matrix
		private double _qBias; // Process noise variance for the gyro bias

		private double _angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
		private double _bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix

		private double _rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

		private double _y; // Angle difference - 1x1 matrix

		public double Q { get; set; } // Process noise variance for the accelerometer
		public double R { get; set; } // Measurement noise variance - this is actually the variance of the measurement noise

		public KalmanFilter1D()
		{
			/* We will set the varibles like so, these can also be tuned by the user */
			Reset();

			// Reset bias
			// Since we assume tha the bias is 0 and we know the starting angle (use setAngle),
			//the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
			SetDefaults();
		}

		// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
		public double GetAngle(double newAngle, double newRate, double dt)
		{
			// KasBot V2 - Kalman filter module - http://www.x-firm.com/?page_id=145
			// Modified by Kristian Lauszus
			// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

			// Discrete Kalman filter time update equations - Time Update ("Predict")
			// Update xhat - Project the state ahead
			/* Step 1 */
			_rate = newRate - _bias;
			_angle += dt*_rate;

			// Update estimation error covariance - Project the error covariance ahead
			/* Step 2 */
			_p[0] += dt * (dt * _p[3] - _p[1] - _p[2] + Q);
			_p[1] -= dt*_p[3];
			_p[2] -= dt*_p[3];
			_p[3] += _qBias*dt;

			// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
			// Calculate Kalman gain - Compute the Kalman gain
			/* Step 4 */
			_s = _p[0] + R;
			/* Step 5 */
			_k[0] = _p[0]/_s;
			_k[1] = _p[2]/_s;

			// Calculate angle and bias - Update estimate with measurement zk (newAngle)
			/* Step 3 */
			_y = newAngle - _angle;
			/* Step 6 */
			_angle += _k[0]*_y;
			_bias += _k[1]*_y;

			// Calculate estimation error covariance - Update the error covariance
			/* Step 7 */
			_p[0] -= _k[0]*_p[0];
			_p[1] -= _k[0]*_p[1];
			_p[2] -= _k[1]*_p[0];
			_p[3] -= _k[1]*_p[1];

			return _angle;
		}

		public void Reset()
		{
			_bias = 0; // Reset bias
			_p[0] = 0; // Since we assume tha the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
			_p[1] = 0;
			_p[2] = 0;
			_p[3] = 0;
		}

		public void SetDefaults()
		{
			/* We will set the varibles like so, these can also be tuned by the user */
			Q = 0.01;
			_qBias = 0.000;
			R = .002;
		}
	}
}