using System;

namespace FlightControl.App
{
	public class PidController
	{
		//private long _lastTime;
		private double _iTerm;
		private double _lastInput;
		public double Kp { get; set; }
		public double Ki { get; set; }
		public double Kd { get; set; }

		public double OutputMinimum { get; set; }
		public double OutputMaximum { get; set; }

		public double SetPoint { get; set; }

		public PidController()
		{
			OutputMinimum = 0;
			OutputMaximum = 1;
		}

		public double Compute(double input, double angularVelocity)
		{
			//var now = DateTime.Now.Ticks;
			//var timeDelta = now - _lastTime;

			var error = SetPoint - input;
			_iTerm += (Ki * error);

			if (_iTerm < OutputMinimum) _iTerm = OutputMinimum;
			if (_iTerm > OutputMaximum) _iTerm = OutputMaximum;

			//var dError = _lastInput - input;
			var dError = angularVelocity;

			var res = Kp * error + _iTerm + Kd * dError;
			if (res < OutputMinimum) res = OutputMinimum;
			if (res > OutputMaximum) res = OutputMaximum;

			//_lastTime = now;
			_lastInput = input;

			return res;
		}
	}
}