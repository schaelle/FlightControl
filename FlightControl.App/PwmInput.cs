using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace FlightControl.App
{
	public class PwmInput
	{
		private static long _pwmInputTime;
		private static float[] _pwmInputData = new float[15];
		private static uint _pwmInputDataPos;
		private static long _pwmResetDuration = 30000;

		public PwmInput(Cpu.Pin pin)
		{
			var pwmInput = new InterruptPort(pin, false, Port.ResistorMode.Disabled, Port.InterruptMode.InterruptEdgeBoth);
			pwmInput.OnInterrupt += PwmInputOnOnInterrupt;
			//pwmInput.EnableInterrupt();
		}

		private static void PwmInputOnOnInterrupt(uint data1, uint data2, DateTime time)
		{
			var currentTicks = time.Ticks;
			var div = currentTicks - _pwmInputTime;

			if (div > _pwmResetDuration)
			{
				//Debug.Print(_pwmInputData[0] + "\t" + _pwmInputData[1] + "\t" + _pwmInputData[2] + "\t" + _pwmInputData[3] + "\t" + _pwmInputData[4] + "\t" + _pwmInputData[5] + "\t" + _pwmInputData[6]);
				_pwmInputDataPos = 0;
			}
			else
			{
				_pwmInputDataPos++;
			}

			if (_pwmInputData.Length > _pwmInputDataPos && _pwmInputDataPos > 0 && _pwmInputDataPos % 2 == 0)
			{
				_pwmInputData[_pwmInputDataPos / 2 - 1] = (float)(div - 10000) / 4082;
			}

			_pwmInputTime = currentTicks;
		}

		public float this[int index]
		{
			get { return _pwmInputData[index]; }
		}
	}
}
