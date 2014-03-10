using System;
using System.IO.Ports;
using System.Text;
using System.Threading;
using FlightControl.App.Filter;
using FlightControl.App.Matrix;
using Gadgeteer.Modules.GHIElectronics;
using GHI.Hardware.G400;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace FlightControl.App
{
	public class Program
	{
		private static double _lastAngle;
		private static I2CDevice _imu;
		private static PWM _pwm1;
		private static PWM _pwm2;
		private static PidController _pid;
		private static int _pos;
		private static IFilter _lowPassFilter;
		private static KalmanFilter3D _kalmanFilter = new KalmanFilter3D();

		private const int PwmPeriode = 20000;

		private static SerialPort _gps;
		private static PwmInput _pwmInput;
		private static I2CDevice.Configuration _motorConfig;
		private static Mpu6050Sensor _mpu;

		private static void GpsOnDataReceived(object sender, SerialDataReceivedEventArgs serialDataReceivedEventArgs)
		{
			var buffer = new byte[_gps.BytesToRead];
			_gps.Read(buffer, 0, buffer.Length);
			Debug.Print(buffer.Length.ToString());
		}

		public static void Main()
		{
			/*_gps = new SerialPort("COM3", 9600);
			_gps.DataReceived += GpsOnDataReceived;
			_gps.ReadTimeout = Timeout.Infinite;
			_gps.Open();

			Thread.Sleep(int.MaxValue);*/

			_pwmInput = new PwmInput(Pin.PB14);

			_pwm1 = new PWM(Cpu.PWMChannel.PWM_2, PwmPeriode, PwmPeriode / 20, PWM.ScaleFactor.Microseconds, false);
			_pwm1.Start();
			_pwm2 = new PWM(Cpu.PWMChannel.PWM_3, PwmPeriode, PwmPeriode / 20, PWM.ScaleFactor.Microseconds, false);
			_pwm2.Start();

			const double pKrit = -.006;
			const double tKrit = 2.1;
			var kp = .45 * pKrit;
			var tv = .12 * tKrit;
			var tn = .85 * tKrit;

			// working i=-.00005 p=-.0012 d=.002
			_pid = new PidController()
			{
				//Ki = -.0001,
				Kp = -.002,
				Kd = .002,
				OutputMinimum = -1,
				OutputMaximum = 1
			};

			//http://www.arc.id.au/FilterDesign.html
			//_lowPassFilter = new FilterButterworth(40, 100, FilterButterworth.PassType.Lowpass, 40);
			_lowPassFilter = new FirFilter(FirFilter.CalculateFilter(0, 20, 40, 100));

			/*var uart = new SerialPort("COM4", 38400, Parity.None, 8, StopBits.One);
			uart.DataReceived += uart_DataReceived;
			uart.Handshake = Handshake.None;
			uart.ReadTimeout = Timeout.Infinite;
			uart.Open();

			var data = Encoding.UTF8.GetBytes("\r\n+STWMOD=0\r\n");
			uart.Flush();
			uart.Write(data, 0, data.Length);

			Thread.Sleep(200);

			data = Encoding.UTF8.GetBytes("\r\n+STPIN=1234\r\n");
			uart.Flush();
			uart.Write(data, 0, data.Length);

			Thread.Sleep(200);

			data = Encoding.UTF8.GetBytes("\r\n+INQ=1\r\n");
			uart.Flush();
			uart.Write(data, 0, data.Length);
			uart.Flush();
			
			while (true)
			{
				String response = "";
				while (uart.BytesToRead > 0)
				{
					response = response + (char)uart.ReadByte();
				}
				if (response.Length > 0)
				{
					Debug.Print(response);

					//Check Bluetooth State Changed
					if (response.IndexOf("+BTSTATE:") > -1)
					{
						string atCommand = "+BTSTATE:";

						//String parsing  
						// Return format: +COPS:<mode>[,<format>,<oper>]
						int first = response.IndexOf(atCommand) + atCommand.Length;
						int last = response.IndexOf("\n", first);
						int state = int.Parse(((response.Substring(first, last - first)).Trim()));

						
					}
					//Check Pin Requested
					if (response.IndexOf("+INPIN") > -1)
					{
						// EDUARDO : Needs testing
						//OnPinRequested(this);
					}
					if (response.IndexOf("+RTINQ") > -1)
					{
						//EDUARDO: Needs testing

						string atCommand = "+RTINQ=";
						//String parsing  
						int first = response.IndexOf(atCommand) + atCommand.Length;
						int mid = response.IndexOf(";", first);
						int last = response.IndexOf("\r", first);

						// Keep reading until the end of the message
						while (last < 0)
						{
							while (uart.BytesToRead > 0)
							{
								response = response + (char)uart.ReadByte();
							}
							last = response.IndexOf("\r", first);
						}

						string address = ((response.Substring(first, mid - first)).Trim());

						string name = (response.Substring(mid + 1, last - mid));

						//OnDeviceInquired(this, address, name);
						//Debug.Print("Add: " + address + ", Name: " + name );
					}
					else
					{
						//OnDataReceived(this, response);
					}
				}
				Thread.Sleep(1);  //poundy changed from thread.sleep(10)
			}

			return;*/

			//_motorConfig = new I2CDevice.Configuration(0x40, 400);
			_imu = new I2CDevice(new I2CDevice.Configuration(0x68, 400));

			_mpu = new Mpu6050Sensor(_imu);
			_mpu.OnMeasurementReceived += MpuOnMeasurementReceived;

			/*WriteByte(_imu, _motorConfig, 0, 0); // reset
			Thread.Sleep(5);

			WriteBits(_imu, _motorConfig, 0, 4, 1, 1); //disable output
			WriteByte(_imu, _motorConfig, 0xFE, 121); // PRE_SCALE 11
			WriteBits(_imu, _motorConfig, 0, 5, 1, 1); // MODE1 autoincrement
			WriteBits(_imu, _motorConfig, 1, 2, 1, 1); // MODE2 OUTDRV = 0
			WriteBits(_imu, _motorConfig, 1, 4, 1, 0); // MODE2 INVRT = 1 ???
			WriteBits(_imu, _motorConfig, 0, 4, 1, 0); // enable output

			UInt16 on = 1;
			UInt16 off = (ushort)((double)4096 / 20);
			WriteBytes(_imu, _motorConfig, 0x6, new[] { (byte)on, (byte)(on >> 8), (byte)off, (byte)(off >> 8) });
			WriteBytes(_imu, _motorConfig, 0x6 + 4, new[] { (byte)on, (byte)(on >> 8), (byte)off, (byte)(off >> 8) });*/


			

			/*var start = DateTime.Now;
			const int interval = 50;
			var a = 0;
			while (true)
			{
				int fifoCount;
				while ((fifoCount = GetFifoCount(imu, imuConfig)) < 46)
				{
				}

				var fifo = GetFifoBytes(imu, imuConfig, System.Math.Min(fifoCount, 128)); // safeguard only 128 bytes
				var acc = GetAccel(fifo);
				var q = GetQuaternion(fifo);
				var euler = GetEuler(q);
				//Debug.Print(acc[0] + "/" + acc[1] + "/" + acc[2]);
				//Debug.Print(euler[0] + "/" + euler[1] + "/" + euler[2]);
				//Debug.Print(q.W + "/" + q.X + "/" + q.Y + "/" + q.Z);

				if (a == 0)
				{
					_lastAngle = euler[2];
				}

				var angle = euler[2];
				//if (System.Math.Abs(angle - _lastAngle) <= 4)
				{
					var angleFilter = (angle + _lastAngle) / 2;
					var output = pid.Compute(angleFilter) + .3;
					_lastAngle = angle;
					Debug.Print(angleFilter + "\t" + output);
					pwm.Duration = (uint)((1 + 0.7) * 1000);
				}
				//var duration = DateTime.Now - start;
				//Debug.Print((duration.Ticks / 10000.0 / a).ToString());
				//}
				a++;
			}*/

			Thread.Sleep(int.MaxValue);

			/*DisableSleepMode(imu, imuConfig);
			InitMag(imu, imuConfig, magConfig);

			Debug.Print("Sensor initialized");

			const int interval = 50;
			var pos = 0;
			while (true)
			{
				var accData = ReadBytes(imu, imuConfig, 0x3B, 14);
				var magData = ReadBytes(imu, magConfig, MPU9150_RA_MAG_XOUT_L, 6);

				var acc = new int[6];
				for (var j = 0; j < 6; j++)
				{
					acc[j] = accData[j * 2] << 8 | accData[j * 2 + 1];
				}

				var mag = new int[3];
				for (var j = 0; j < 3; j++)
				{
					mag[j] = magData[j * 2] << 8 | magData[j * 2 + 1];
				}

				if (pos % interval == 0)
				{
					Debug.Print("Acc:" + acc[0] + "/" + acc[1] + "/" + acc[2]);
					Debug.Print("Gyro:" + acc[3] + "/" + acc[4] + "/" + acc[5]);
					Debug.Print("Mag:" + mag[0] + "/" + mag[1] + "/" + mag[2]);
				}
				pos++;
			}*/
		}

		static void MpuOnMeasurementReceived(Mpu6050Sensor.MpuMeasurement measurement)
		{
			var acc = measurement.Acceleration;
			var gyro = measurement.Gyro;
			var q = measurement.Quaternion;
			var euler = GetEuler(q);
			var grav = GetGravity(q);
			var pitch = GetYawPitchRoll(q, grav);
			//Debug.Print(acc[0] + "\t" + acc[1] + "\t" + acc[2]);
			//Debug.Print(gyro[0] + "\t" + gyro[1] + "\t" + gyro[2]);
			//Debug.Print(euler[0] + "\t" + euler[1] + "\t" + euler[2]);
			//Debug.Print(pitch[0] + "\t" + pitch[1] + "\t" + pitch[2]);
			//Debug.Print(q.W + "\t" + q.X + "\t" + q.Y + "\t" + q.Z);
			//Debug.Print(_pwmInput[3].ToString());


			var angle = pitch[2];
			//angle = _lowPassFilter.Input(angle);
			//angle += _pwmInput[1] * 20;

			var angleKalman = _kalmanFilter.Update(1.0 / 100, new DoubleVector(gyro), new DoubleVector(new[] { q.W, q.X, q.Y, q.Z }));
			var angleKalmanQ = new Quaternion { W = angleKalman[0], X = angleKalman[1], Y = angleKalman[2], Z = angleKalman[3] };
			var gravKalman = GetGravity(angleKalmanQ);
			var pitchKalman = GetYawPitchRoll(angleKalmanQ, gravKalman);

			/*_lowPassFilterGyro.Update((float) gyro[0]);
			var gyroFilter = _lowPassFilterGyro.Value;*/

			_pid.Kd = .001 + (_pwmInput[5]) * .001;
			//_pid.Kp = -.002 - (_pwmInput[5]) * .002;

			var output = _pid.Compute(angle, gyro[0]);
			//output = 0;
			var push = (_pwmInput[2] + 1) / 2;
			push = (float)(.4 + ((_pwmInput[2] + 1) / 2) * .3);
			//push = 0;
			_pwm1.Duration = (uint)((1 + push - output) * PwmPeriode / 20);
			_pwm2.Duration = (uint)((1 + push + output) * PwmPeriode / 20);
			Debug.Print(angle + "\t" + pitchKalman[2] + "\t" + gyro[0] + "\t" + output + "\tKi=" + _pid.Ki + "\tKp=" + _pid.Kp + "\tKd=" + _pid.Kd + "\tp=" + push);

			/*const ushort on = 1;
			var off = (ushort)((double)4095 / 20 * (1 + .5 - output) + 1);
			WriteBytes(_imu, _motorConfig, 0x6, new[] { (byte)on, (byte)(on >> 8), (byte)off, (byte)(off >> 8) });
			off = (ushort)((double)4095 / 20 * (1 + .5 + output) + 1);
			WriteBytes(_imu, _motorConfig, 0x6 + 4, new[] { (byte)on, (byte)(on >> 8), (byte)off, (byte)(off >> 8) });*/

			_pos++;
		}

		private static double[] GetEuler(Quaternion q)
		{
			var data = new double[3];
			data[0] = System.Math.Atan2(2 * q.X * q.Y - 2 * q.W * q.Z, 2 * q.W * q.W + 2 * q.X * q.X - 1) / System.Math.PI * 180;   // psi
			data[1] = -System.Math.Asin(-2 * (q.X * q.Z - q.W * q.Y)) / System.Math.PI * 180;                                       // theta
			data[2] = System.Math.Atan2(2 * q.Y * q.Z - 2 * q.W * q.X, 2 * q.W * q.W + 2 * q.Z * q.Z - 1) / System.Math.PI * 180;   // phi
			return data;
		}

		private static double[] GetYawPitchRoll(Quaternion q, double[] g)
		{
			var data = new double[3];
			data[0] = System.Math.Atan2(2 * q.X * q.Y - 2 * q.W * q.Z, 2 * q.W * q.W + 2 * q.X * q.X - 1) / System.Math.PI * 180;   // psi
			var div = System.Math.Sqrt(g[0] * g[0] + g[2] * g[2]);
			if (div != 0)
			{
				data[1] = System.Math.Atan(g[0] / div) / System.Math.PI * 180; // theta
				data[2] = System.Math.Atan(g[1] / div) / System.Math.PI * 180; // phi
			}
			return data;
		}

		private static double[] GetGravity(Quaternion q)
		{
			var data = new double[3];
			data[0] = 2 * (q.X * q.Z - q.W * q.Y); // x
			data[1] = 2 * (q.W * q.X + q.Y * q.Z); // y
			data[2] = q.W * q.W - q.X * q.X - q.Y * q.Y + q.Z * q.Z; // z
			return data;
		}
	}
}
