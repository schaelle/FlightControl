using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using FlightControl.Imu.Android;

namespace FlightControl
{
	class Program
	{
		static void Main(string[] args)
		{
			new AndroidImuSensor(5555);

			Console.WriteLine("Started");
			Console.ReadLine();
		}
	}
}