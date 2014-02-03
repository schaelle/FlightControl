using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace FlightControl
{
	public interface IImuSensor
	{
		event OnMeasurementUpdatedHandler OnMeasurementUpdated;
	}

	public delegate void OnMeasurementUpdatedHandler(ImuMeasurment msg);

	public class ImuMeasurment
	{
	}
}
