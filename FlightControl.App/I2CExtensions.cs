using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace FlightControl.App
{
	public static class I2CExtensions
	{
		private const int I2CTimeout = 1000;

		public static void WriteByte(this I2CDevice device, I2CDevice.Configuration config, byte block, byte data)
		{
			var transactions = new I2CDevice.I2CTransaction[1];
			transactions[0] = I2CDevice.CreateWriteTransaction(new[] { block, data });

			lock (device)
			{
				device.Config = config;
				if (device.Execute(transactions, I2CTimeout) == 0)
				{
					throw new Exception("Failed to perform I2C transaction");
				}
			}
		}

		public static void WriteBytes(this I2CDevice device, I2CDevice.Configuration config, byte block, byte[] data)
		{
			var transactions = new I2CDevice.I2CTransaction[1];
			var buffer = new byte[data.Length + 1];
			buffer[0] = block;
			Array.Copy(data, 0, buffer, 1, data.Length);
			transactions[0] = I2CDevice.CreateWriteTransaction(buffer);
			lock (device)
			{
				device.Config = config;
				if (device.Execute(transactions, I2CTimeout) == 0)
				{
					throw new Exception("Failed to perform I2C transaction");
				}
			}
		}

		public static byte[] ReadBytes(this I2CDevice device, I2CDevice.Configuration config, byte block, int length)
		{
			var transactions = new I2CDevice.I2CTransaction[2];
			transactions[0] = I2CDevice.CreateWriteTransaction(new[] { block });
			var buffer = new byte[length];
			transactions[1] = I2CDevice.CreateReadTransaction(buffer);

			lock (device)
			{
				device.Config = config;
				if (device.Execute(transactions, I2CTimeout) == 0)
				{
					//throw new Exception("Failed to perform I2C transaction");
				}
			}
			return buffer;
		}

		public static void WriteBits(this I2CDevice device, I2CDevice.Configuration config, byte block, byte bitStart, byte length, byte data)
		{
			var transactions = new I2CDevice.I2CTransaction[2];
			transactions[0] = I2CDevice.CreateWriteTransaction(new[] { block });
			var buffer = new byte[1];
			transactions[1] = I2CDevice.CreateReadTransaction(buffer);

			lock (device)
			{
				device.Config = config;
				if (device.Execute(transactions, I2CTimeout) == 0)
				{
					throw new Exception("Failed to perform I2C transaction");
				}

				var mask = ((1 << length) - 1) << (bitStart - length + 1);
				data <<= (bitStart - length + 1); // shift data into correct position
				data = (byte)(data & mask); // zero all non-important bits in data
				buffer[0] = (byte)(buffer[0] & ~(mask)); // zero all important bits in existing byte
				buffer[0] |= data; // combine data with existing byte

				WriteByte(device, config, block, buffer[0]);
			}
		}
	}
}
