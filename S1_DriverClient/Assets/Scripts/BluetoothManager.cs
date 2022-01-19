using UnityEngine;
using System.IO.Ports;

public class BluetoothManager : BaseSystemManager
{

    SerialPort _serialPort;

    public override void Initialize(GameManager manager)
    {
        base.Initialize(manager);

        string[] ports = SerialPort.GetPortNames();

        for (int i = 0; i < ports.Length; i++)
        {
            Debug.Log(ports[i]);
        }


        _serialPort = new SerialPort(ports[1], 9600, Parity.None, 8, StopBits.One); // This says COM13 doesn't exist
        _serialPort.Open();


    }

    public override void UpdateFromGameManager()
    {
        if (_serialPort.IsOpen)
        {
            var dataToSend =
                $"{{\"l\": {GameManager.TranslationManager.LeftMotorThrottle},\"r\":{GameManager.TranslationManager.RightMotorThrottle}, \"w\": {GameManager.TranslationManager.WeaponMotorThrottle}}}\n";
            Debug.Log(dataToSend);
            _serialPort.Write(dataToSend);
        }
        else
        {
            Debug.LogError("Serial port is closed");
        }

    }

}