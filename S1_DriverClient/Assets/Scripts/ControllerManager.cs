using UnityEngine;

public class ControllerManager : BaseSystemManager
{

    private float _leftVerticalValue = 0f;
    private float _rightVerticalValue = 0f;
    private float _leftTriggerValue = 0f;
    private float _rightTriggerValue = 0f;

    private bool _turboOn = false;
    private bool _slowOn = false;
    private bool _resetOn = false;

    public float LeftVerticalValue
    {
        get { return _leftVerticalValue; }
    }

    public float RightVerticalValue
    {
        get { return _rightVerticalValue; }
    }

    public float LeftTriggerValue
    {
        get { return _leftTriggerValue; }
    }

    public float RightTriggerValue
    {
        get { return _rightTriggerValue; }
    }

    public bool TurboOn
    {
        get { return _turboOn; }
    }

    public bool SlowOn
    {
        get { return _slowOn; }
    }

    public bool ResetOn
    {
        get { return _resetOn; }
    }

    public override void UpdateFromGameManager()
    {
        _leftVerticalValue = Input.GetAxis("LeftVertical");
        _rightVerticalValue = Input.GetAxis("RightVertical");
        _leftTriggerValue = Input.GetAxis("LeftTrigger");
        _rightTriggerValue = Input.GetAxis("RightTrigger");
        
        _slowOn = Input.GetButton("SlowMotion");
        _turboOn = Input.GetButton("Turbo");
        _resetOn = Input.GetButton("ResetBluetooth");
    }

}