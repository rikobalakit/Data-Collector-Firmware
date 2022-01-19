using UnityEngine;

public class TranslationManager : BaseSystemManager
{

    private const float SLOW_MULTIPLIER = 0.2f;
    private const float TURBO_MULTIPLIER = 1f;
    private const float DEFAULT_MULTIPLIER = 0.5f;
    
    private float _leftMotorThrottle = 0f;
    private float _rightMotorThrottle = 0f;
    private float _weaponMotorThrottle = 0f;

    public float LeftMotorThrottle
    {
        get { return _leftMotorThrottle; }
    }

    public float RightMotorThrottle
    {
        get { return _rightMotorThrottle; }
    }

    public float WeaponMotorThrottle
    {
        get { return _weaponMotorThrottle; }
    }

    public override void UpdateFromGameManager()
    {
        float throttleMultiplier = 0f;
        
        if (GameManager.ControllerManager.TurboOn && !GameManager.ControllerManager.SlowOn)
        {
            throttleMultiplier = TURBO_MULTIPLIER;
        }
        else if(!GameManager.ControllerManager.TurboOn && GameManager.ControllerManager.SlowOn)
        {
            throttleMultiplier = SLOW_MULTIPLIER;
        }
        else
        {
            throttleMultiplier = DEFAULT_MULTIPLIER;
        }
        
        _leftMotorThrottle = GameManager.ControllerManager.LeftVerticalValue * throttleMultiplier;
        _rightMotorThrottle = GameManager.ControllerManager.RightVerticalValue * throttleMultiplier;
        _weaponMotorThrottle = Mathf.Max(GameManager.ControllerManager.LeftTriggerValue, GameManager.ControllerManager.RightTriggerValue)/2f+0.5f;
    }
}
