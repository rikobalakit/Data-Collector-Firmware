using UnityEngine;

public class GameManager : BaseSystemManager
{
    
    private BluetoothManager _bluetoothManager;
    private ControllerManager _controllerManager;
    private TranslationManager _translationManager;

    public BluetoothManager BluetoothManager => _bluetoothManager;
    public ControllerManager ControllerManager => _controllerManager;
    public TranslationManager TranslationManager => _translationManager;
    
    private void Start()
    {
        Time.fixedDeltaTime = 0.05f;
        
        _controllerManager = gameObject.AddComponent<ControllerManager>();
        _translationManager = gameObject.AddComponent<TranslationManager>();
        _bluetoothManager = gameObject.AddComponent<BluetoothManager>();
        
        _controllerManager.Initialize(this);
        _translationManager.Initialize(this);
        _bluetoothManager.Initialize(this);
    }

    private void FixedUpdate()
    {
        _controllerManager.UpdateFromGameManager();
        _translationManager.UpdateFromGameManager();
        _bluetoothManager.UpdateFromGameManager();
    }

}
