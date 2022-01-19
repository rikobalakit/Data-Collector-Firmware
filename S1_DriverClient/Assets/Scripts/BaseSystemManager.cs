using UnityEngine;

public class BaseSystemManager : MonoBehaviour
{

    private GameManager _gameManager;
    public GameManager GameManager => _gameManager;
    private bool _isInitialized = false;
    public bool IsInitialized => _isInitialized;
    
    public virtual void Initialize(GameManager manager)
    {
        _gameManager = manager;

        _isInitialized = true;
    }

    public virtual void UpdateFromGameManager()
    {
        
    }
}
