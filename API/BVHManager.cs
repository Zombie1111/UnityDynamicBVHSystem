using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class BVHManager : MonoBehaviour
{
    #region Singleton
    
    private static BVHManager instance;
    internal static BVHManager _instance
    {
        get
        {
            if (instance != null) return instance;
    
            instance = GameObject.FindAnyObjectByType<BVHManager>(FindObjectsInactive.Exclude);
            if (instance == null)
            {
                GameObject newObj = new("BVHSystem")
                {
                    hideFlags = HideFlags.HideAndDontSave
                };
    
                instance = newObj.AddComponent<BVHManager>();
            }
    
            if (instance.transform.childCount > 0) Debug.Log("The BVHManager should not have any children but " + instance.transform.name + " has!");
            instance.gameObject.SetActive(true);
            DontDestroyOnLoad(instance.gameObject);
            return instance;
        }
    }

    #endregion Singleton

    #region Main

    private bool isInitilized = false;

    private void Awake()
    {
        Init();
    }

    private void Init()
    {
        if (isInitilized == true) return;
        isInitilized = true;
    }
        
    private void OnDestroy()
    {
        Destroy();
    }

    private void Destroy()
    {
        if (isInitilized == false) return;
        isInitilized = false;

        Destroy(this);
    }

    #endregion Main
}
