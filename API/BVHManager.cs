using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

public abstract class BVHManager : MonoBehaviour
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
                    //hideFlags = HideFlags.HideAndDontSave
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

    private class NativeReusableArray<T> where T : unmanaged
    {
        private readonly Queue<int> unusedIndexes;
        public NativeArray<T> array;
        public int[] arrayUsers;
        public readonly int lenght;

        public NativeReusableArray(int lenght, Allocator allocator)
        {
            this.lenght = lenght;
            unusedIndexes = new(lenght);
            array = new NativeArray<T>(lenght, allocator);
            arrayUsers = new int[lenght];
            Clear();
        }

        public void AddUserToIndex(int index)
        {
            arrayUsers[index]++;
        }

        public void RemoveUserFromIndex(int index)
        {
            arrayUsers[index]--;
            if (arrayUsers[index] > 0) return;

            unusedIndexes.Enqueue(index);
        }

        public int GetUnusedIndex()
        {
            return unusedIndexes.Dequeue();
        }

        public void Clear()
        {
            unusedIndexes.Clear();
            int lenght = this.lenght;

            for (int i = 0; i < lenght; i++)
            {
                arrayUsers[i] = 0;
                unusedIndexes.Enqueue(i);
            }
        }

        public void Dispose()
        {
            array.Dispose();
        }

        public T this[int index]
        {
            // Getter
            get
            {
                return array[index];
            }
            // Setter
            set
            {
                array[index] = value;
            }
        }
    }

    private bool isInitilized = false;

    private void Awake()
    {
        Init();
    }

    private void Init()
    {
        if (isInitilized == true) return;
        isInitilized = true;
        
        blasObjects = new NativeReusableArray<BLASBuilder.BLASObject>(maxBlasObjects, Allocator.Persistent);
    }
        
    private void OnDestroy()
    {
        Destroy();
    }

    private void Destroy()
    {
        if (isInitilized == false) return;
        isInitilized = false;

        blasObjects.Dispose();
        Destroy(this);
    }

    #endregion Main

    #region Adding Objects

    private const int maxBlasObjects = 1024;
    private NativeReusableArray<BLASBuilder.BLASObject> blasObjects;

    public int AddBlasObject(in BLASBuilder.BLASObject blasO)
    {
        int index = blasObjects.GetUnusedIndex();
        blasObjects[index] = blasO;
        return index;
    }

    private const int maxBlasInstances = 1024;
    private NativeReusableArray<BLASBuilder.BLASInstance> blasInstances;

    //public int CreateBlasInstance(int blasIndex, ref Matrix4x4 wToL, ref Matrix4x4 lToW)
    //{
    //    int index = blasInstances.GetUnusedIndex();
    //    blasInstances[index] = new BLASBuilder.BLASInstance(blasObjects[blasIndex], ref wToL, ref lToW);
    //}

    #endregion Adding Objects

    #region Getting Tlas

    private Dictionary<int, TLASBuilder.TLASScene> activeTlasScenes = new();
    private int nextTlasID = 0;

    //public TLASBuilder.TLASScene GetReadonlyTlas(out int tlasID)
    //{
    //    nextTlasID++;
    //
    //}
    //
    //public void ReturnReadonlyArray(int tlasID)
    //{
    //
    //}

    #endregion Getting Tlas
}
