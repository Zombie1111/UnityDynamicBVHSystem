using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using Unity.Collections.LowLevel.Unsafe;
using PlasticGui.Help.Actions;
using Unity.Collections;
using PlasticPipe.PlasticProtocol.Messages;
using System.Linq;
using UnityEngine.AI;

public class BVHSystem : MonoBehaviour
{
    #region Singleton

    private static bool isInitilized = false;
    private static bool canCreateInstance = false;

    private static BVHSystem instance;
    internal static BVHSystem _instance
    {
        get
        {
            if (instance != null || canCreateInstance == false) return instance;

            instance = GameObject.FindAnyObjectByType<BVHSystem>(FindObjectsInactive.Exclude);
            if (instance == null)
            {
                GameObject newObj = new("BVHSystem")
                {
                    hideFlags = HideFlags.HideAndDontSave
                };

                instance = newObj.AddComponent<BVHSystem>();
            }

            if (instance.transform.childCount > 0) Debug.Log("The BVHSystem should not have any children but " + instance.transform.name + " has!");
            instance.gameObject.SetActive(true);
            DontDestroyOnLoad(instance.gameObject);
            return instance;
        }
    }

    #endregion Singleton

    #region Main

    private void Awake()
    {
        //Allocate
        flipA.Allocate();
        flipB.Allocate();
        isFlipped = false;
        FlipFlops();

        SceneManager.sceneLoaded += OnSceneLoaded;
        OnSceneLoaded(SceneManager.GetActiveScene(), LoadSceneMode.Additive);
    }

    private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        if (mode == LoadSceneMode.Single)
        {

        }

#pragma warning disable CS0162 // Unreachable code detected
        if (Globals._registerAllCollidersOnLoad == false) return;

        foreach (GameObject rootObj in scene.GetRootGameObjects())
        {
            foreach (Collider col in rootObj.GetComponentsInChildren<Collider>(true))
            {
                BLASBuilder.ObjectData od = new(col, null);
            }
        }
#pragma warning restore CS0162 // Unreachable code detected
    }

    private void OnDestroy()
    {
        if (isInitilized == false) return;

        isInitilized = false;
        canCreateInstance = false;

        flipA.Dispose();
        flipB.Dispose();
        SceneManager.sceneLoaded -= OnSceneLoaded;

        Destroy(this);
        instance = null;//Set null immediately
    }

    #endregion Main

    #region Manage Objects

    private enum ObjectState
    {
        
    }

    private struct BLASInstance
    {
        internal Matrix4x4 wToL;
        internal int blasObjectI;
        /// <summary>
        /// 0 = invalid, 1 = valid disabled, 2 = valid enabled
        /// </summary>
        internal sbyte state;
    }

    private struct BLASInput
    {
        internal int newBLASObjectI;
        internal int newState;
    }

    /// <summary>
    /// col/mf to BLAS instance index
    /// </summary>
    private readonly Dictionary<UnityEngine.Object, int> managedObjects = new(16);

    public void RegisterObject(Collider col, MeshFilter mf = null)
    {
        GameObject obj = col != null ? col.gameObject : mf.gameObject;
        if (managedObjects.ContainsKey(obj) == true) return;

        int blasInstanceI = GetBLASInstanceI();
        if (blasInstanceI < 0) return;

        int blasObjectI = GetBlastObjectFor(col, mf);
        if (blasObjectI < 0) return;

        managedObjects[obj] = blasInstanceI;
        activeFlip.blasInputs.Add(blasInstanceI, new()
        {
            newState = obj.activeInHierarchy == true ? 2 : 1,
            newBLASObjectI = blasObjectI,
        });
    }

    private int nextBLASInstanceI = 0;
    private readonly Queue<int> unusedBLASInstanceIs = new(16);

    private int GetBLASInstanceI()
    {
        if (unusedBLASInstanceIs.TryDequeue(out int blasI) == true) return blasI;
        if (nextBLASInstanceI >= Globals._maxRegisteredBlasInstances)
        {
            Debug.LogError("Unable to register BLAS instance, too many already exists. Please destroy some or increase _maxRegisteredBlasInstances");
            return -1;
        }

        blasI = nextBLASInstanceI;
        nextBLASInstanceI++;
        return blasI;
    }

    private class BLASObjectData
    {
        internal int index;
        internal int numUsers;
        internal bool runtimeCreated;
    }

    private void BLASObjectAddUser(int shapeID)
    {
        managedBLASObjects[shapeID].numUsers++;
    }

    private void BLASObjectRemoveUser(int shapeID)
    {
        BLASObjectData od = managedBLASObjects[shapeID];
        od.numUsers--;

        if (od.numUsers == 0 && od.runtimeCreated == true)
        {
            MarkBLASObjectUnused(od, shapeID);
        }
    }

    private void MarkBLASObjectUnused(BLASObjectData od, int shapeID)
    {
        unusedBLASObjectIs.Enqueue(od.index);
        managedBLASObjects.Remove(shapeID);
    }

    /// <summary>
    /// shape id to BLASObjectData
    /// </summary>
    private readonly Dictionary<int, BLASObjectData> managedBLASObjects = new(16);
    private int nextBLASObjectI = 0;
    private readonly Queue<int> unusedBLASObjectIs = new(16);

    private int GetBlastObjectFor(Collider col, MeshFilter mf)
    {
        return GetBlastObjectFor(HelpMethods.GetShapeIDFrom(col, col != null ? null : mf), false);
    }

    /// <summary>
    /// Can also be used to simple create a BlastObject
    /// </summary>
    private int GetBlastObjectFor(int shapeID, bool runtimeCreated)
    {
        if (managedBLASObjects.TryGetValue(shapeID, out BLASObjectData od) == true) return od.index;

        if (unusedBLASObjectIs.TryDequeue(out int objI) == false)
        {
            if (nextBLASObjectI >= Globals._maxRegisteredBlasObjects)
            {
                Debug.LogError("Unable to register BLAS object, too many already exists. Please destroy some or increase _maxRegisteredBlasObjects");
                return -1;
            }

            objI = nextBLASObjectI;
            nextBLASObjectI++;
        }

        managedBLASObjects.Add(shapeID, new()
        {
            index = objI,
            numUsers = 0,
            runtimeCreated = runtimeCreated
        });

        return objI;
    }

    #endregion Manage Objects

    #region FlipFlop

    private void FlipFlops()
    {
        isFlipped = !isFlipped;
        activeFlip = isFlipped == false ? flipA : flipB;
    }

    private bool isFlipped = false;
    private FlipFlopped activeFlip = null;
    private FlipFlopped flipA = new();
    private FlipFlopped flipB = new();

    private class FlipFlopped
    {
        /// <summary>
        /// BLAS instance index to its new data
        /// </summary>
        internal NativeHashMap<int, BLASInput> blasInputs;

        internal void Allocate()
        {
            blasInputs = new NativeHashMap<int, BLASInput>(16, Allocator.Persistent);
        }

        internal void Dispose()
        {
            if (blasInputs.IsCreated == true) blasInputs.Dispose();
        }
    }

    #endregion FlipFlop
}
