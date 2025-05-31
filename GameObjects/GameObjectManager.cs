using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

public class GameObjectManager : MonoBehaviour
{
    [SerializeField] private Transform raySource;
    private NativeArray<BLASBuilder.BLASInstance> blasInstances;
    private NativeArray<int> blasInstanceLocks;
    private BLASBuilder.BLASObject blas;

    private bool isInitlized = false;

    // Start is called before the first frame update
    void Init()
    {
        blasInstances = new(1, Allocator.Persistent);
        blasInstanceLocks = new(1, Allocator.Persistent);

        GameObjectBuilder.GameObjectData gOd = new(GetComponent<MeshCollider>(), null);
        //blas = GameObjectBuilder.CreateFrom(gOd);
        Test_BlasJob blasJob = new()
        {
            god = new(gOd, Allocator.TempJob),
            blas = new(Allocator.TempJob)
        };

        HelpMethods.Debug_toggleTimer();
        var handle = blasJob.Schedule();
        handle.Complete();
        HelpMethods.Debug_toggleTimer();

        gOd.Dispose();
        blas = blasJob.blas.Value;
        blasJob.blas.Dispose();
        blasJob.god.Dispose();
        blasInstanceLocks[0] = 0;
        isInitlized = true;
    }

    private Test_RayJob rayJob;


    [BurstCompile]
    private struct Test_RayJob : IJob
    {
        public NativeReference<TLASBuilder.TLASScene> tlas;
        public Vector3 pos;
        public Vector3 forward;

        public void Execute()
        {
            TLASBuilder.TLASScene t = tlas.Value;

            for (int i = 0; i < 10000; i++)
            {
                Ray ray = new(pos + (Vector3.one * (i / 10000.0f)), forward);
                t.Raycast(ray, out Hit hit);
                Debug.DrawLine(hit.pos, ray.orgin);
            }
        }
    }


    [BurstCompile]
    private struct Test_BlasJob : IJob
    {
        public NativeReference<GameObjectBuilder.GameObjectData> god;
        public NativeReference<BLASBuilder.BLASObject> blas;

        public void Execute()
        {
            blas.Value = GameObjectBuilder.CreateFrom(god.Value);
        }
    }

    private void Update()
    {
        if (isInitlized == false) Init();
    
        //Debug rays
        Matrix4x4 wToL = transform.worldToLocalMatrix;
        Matrix4x4 lToW = transform.localToWorldMatrix;
        blasInstances[0] = new(blas, ref wToL, ref lToW);

        if (rayJob.tlas.IsCreated == true)
        {
            rayJob.tlas.Value.Dispose();
            rayJob.tlas.Dispose();
        }

        rayJob = new()
        {
            tlas = new(new(blasInstances, blasInstanceLocks, 1), Allocator.Persistent),
            pos = raySource.position,
            forward = raySource.forward
        };
    
        var handle = rayJob.Schedule();
        handle.Complete();
    }

    private void OnDrawGizmos()
    {
        //return;
        if (Application.isPlaying == false) return;
        if (isInitlized == false) Init();
    
        ////Draw tlas
        if (rayJob.tlas.IsCreated == true) rayJob.tlas.Value.Debug_drawGizmos();
    }

    private void OnDestroy()
    {
        blasInstances.Dispose();
        blasInstanceLocks.Dispose();

        if (rayJob.tlas.IsCreated == true)
        {
            rayJob.tlas.Value.Dispose();
            rayJob.tlas.Dispose();
        }

        blas.Dispose();
    }
}
