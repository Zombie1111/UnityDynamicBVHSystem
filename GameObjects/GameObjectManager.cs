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
    //TLASBuilder.TLASScene tlas;
    private BLASBuilder.BLASObject blas;

    private bool iscre = false;

    // Start is called before the first frame update
    void ddddd()
    {
        blasInstances = new(1, Allocator.Persistent);
        blasInstanceLocks = new(1, Allocator.Persistent);

        GameObjectBuilder.GameObjectData gOd = new(GetComponent<MeshCollider>(), null);
        blas = GameObjectBuilder.CreateFrom(gOd);

        Matrix4x4 wToL = transform.worldToLocalMatrix;
        Matrix4x4 lToW = transform.localToWorldMatrix;
        //blasInstances[0] = new(blas, ref wToL, ref lToW);
        blasInstanceLocks[0] = 0;
        //tlas = new(blasInstances, blasInstanceLocks, 1);
        iscre = true;
    }

    private RayJob job;

    private void Update()
    {
        if (iscre == false) ddddd();

        Matrix4x4 wToL = transform.worldToLocalMatrix;
        Matrix4x4 lToW = transform.localToWorldMatrix;
        blasInstances[0] = new(blas, ref wToL, ref lToW);

        job = new()
        {
            tlas = new(new(blasInstances, blasInstanceLocks, 1), Allocator.Persistent),
            pos = raySource.position,
            forward = raySource.forward
        };

        var handle = job.Schedule();
        handle.Complete();

        job.tlas.Dispose();
        //DoRays();
    }

    [BurstCompile]
    private struct RayJob : IJob
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

    //[BurstCompile]
    //private void DoRays()
    //{
    //    for (int i = 0; i < 10000; i++)
    //    {
    //        Ray ray = new(raySource.position, raySource.forward);
    //        tlas.Raycast(ray, out Hit hit);
    //    }
    //}
    //
    //private void OnDrawGizmos()
    //{
    //    return;
    //    if (Application.isPlaying == false) return;
    //
    //    if (iscre == false) ddddd();
    //
    //    Matrix4x4 wToL = transform.worldToLocalMatrix;
    //    Matrix4x4 lToW = transform.localToWorldMatrix;
    //    blasInstances[0] = new(blas, ref wToL, ref lToW);
    //
    //    Gizmos.color = Color.yellow;
    //    tlas = new(blasInstances, blasInstanceLocks, 1);
    //
    //    Gizmos.color = Color.green;
    //    tlas.Debug_drawGismos();
    //
    //    Ray ray = new(raySource.position, raySource.forward);
    //    tlas.Raycast(ray, out Hit hit);
    //    Gizmos.color = Color.blue;
    //    Gizmos.DrawLine(raySource.position, hit.pos);
    //
    //    //Physics.Raycast(ray.orgin, ray.direction, out RaycastHit hitt);
    //    //Debug.Log("ur " + hitt.distance + " br " + hit.dis);
    //}

    private void OnDestroy()
    {
        //tlas.Dispose();
        blasInstances.Dispose();
        blasInstanceLocks.Dispose();
    }
}
