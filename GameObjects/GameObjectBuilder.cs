using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Rendering;
using BLASObject = BLASBuilder.BLASObject;
using static UnityEngine.Mesh;
using Unity.Collections.LowLevel.Unsafe;

public static class GameObjectBuilder
{
    /// <summary>
    /// Contains data required to build a BLASOObject from a Collider or MeshFilter.
    /// </summary>
    [BurstCompile]
    internal readonly struct GameObjectData
    {
        internal unsafe GameObjectData(Collider col, MeshFilter mf)
        {
            subMeshMatI = new();
            void* subMeshMatI_ptr = UnsafeUtility.AddressOf(ref subMeshMatI);
            short* subMeshMatI_shorts = (short*)subMeshMatI_ptr;

            if (mf != null)
            {
                offset = Vector3.zero;
                scale = Vector3.one;
                type = ShapeType.ConcaveMesh;
                Mesh mesh = mf.sharedMesh;

                meshData = Mesh.AcquireReadOnlyMeshData(mesh)[0];
                if (SetSubMeshMatIndexs(mesh.subMeshCount) == false)
                {
                    type = ShapeType.Invalid;
                    return;
                }
            }
            else if (col is MeshCollider meshC)
            {
                offset = Vector3.zero;
                scale = Vector3.one;
                type = meshC.convex == true ? ShapeType.ConvexMesh : ShapeType.ConcaveMesh;

                Mesh mesh = meshC.sharedMesh;

                Debug.Log(mesh.triangles.Length + " " + mesh.vertices.Length + " " + mesh.GetIndexCount(0));
                meshData = Mesh.AcquireReadOnlyMeshData(mesh)[0];
                if (SetSubMeshMatIndexs(meshC.convex == true ? 1 : mesh.subMeshCount) == false)
                {
                    type = ShapeType.Invalid;
                    return;
                }
            }
            //else if (col is BoxCollider boxC)
            //{
            //    offset = boxC.center;
            //    scale = boxC.size;
            //    type = ShapeType.Box;
            //    meshData = new();
            //}
            //else if (col is SphereCollider sphereC)
            //{
            //    center = sphereC.center;
            //    shape = new Vector3(sphereC.radius, 0.0f, 0.0f);
            //    type = ShapeType.Sphere;
            //    meshData = new();
            //}
            //else if (col is CapsuleCollider capC)
            //{
            //    center = capC.center;
            //    shape = new Vector3(capC.radius, capC.height, capC.direction);
            //    type = ShapeType.Sphere;
            //    meshData = new();
            //}
            else
            {
                Debug.LogError(col.GetType() + " is not supported by the BVH builder!");
                offset = Vector3.zero; scale = Vector3.zero; type = ShapeType.Invalid; meshData = new();
                return;
            }

            bool SetSubMeshMatIndexs(int subMeshCount)
            {
                Transform trans = col == null ? mf.transform : col.transform;

                if (subMeshCount > _maxSubMeshes)
                {
                    Debug.LogError(trans.name + " had more than " + _maxSubMeshes + " submeshes! Either reduce subMesh count or increase _maxSubMeshes");
                    return false;
                }

                for (int i = 0; i < subMeshCount; i++)
                {
                    subMeshMatI_shorts[i] = 0;//Fixme, get actuall material index
                }

                return true;
            }
        }

        internal int GetId()
        {
            int id = 17;

            unchecked
            {
                id *= 31 + (int)Math.Round(offset.x * 1000);
                id *= 31 + (int)Math.Round(offset.y * 1000);
                id *= 31 + (int)Math.Round(offset.z * 1000);
                id *= 31 + (int)Math.Round(scale.x * 1000);
                id *= 31 + (int)Math.Round(scale.y * 1000);
                id *= 31 + (int)Math.Round(scale.z * 1000);
                id *= 31 + (int)type;
                if (type == ShapeType.ConcaveMesh || type == ShapeType.ConvexMesh) id *= 31 + meshData.vertexCount;
            }

            return id;
        }

        internal readonly Vector3 offset;
        /// <summary>
        /// (Sphere: X radius) (Capsule: X radius, Y height, Z direction)
        /// </summary>
        internal readonly Vector3 scale;
        internal readonly ShapeType type;
        internal readonly MeshData meshData;
        internal readonly FixedBytes16 subMeshMatI; internal const int _maxSubMeshes = 8;
    }

    internal static unsafe BLASObject CreateFrom(in GameObjectData god)
    {
        if (god.type != ShapeType.ConcaveMesh)
        {
            throw new Exception(god.type + " not implemented!");
        }

        //Get mesh data and allocate native arrays
        int subMeshCount = god.meshData.subMeshCount;
        int maxIndicesCount = 0;
        int totalIndicesCount = 0;

        for (int i = 0; i < subMeshCount; i++)
        {
            SubMeshDescriptor subMD = god.meshData.GetSubMesh(i);
            if (subMD.topology != MeshTopology.Triangles) Debug.LogError("Mesh topology must be Triangles");
            maxIndicesCount = Math.Max(maxIndicesCount, subMD.indexCount);
            totalIndicesCount += subMD.indexCount;
        }

        int verCount = god.meshData.vertexCount;
        if (totalIndicesCount % 3 != 0)
        {
            //== 2 means its one less vertex so out of bounds is likely
            if (totalIndicesCount % 3 == 2) Debug.LogError(god.meshData + " vertex count is not dividable with 3, may cause issues!");
            else Debug.LogWarning(god.meshData + " vertex count is not dividable with 3, may cause issues!");
        }

        int triCount = totalIndicesCount / 3;
        int triI = 0;
        NativeArray<int> indices = new(maxIndicesCount, Allocator.Temp);
        NativeArray<Vector3> vertics = new(verCount, Allocator.Temp);
        NativeArray<Triangle.Extended> eTris = new(triCount, Allocator.Temp);
        god.meshData.GetVertices(vertics);

        //Get triangles per submesh
        fixed (FixedBytes16* subMeshMatI_ptr = &god.subMeshMatI)
        {
            short* subMeshMatI_shorts = (short*)subMeshMatI_ptr;

            for (int i = 0; i < subMeshCount; i++)
            {
                SubMeshDescriptor subMD = god.meshData.GetSubMesh(i);
                int subIndiceCount = subMD.indexCount;
                god.meshData.GetIndices(indices, i, true);

                for (int subIndiceI = 0; subIndiceI < subIndiceCount; subIndiceI += 3)
                {
                    eTris[triI] = new(vertics[indices[subIndiceI]], vertics[indices[subIndiceI + 1]], vertics[indices[subIndiceI + 2]], subMeshMatI_shorts[i]);
                    triI++;
                }
            }
        }

        //Build BVH
        return new(eTris);
    }

    internal enum ShapeType
    {
        ConcaveMesh = 0,
        ConvexMesh = 1,
        box = 10,
        Sphere = 11,
        Capsule = 12,
        Invalid = 100,
    }
}
