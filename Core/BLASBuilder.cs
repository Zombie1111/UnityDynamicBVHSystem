using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Burst;
using static UnityEngine.Mesh;
using System;
using UnityEngine.Rendering;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;

public static class BLASBuilder
{
    //Creates BVHObjects
    //A node in the BVH tree
    [BurstCompile]
    internal struct Node
    {
        internal Vector3 min;
        internal Vector3 max;
        /// <summary>
        /// (If triCount == 0 this is left kid index (Right kid index is always +1))
        /// (If triCount > 0 this is index of the first triangle)
        /// </summary>
        internal int leftStartI;
        internal int triCount;
    }

    /// <summary>
    /// Contains data required to build a BLASOObject from a Collider or MeshFilter.
    /// </summary>
    [BurstCompile]
    internal readonly unsafe struct GameObjectData
    {
        internal GameObjectData(Collider col, MeshFilter mf)
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
                Transform trans = col == null ? mf.transform: col.transform;

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

    internal enum ShapeType
    {
        ConcaveMesh = 0,
        ConvexMesh = 1,
        box = 10,
        Sphere = 11,
        Capsule = 12,
        Invalid = 100,
    }

    /// <summary>
    /// A BLASObject is a local BVH
    /// </summary>
    [BurstCompile]
    internal readonly struct BLASObject
    {
        internal readonly NativeArray<Node> nodes;
        internal readonly NativeArray<Triangle> tris;

        #region BLASObject creation
        private BLASObject(NativeArray<Triangle.Extended> eTris)
        {
            int triCount = eTris.Length;
            NativeArray<Node> nodes = this.nodes = new(triCount * 2 - 1, Allocator.Persistent);
            tris = new(triCount, Allocator.Persistent);

            //Build BVH
            int rootNodeI = 0;
            int nodesUsed = 1;
            Node rootNode = nodes[rootNodeI];

            rootNode.leftStartI = 0;
            rootNode.triCount = triCount;

            Subdivide(ref rootNode, rootNodeI);
            void Subdivide(ref Node node, int nodeI)
            {
                //Update bounds
                node.min = new Vector3(1e30f, 1e30f, 1e30f);
                node.max = new Vector3(-1e30f, -1e30f, -1e30f);

                for (int first = node.leftStartI, ii = 0; ii < node.triCount; ii++)
                {
                    Triangle.Extended tri = eTris[first + ii];

                    node.min = HelpMethods.Min(node.min, tri.v0);
                    node.min = HelpMethods.Min(node.min, tri.v1);
                    node.min = HelpMethods.Min(node.min, tri.v2);
                    node.max = HelpMethods.Max(node.max, tri.v0);
                    node.max = HelpMethods.Max(node.max, tri.v1);
                    node.max = HelpMethods.Max(node.max, tri.v2);
                }

                //Terminate recursion
                if (node.triCount <= 2)
                {
                    nodes[nodeI] = node;
                    return;
                }

                //Determine split axis and position
                Vector3 extent = node.max - node.min;
                int axis = 0;
                if (extent.y > extent.x) axis = 1;
                if (extent.z > extent[axis]) axis = 2;
                float splitPos = node.min[axis] + extent[axis] * 0.5f;

                //Split triangles
                int i = node.leftStartI;
                int j = i + node.triCount - 1;

                while (i <= j)
                {
                    if (eTris[i].center[axis] < splitPos)
                    {
                        i++;
                        continue;
                    }

                    Triangle.Extended tri = eTris[i];
                    eTris[i] = eTris[j--];
                    eTris[j] = tri;
                }

                //Want more kids?
                int leftCount = i - node.leftStartI;
                if (leftCount == 0 || leftCount == node.triCount)
                {
                    nodes[nodeI] = node;
                    return;
                }

                //Create kids
                int leftKidI = nodesUsed++;
                int rightKidI = nodesUsed++;

                Node leftKid = nodes[leftKidI];
                Node rightKid = nodes[rightKidI];
                leftKid.leftStartI = node.leftStartI;
                leftKid.triCount = leftCount;
                rightKid.leftStartI = i;
                rightKid.triCount = node.triCount - leftCount;

                node.leftStartI = leftKidI;
                node.triCount = 0;
                nodes[nodeI] = node;

                Subdivide(ref leftKid, leftKidI);
                Subdivide(ref rightKid, rightKidI);
            }

            //Extended triangles to normal triangles
            for (int i = 0; i < triCount; i++)
            {
                tris[i] = new(eTris[i]);
            }
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

            for (int i = 0; i < subMeshCount; i++)
            {
                SubMeshDescriptor subMD = god.meshData.GetSubMesh(i);
                maxIndicesCount = Math.Max(maxIndicesCount, subMD.indexCount);
            }

            int triCount = god.meshData.vertexCount / 3;
            int triI = 0;
            NativeArray<int> indices = new(maxIndicesCount, Allocator.Temp);
            NativeArray<Vector3> vertics = new(triCount * 3, Allocator.Temp);
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
        #endregion BLASObject creation
    }

    internal unsafe struct BLASInstance
    {
        #region BLASInstance data

        internal BLASInstance(in BLASObject blasO, ref Matrix4x4 wToL, ref Matrix4x4 lToW)
        {
            nodesPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.nodes);
            nodesLenght = blasO.nodes.Length;
            trisPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.tris);
            trisLenght = blasO.tris.Length;

            this.wToL = wToL;

            Node node = blasO.nodes[0];
            Vector3 newMin = lToW.MultiplyPoint3x4(node.min);
            max = lToW.MultiplyPoint3x4(node.max);
            min = newMin;
            min = HelpMethods.Min(min, max);
            max = HelpMethods.Max(max, newMin);
        }

        private void* nodesPTR;
        private int nodesLenght;
        private void* trisPTR;
        private int trisLenght;

        private Matrix4x4 wToL;
        /// <summary>
        /// min pos of AABB containing all nodes in worldspace
        /// </summary>
        internal Vector3 min;
        /// <summary>
        /// max pos of AABB containing all nodes in worldspace
        /// </summary>
        internal Vector3 max;

        internal void SetBLASObject(in BLASObject blasO)
        {
            nodesPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.nodes);
            nodesLenght = blasO.nodes.Length;
            trisPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.tris);
            trisLenght = blasO.tris.Length;
        }

        internal void SetMatrix(ref Matrix4x4 wToL, ref Matrix4x4 lToW)
        {
            this.wToL = wToL;

            Node node = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                nodesPTR, nodesLenght, Allocator.None)[0];//Could use raw pointer

            Vector3 newMin = lToW.MultiplyPoint3x4(node.min);
            max = lToW.MultiplyPoint3x4(node.max);
            min = newMin;
            min = HelpMethods.Min(min, max);
            max = HelpMethods.Max(max, newMin);
        }

        #endregion BLASInstance data

        #region BLASInstance raycast

        internal bool Raycast(in Ray ray, out Hit hit)
        {
            Vector3 dirL = wToL.MultiplyVector(ray.direction);
            float disScale = dirL.magnitude;
            dirL.Normalize();//Is a normalized vector required?

            Vector3 orginL = wToL.MultiplyPoint3x4(ray.orgin);
            float hitDisL = float.MaxValue;
            int hitTriI = -1;

            NativeArray<Node> nodes = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                nodesPTR, nodesLenght, Allocator.None);
            NativeArray<Triangle> tris = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Triangle>(
                nodesPTR, nodesLenght, Allocator.None);

            IntersectBVH(0);

            //Get result
            if (hitTriI < 0)
            {
                hit = new();
                return false;
            }

            hitDisL /= disScale;//Makes it worldspace
            hit = new(tris[hitTriI], ray.orgin + (ray.direction * hitDisL), hitDisL);
            return true;

            void IntersectBVH(int nodeI)
            {
                Node node = nodes[nodeI];
                if (IntersectAABB(in node.min, in node.max) == false) return;
                if (node.triCount > 0)
                {
                    for (int i = 0; i < node.triCount; i++)
                    {
                        IntersectTri(i);
                    }
                }
                else
                {
                    IntersectBVH(node.leftStartI);
                    IntersectBVH(node.leftStartI + 1);
                }
            }

            bool IntersectAABB(in Vector3 min, in Vector3 max)
{
                float tx1 = (min.x - orginL.x) / dirL.x, tx2 = (max.x - orginL.x) / dirL.x;
                float tmin = Math.Min(tx1, tx2), tmax = Math.Max(tx1, tx2);
                float ty1 = (min.y - orginL.y) / dirL.y, ty2 = (max.y - orginL.y) / dirL.y;
                tmin = Math.Max(tmin, Math.Min(ty1, ty2)); tmax = Math.Min(tmax, Math.Max(ty1, ty2));
                float tz1 = (min.z - orginL.z) / dirL.z, tz2 = (max.z - orginL.z) / dirL.z;
                tmin = Math.Max(tmin, Math.Min(tz1, tz2)); tmax = Math.Min(tmax, Math.Max(tz1, tz2));
                return tmax >= tmin && tmin < hitDisL     && tmax > 0;
            }

            void IntersectTri(int triI)
            {
                Triangle tri = tris[triI];

                Vector3 edge1 = tri.v1 - tri.v0;
                Vector3 edge2 = tri.v2 - tri.v0;
                Vector3 h = Vector3.Cross(dirL, edge2);
                float a = Vector3.Dot(edge1, h);
                if (a > -0.0001f && a < 0.0001f) return;//Ray parallel to triangle
                float f = 1 / a;
                Vector3 s = orginL - tri.v0;
                float u = f * Vector3.Dot(s, h);
                if (u < 0 || u > 1) return;
                Vector3 q = Vector3.Cross(s, edge1);
                float v = f * Vector3.Dot(dirL, q);
                if (v < 0 || u + v > 1) return;
                float t = f * Vector3.Dot(edge2, q);
                if (t < 0.0001f || t > hitDisL) return;

                //We hit the triangle
                hitDisL = t;
                hitTriI = triI;
            }
        }

        #endregion BLASInstance raycast
    }
}
