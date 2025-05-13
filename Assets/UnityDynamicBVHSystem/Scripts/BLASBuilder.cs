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

public class BLASBuilder : MonoBehaviour
{
    //Creates BVHObjects
    //A node in the BVH tree
    [BurstCompile]
    internal struct Node
    {
        internal Vector3 min;
        internal Vector3 max;
        internal int leftKidI;//Right kid is always leftKid + 1
        internal int triStartI;
        internal int triCount;
    }

    [BurstCompile]
    internal readonly struct Triangle
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]//Works on constructors?
        internal Triangle(in Extended extendedTri)
        {
            v0 = extendedTri.v0;
            v1 = extendedTri.v1;
            v2 = extendedTri.v2;
            matI = extendedTri.matID;
        }

        internal readonly Vector3 v0;
        internal readonly Vector3 v1;
        internal readonly Vector3 v2;
        internal readonly short matI;//The index of the material this tri uses

        internal readonly struct Extended
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]//Works on constructors?
            internal Extended(Vector3 ver0, Vector3 ver1, Vector3 ver2, short newMatI)
            {
                v0 = ver0;
                v1 = ver1;
                v2 = ver2;
                center = (ver0 + ver1 + ver2) * 0.3333f;
                matID = newMatI;
            }

            internal readonly Vector3 v0;
            internal readonly Vector3 v1;
            internal readonly Vector3 v2;
            internal readonly Vector3 center;//Only needed durring building
            internal readonly short matID;//The id of the material this tri uses
        }
    }

    [BurstCompile]
    internal readonly unsafe struct ObjectData
    {
        internal ObjectData(Collider col, MeshFilter mf)
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

    [BurstCompile]
    internal readonly struct BLASObject
    {
        internal readonly NativeArray<Node> nodes;
        internal readonly NativeArray<Triangle> tris;

        internal unsafe BLASObject(in ObjectData od)
        {
            if (od.type != ShapeType.ConcaveMesh)
            {
                throw new Exception(od.type + " not implemented!");
            }

            //Get mesh data and allocate native arrays
            int subMeshCount = od.meshData.subMeshCount;
            int maxIndicesCount = 0;

            for (int i = 0; i < subMeshCount; i++)
            {
                SubMeshDescriptor subMD = od.meshData.GetSubMesh(i);
                maxIndicesCount = Math.Max(maxIndicesCount, subMD.indexCount);
            }

            int triCount = od.meshData.vertexCount / 3;
            int triI = 0;
            NativeArray<int> indices = new(maxIndicesCount, Allocator.Temp);
            NativeArray<Vector3> vertics = new(triCount * 3, Allocator.Temp);
            NativeArray<Triangle.Extended> eTris = new(triCount, Allocator.Temp);

            od.meshData.GetVertices(vertics);
            NativeArray<Node> nodes = this.nodes = new(triCount * 2 - 1, Allocator.Persistent);
            tris = new(triCount, Allocator.Persistent);

            //Get triangles per submesh
            fixed (FixedBytes16* subMeshMatI_ptr = &od.subMeshMatI)
            {
                short* subMeshMatI_shorts = (short*)subMeshMatI_ptr;

                for (int i = 0; i < subMeshCount; i++)
                {
                    SubMeshDescriptor subMD = od.meshData.GetSubMesh(i);
                    int subIndiceCount = subMD.indexCount;
                    od.meshData.GetIndices(indices, i, true);

                    for (int subIndiceI = 0; subIndiceI < subIndiceCount; subIndiceI += 3)
                    {
                        eTris[triI] = new(vertics[indices[subIndiceI]], vertics[indices[subIndiceI + 1]], vertics[indices[subIndiceI + 2]], subMeshMatI_shorts[i]);
                        triI++;
                    }
                }
            }

            //Build BVH
            int rootNodeI = 0;
            int nodesUsed = 1;
            Node rootNode = nodes[rootNodeI];

            rootNode.leftKidI = 0;
            rootNode.triStartI = 0;
            rootNode.triCount = triCount;

            Subdivide(ref rootNode, rootNodeI);
            void Subdivide(ref Node node, int nodeI)
            {
                //Update bounds
                node.min = new Vector3(1e30f, 1e30f, 1e30f);
                node.max = new Vector3(-1e30f, -1e30f, -1e30f);

                for (int first = node.triStartI, ii = 0; ii < node.triCount; ii++)
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
                int i = node.triStartI;
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
                int leftCount = i - node.triStartI;
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
                leftKid.triStartI = node.triStartI;
                leftKid.triCount = leftCount;
                rightKid.triStartI = i;
                rightKid.triCount = node.triCount - leftCount;

                node.leftKidI = leftKidI;
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
    }
}
