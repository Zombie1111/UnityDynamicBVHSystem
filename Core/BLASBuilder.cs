using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Burst;
using System;
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
    /// A BLASObject is a local BVH
    /// </summary>
    [BurstCompile]
    public readonly struct BLASObject
    {
        internal readonly NativeArray<Node> nodes;
        internal readonly NativeArray<Triangle> tris;

        #region BLASObject creation
        public BLASObject(NativeArray<Triangle.Extended> eTris)
        {
            int triCount = eTris.Length;
            NativeArray<Node> nodes = this.nodes = new(triCount * 2 - 1, Allocator.Persistent);

            ////Sort tris per axis
            //NativeArray<int> axisTriIs = new(triCount * 3, Allocator.Temp);
            //eTris.Sort((a, b) => a.center.x.CompareTo(b.center.x));

            //Build BVH
            int rootNodeI = 0;
            int nodesUsed = 1;
            Node rootNode = nodes[rootNodeI];

            rootNode.leftStartI = 0;
            rootNode.triCount = triCount;

            Vector3 gizS = Vector3.one * 0.01f;

            HelpMethods.Debug_toggleTimer();
            Subdivide(ref rootNode, rootNodeI);
            HelpMethods.Debug_toggleTimer();//1850ms with SAH wtf?

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

                Vector3 e = node.max - node.min; // extent of parent
                float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
                float parentCost = node.triCount * parentArea;

                //Terminate recursion
                if (node.triCount <= 2)
                {
                    nodes[nodeI] = node;
                    return;
                }

                //Determine split axis and position
                //SAH
                int splitAxis = -1;
                float splitPos = 0;
                float bestCost = float.MaxValue;

                for (int triOffset = 0; triOffset < node.triCount; triOffset++)
                {
                    for (int axis = 0; axis < 3; axis++)
                    {
                        Triangle.Extended testTri = eTris[node.leftStartI + triOffset];
                        float testTriPos = testTri.center[axis];
                
                        #region Get sah cost
                        Vector3 lMin = Vector3.positiveInfinity, lMax = Vector3.negativeInfinity;
                        Vector3 rMin = Vector3.positiveInfinity, rMax = Vector3.negativeInfinity;
                        int lCount = 0;
                        int rCount = 0;
                
                        for (int triI = 0; triI < node.triCount; triI++)
                        {
                            Triangle.Extended tri = eTris[node.leftStartI + triI];
                
                            if (tri.center[axis] < testTriPos)
                            {
                                lCount++;
                                lMin = HelpMethods.Min(lMin, tri.v0); lMax = HelpMethods.Max(lMax, tri.v0);
                                lMin = HelpMethods.Min(lMin, tri.v1); lMax = HelpMethods.Max(lMax, tri.v1);
                                lMin = HelpMethods.Min(lMin, tri.v2); lMax = HelpMethods.Max(lMax, tri.v2);
                            }
                            else
                            {
                                rCount++;
                                rMin = HelpMethods.Min(rMin, tri.v0); rMax = HelpMethods.Max(rMax, tri.v0);
                                rMin = HelpMethods.Min(rMin, tri.v1); rMax = HelpMethods.Max(rMax, tri.v1);
                                rMin = HelpMethods.Min(rMin, tri.v2); rMax = HelpMethods.Max(rMax, tri.v2);
                            }
                        }
                
                        float cost = lCount * HelpMethods.Area(lMax - lMin) + rCount * HelpMethods.Area(rMax - rMin);
                        if (cost <= 0) cost = float.MaxValue;
                        #endregion Get sah cost
                
                        if (cost < bestCost)
                        {
                            splitPos = testTriPos;
                            splitAxis = axis;
                            bestCost = cost;
                        }
                    }
                }
                
                if (bestCost >= parentCost)
                {
                    return;
                }

                ////Longest axis
                //Vector3 extent = node.max - node.min;
                //int splitAxis = 0;
                //if (extent.y > extent.x) splitAxis = 1;
                //if (extent.z > extent[splitAxis]) splitAxis = 2;
                //float splitPos = node.min[splitAxis] + (extent[splitAxis] * 0.5f);

                //Split triangles
                int i = node.leftStartI;
                int j = i + node.triCount - 1;

                while (i <= j)
                {
                    if (eTris[i].center[splitAxis] < splitPos)
                    {
                        i++;
                        continue;
                    }

                    (eTris[j], eTris[i]) = (eTris[i], eTris[j]);
                    j--;
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
            tris = new(triCount, Allocator.Persistent);

            for (int i = 0; i < triCount; i++)
            {
                tris[i] = new(eTris[i]);
            }
        }

        #endregion BLASObject creation
    }

    public unsafe struct BLASInstance
    {
        #region BLASInstance data

        public BLASInstance(in BLASObject blasO, ref Matrix4x4 wToL, ref Matrix4x4 lToW)
        {
            nodesPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.nodes);
            nodesLenght = blasO.nodes.Length;
            trisPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.tris);
            trisLenght = blasO.tris.Length;

            this.wToL = wToL;

            Node node = blasO.nodes[0];
            Vector3 localExtents = 0.5f * (node.max - node.min);
            Vector3 worldCenter = lToW.MultiplyPoint3x4(0.5f * (node.min + node.max));
            Vector3 worldExtents = new            (
              Math.Abs(lToW.m00) * localExtents.x + Math.Abs(lToW.m01) * localExtents.y + Math.Abs(lToW.m02) * localExtents.z,
              Math.Abs(lToW.m10) * localExtents.x + Math.Abs(lToW.m11) * localExtents.y + Math.Abs(lToW.m12) * localExtents.z,
              Math.Abs(lToW.m20) * localExtents.x + Math.Abs(lToW.m21) * localExtents.y + Math.Abs(lToW.m22) * localExtents.z
            );
            
            min = worldCenter - worldExtents;
            max = worldCenter + worldExtents;
        }

        private void* nodesPTR;
        /// <summary>
        /// 0 if unused, negative if disabled
        /// </summary>
        internal int nodesLenght;
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

            Vector3 localExtents = 0.5f * (node.max - node.min);
            Vector3 worldCenter = lToW.MultiplyPoint3x4(0.5f * (node.min + node.max));
            Vector3 worldExtents = new(
              Math.Abs(lToW.m00) * localExtents.x + Math.Abs(lToW.m01) * localExtents.y + Math.Abs(lToW.m02) * localExtents.z,
              Math.Abs(lToW.m10) * localExtents.x + Math.Abs(lToW.m11) * localExtents.y + Math.Abs(lToW.m12) * localExtents.z,
              Math.Abs(lToW.m20) * localExtents.x + Math.Abs(lToW.m21) * localExtents.y + Math.Abs(lToW.m22) * localExtents.z
            );

            min = worldCenter - worldExtents;
            max = worldCenter + worldExtents;
        }

        #endregion BLASInstance data

        #region BLASInstance raycast

        [BurstCompile]
        internal bool Raycast(in Ray ray, out Hit hit)
        {
            Vector3 dirL = wToL.MultiplyVector(ray.direction);
            float disScale = dirL.magnitude;
            dirL.Normalize();//Is a normalized vector required?

            Vector3 orginL = wToL.MultiplyPoint3x4(ray.orgin);
            float hitDisL = ray.maxDistance * disScale;
            int hitTriI = -1;

            NativeArray<Node> nodes = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                nodesPTR, nodesLenght, Allocator.None);
            NativeArray<Triangle> tris = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Triangle>(
                trisPTR, trisLenght, Allocator.None);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref nodes, AtomicSafetyHandle.
            GetTempUnsafePtrSliceHandle
            ());
#endif

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref tris, AtomicSafetyHandle.
            GetTempUnsafePtrSliceHandle
            ());
#endif

            IntersectBVH(0);

            //Get result
            if (hitTriI < 0)
            {
                hit = new(ray);
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
                        IntersectTri(node.leftStartI + i);
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
                return tmax >= tmin && tmin < hitDisL && tmax > 0;
            }

            void IntersectTri(int triI)
            {
                Triangle tri = tris[triI];

                //Gizmos.DrawCube(tri.v0, Vector3.one * 0.01f);
                //Gizmos.DrawCube(tri.v1, Vector3.one * 0.01f);
                //Gizmos.DrawCube(tri.v2, Vector3.one * 0.01f);

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


        #region Debug

        public void Debug_drawGismos()
        {
            Matrix4x4 lToW = wToL.inverse;
            NativeArray<Node> nodes = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                nodesPTR, nodesLenght, Allocator.None);

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref nodes, AtomicSafetyHandle.
            GetTempUnsafePtrSliceHandle
            ());
#endif

            float maxDepth = Mathf.Log(nodesLenght, 2);
            DrawNode(nodes[0], 0);

            void DrawNode(Node node, int depth)
            {
                if (node.triCount > 0) return;

                //Debug.Log("Box " + depth + " " + maxDepth);

                //Draw
                Vector3 localExtents = 0.5f * (node.max - node.min);
                Vector3 worldCenter = lToW.MultiplyPoint3x4(0.5f * (node.min + node.max));
                Vector3 worldExtents = new(
                  Math.Abs(lToW.m00) * localExtents.x + Math.Abs(lToW.m01) * localExtents.y + Math.Abs(lToW.m02) * localExtents.z,
                  Math.Abs(lToW.m10) * localExtents.x + Math.Abs(lToW.m11) * localExtents.y + Math.Abs(lToW.m12) * localExtents.z,
                  Math.Abs(lToW.m20) * localExtents.x + Math.Abs(lToW.m21) * localExtents.y + Math.Abs(lToW.m22) * localExtents.z
                );

                Gizmos.color = Color.Lerp(Color.white, Color.red, depth / maxDepth);
                Gizmos.DrawWireCube(worldCenter, worldExtents * 2);

                //Go deeper
                depth++;
                DrawNode(nodes[node.leftStartI], depth);
                DrawNode(nodes[node.leftStartI + 1], depth);
            }
        }

        #endregion Debug
    }
}
