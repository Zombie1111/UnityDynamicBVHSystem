using Unity.Collections;
using UnityEngine;
using Unity.Burst;
using System;
using Unity.Collections.LowLevel.Unsafe;

namespace DyBVHSys_core
{
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

            public static int SizeOf()
            {
                return UnsafeUtility.SizeOf<Node>();
                //return (sizeof(float) * 6) + (sizeof(int) * 2);
            }
        }

        /// <summary>
        /// A BLASObject is a local BVH
        /// </summary>
        [BurstCompile]
        public readonly unsafe struct BLASObject
        {
            internal readonly Node* nodes;
            internal readonly int nodesLenght;
            internal readonly Triangle* tris;
            internal readonly int trisLenght;
            //internal readonly NativeArray<Node> nodes;
            //internal readonly NativeArray<Triangle> tris;

            #region BLASObject creation
            public BLASObject(NativeArray<Triangle.Extended> eTris)
            {
                int triCount = eTris.Length;
                var axisSortedTrisI = new NativeArray<int>(triCount, Allocator.Temp);//* 3 not needed since cant pre-sort??
                var leftMins = new NativeArray<Vector3>(triCount, Allocator.Temp);
                var leftMaxs = new NativeArray<Vector3>(triCount, Allocator.Temp);

                nodesLenght = triCount * 2 - 1;
                var nodes = this.nodes = (Node*)UnsafeUtility.Malloc(nodesLenght * Node.SizeOf(), UnsafeUtility.AlignOf<Node>(), Allocator.Persistent);
                int i;

                //Build BVH
                //Get root node
                int rootNodeI = 0;
                int nodesUsed = 1;
                Node rootNode = nodes[rootNodeI];

                rootNode.leftStartI = 0;
                rootNode.triCount = triCount;
                {
                    Vector3 nodeMin = Vector3.positiveInfinity;
                    Vector3 nodeMax = Vector3.negativeInfinity;

                    for (i = 0; i < triCount; i++)
                    {
                        Triangle.Extended tri = eTris[i];
                        nodeMin = HelpMethods.Min(nodeMin, tri.v0);
                        nodeMin = HelpMethods.Min(nodeMin, tri.v1);
                        nodeMin = HelpMethods.Min(nodeMin, tri.v2);
                        nodeMax = HelpMethods.Max(nodeMax, tri.v0);
                        nodeMax = HelpMethods.Max(nodeMax, tri.v1);
                        nodeMax = HelpMethods.Max(nodeMax, tri.v2);
                    }

                    rootNode.min = nodeMin;
                    rootNode.max = nodeMax;
                }

                //Recursively subdivide the root node
                Subdivide(ref rootNode, rootNodeI);

                void Subdivide(ref Node node, int nodeIdx)//https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
                {
                    //Stop splitting if too small
                    int count = node.triCount;

                    if (count <= 2)
                    {
                        nodes[nodeIdx] = node;
                        return;
                    }

                    //Split using SAH
                    //Get parent cost
                    int start = node.leftStartI;
                    Vector3 parentExtent = node.max - node.min;
                    float parentCost = count
                        * (parentExtent.x * parentExtent.y + parentExtent.y * parentExtent.z + parentExtent.z * parentExtent.x);

                    int bestAxis = -1;
                    float bestSplitPos = 0;
                    float bestCost = float.MaxValue;
                    Vector3 bestLeftMax = Vector3.zero;
                    Vector3 bestLeftMin = Vector3.zero;
                    Vector3 bestRightMax = Vector3.zero;
                    Vector3 bestRightMin = Vector3.zero;

                    for (int axis = 0; axis < 3; axis++)
                    {
                        //Sort triangles by center per axis. Ive tried pre sorting before any splitting but that fucks up the result, probably because of changed tri indexes when splitting.
                        //This is the main bottleneck
                        //int baseIdx = axis * triCount;
                        int baseIdx = 0;//No need to offset if I cant pre sort anyway

                        for (int i = 0; i < count; i++)//Fixlater: only need to sort the two axises other than the selected split one
                        {
                            axisSortedTrisI[baseIdx + i] = start + i;
                        }

                        #region Sort
                        QuickSort(baseIdx, count, axis);//https://code-maze.com/csharp-quicksort-algorithm/

                        void QuickSort(int baseIdx, int count, int axis)
                        {
                            //Insertion is faster for small arrays
                            if (count < 17)
                            {
                                InsertionSort(baseIdx, count, axis);
                                return;
                            }

                            int lo = baseIdx;
                            int hi = baseIdx + count - 1;
                            int mid = lo + ((hi - lo) >> 1);

                            float a = eTris[axisSortedTrisI[lo]].center[axis];
                            float b = eTris[axisSortedTrisI[mid]].center[axis];
                            float c = eTris[axisSortedTrisI[hi]].center[axis];
                            int pivotIdx;

                            if ((a < b && b < c) || (c < b && b < a)) pivotIdx = mid;
                            else if ((b < a && a < c) || (c < a && a < b)) pivotIdx = lo;
                            else pivotIdx = hi;

                            (axisSortedTrisI[pivotIdx], axisSortedTrisI[lo]) = (axisSortedTrisI[lo], axisSortedTrisI[pivotIdx]);

                            float pivotValue = eTris[axisSortedTrisI[lo]].center[axis];
                            int left = lo + 1;
                            int right = hi;

                            while (true)
                            {
                                while (left <= right && eTris[axisSortedTrisI[left]].center[axis] < pivotValue)
                                    left++;

                                while (right >= left && eTris[axisSortedTrisI[right]].center[axis] >= pivotValue)
                                    right--;

                                if (left > right)
                                    break;

                                (axisSortedTrisI[right], axisSortedTrisI[left]) = (axisSortedTrisI[left], axisSortedTrisI[right]);
                                left++;
                                right--;
                            }

                            (axisSortedTrisI[right], axisSortedTrisI[lo]) = (axisSortedTrisI[lo], axisSortedTrisI[right]);

                            int leftLen = right - baseIdx;
                            int rightLen = (baseIdx + count - 1) - (right + 1) + 1;

                            //Sort the smallest half first
                            if (leftLen < rightLen)
                            {
                                QuickSort(baseIdx, leftLen, axis);
                                QuickSort(right + 1, rightLen, axis);
                            }
                            else
                            {
                                QuickSort(right + 1, rightLen, axis);
                                QuickSort(baseIdx, leftLen, axis);
                            }
                        }

                        void InsertionSort(int baseIdx, int length, int axis)//https://code-maze.com/insertion-sort-csharp/
                        {
                            for (int i = 1; i < length; i++)
                            {
                                int keyIdx = axisSortedTrisI[baseIdx + i];
                                float keyVal = eTris[keyIdx].center[axis];
                                int j = i - 1;

                                while (j >= 0)
                                {
                                    int idxJ = axisSortedTrisI[baseIdx + j];
                                    float valJ = eTris[idxJ].center[axis];
                                    if (valJ > keyVal)
                                    {
                                        axisSortedTrisI[baseIdx + j + 1] = idxJ;
                                        j--;
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                axisSortedTrisI[baseIdx + j + 1] = keyIdx;
                            }
                        }
                        #endregion Sort

                        //Grow from left
                        {
                            Vector3 leftMin = Vector3.positiveInfinity;
                            Vector3 leftMax = Vector3.negativeInfinity;

                            for (int i = 0; i < count; i++)
                            {
                                int triIdx = axisSortedTrisI[baseIdx + i];
                                Triangle.Extended tri = eTris[triIdx];

                                leftMin = HelpMethods.Min(leftMin, tri.v0); leftMax = HelpMethods.Max(leftMax, tri.v0);
                                leftMin = HelpMethods.Min(leftMin, tri.v1); leftMax = HelpMethods.Max(leftMax, tri.v1);
                                leftMin = HelpMethods.Min(leftMin, tri.v2); leftMax = HelpMethods.Max(leftMax, tri.v2);

                                leftMaxs[i] = leftMax;
                                leftMins[i] = leftMin;
                            }
                        }

                        //Grow from right and get if best so far
                        {
                            Vector3 rightMin = Vector3.positiveInfinity;
                            Vector3 rightMax = Vector3.negativeInfinity;

                            for (int i = count - 1; i >= 0; i--)
                            {
                                int triIdx = axisSortedTrisI[baseIdx + i];
                                Triangle.Extended tri = eTris[triIdx];

                                rightMin = HelpMethods.Min(rightMin, tri.v0); rightMax = HelpMethods.Max(rightMax, tri.v0);
                                rightMin = HelpMethods.Min(rightMin, tri.v1); rightMax = HelpMethods.Max(rightMax, tri.v1);
                                rightMin = HelpMethods.Min(rightMin, tri.v2); rightMax = HelpMethods.Max(rightMax, tri.v2);

                                if (i == 0) break;
                                Vector3 leftMax = leftMaxs[i - 1];
                                Vector3 leftMin = leftMins[i - 1];
                                float cost = i * HelpMethods.Area(leftMax - leftMin) + (count - i) * HelpMethods.Area(rightMax - rightMin);

                                if (cost < bestCost)
                                {
                                    bestLeftMax = leftMax;
                                    bestLeftMin = leftMin;
                                    bestRightMax = rightMax;
                                    bestRightMin = rightMin;

                                    bestCost = cost;
                                    bestAxis = axis;
                                    bestSplitPos = 0.5f * (eTris[axisSortedTrisI[baseIdx + i - 1]].center[axis] + tri.center[axis]);
                                }
                            }
                        }
                    }

                    //Stop splitting if not better than parent
                    if (bestCost >= parentCost)
                    {
                        nodes[nodeIdx] = node;
                        return;
                    }

                    //Split tris by best axis and pos
                    int splitStartI = start;
                    int splitEndI = start + count - 1;

                    while (splitStartI <= splitEndI)
                    {
                        if (eTris[splitStartI].center[bestAxis] < bestSplitPos)
                        {
                            splitStartI++;
                        }
                        else
                        {
                            (eTris[splitStartI], eTris[splitEndI]) = (eTris[splitEndI], eTris[splitStartI]);
                            splitEndI--;
                        }
                    }

                    //Stop splitting if all tris is on one side
                    int leftCountFinal = splitStartI - start;
                    if (leftCountFinal == 0 || leftCountFinal == count)
                    {
                        nodes[nodeIdx] = node;
                        return;
                    }

                    //Make more kids
                    int leftChildIdx = nodesUsed++;
                    int rightChildIdx = nodesUsed++;

                    Node leftChild = new()
                    {
                        leftStartI = start,
                        triCount = leftCountFinal,
                        min = bestLeftMin,
                        max = bestLeftMax
                    };

                    Node rightChild = new()
                    {
                        leftStartI = start + leftCountFinal,
                        triCount = count - leftCountFinal,
                        min = bestRightMin,
                        max = bestRightMax
                    };

                    //Make input node a branch
                    node.leftStartI = -1;
                    node.triCount = 0;
                    node.leftStartI = leftChildIdx;
                    nodes[nodeIdx] = node;

                    //Recursively subdivide children
                    nodes[leftChildIdx] = leftChild;
                    Subdivide(ref leftChild, leftChildIdx);

                    nodes[rightChildIdx] = rightChild;
                    Subdivide(ref rightChild, rightChildIdx);
                }

                //Extended triangles to normal triangles
                trisLenght = triCount;
                tris = (Triangle*)UnsafeUtility.Malloc(trisLenght * Triangle.SizeOf(), UnsafeUtility.AlignOf<Triangle>(), Allocator.Persistent);

                for (i = 0; i < triCount; i++)
                {
                    tris[i] = new(eTris[i]);
                }
            }

            /// <summary>
            /// There is no check for if its has already been diposed!
            /// </summary>
            public void Dispose()
            {
                if (trisLenght > 0) UnsafeUtility.Free(tris, Allocator.Persistent);
                if (nodesLenght > 0) UnsafeUtility.Free(nodes, Allocator.Persistent);
            }

            #endregion BLASObject creation
        }

        public unsafe struct BLASInstance
        {
            #region BLASInstance data

            public BLASInstance(in BLASObject blasO, ref Matrix4x4 wToL, ref Matrix4x4 lToW)
            {
                nodes = blasO.nodes;
                nodesLenght = blasO.nodesLenght;
                tris = blasO.tris;
                trisLenght = blasO.trisLenght;
                //nodesPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.nodes);
                //nodesLenght = blasO.nodes.Length;
                //trisPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.tris);
                //trisLenght = blasO.tris.Length;

                this.wToL = wToL;

                Node node = blasO.nodes[0];
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

            private Node* nodes;
            /// <summary>
            /// 0 if unused, negative if disabled
            /// </summary>
            internal int nodesLenght;
            private Triangle* tris;
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
                nodes = blasO.nodes;
                nodesLenght = blasO.nodesLenght;
                tris = blasO.tris;
                trisLenght = blasO.trisLenght;

                //nodesPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.nodes);
                //nodesLenght = blasO.nodes.Length;
                //trisPTR = NativeArrayUnsafeUtility.GetUnsafeReadOnlyPtr(blasO.tris);
                //trisLenght = blasO.tris.Length;
            }

            internal void SetMatrix(ref Matrix4x4 wToL, ref Matrix4x4 lToW)
            {
                this.wToL = wToL;

                Node node = nodes[0];
                //Node node = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                //    nodesPTR, nodesLenght, Allocator.None)[0];//Could use raw pointer

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

                var nodes = this.nodes;
                var tris = this.tris;

                //NativeArray<Node> nodes = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                //    nodesPTR, nodesLenght, Allocator.None);
                //NativeArray<Triangle> tris = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Triangle>(
                //    trisPTR, trisLenght, Allocator.None);

                //#if ENABLE_UNITY_COLLECTIONS_CHECKS
                //            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref nodes, AtomicSafetyHandle.
                //            GetTempUnsafePtrSliceHandle
                //            ());
                //#endif
                //
                //#if ENABLE_UNITY_COLLECTIONS_CHECKS
                //            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref tris, AtomicSafetyHandle.
                //            GetTempUnsafePtrSliceHandle
                //            ());
                //#endif

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
                var nodes = this.nodes;
                //NativeArray<Node> nodes = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<Node>(
                //    nodesPTR, nodesLenght, Allocator.None);

                //#if ENABLE_UNITY_COLLECTIONS_CHECKS
                //            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref nodes, AtomicSafetyHandle.
                //            GetTempUnsafePtrSliceHandle
                //            ());
                //#endif

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
}

