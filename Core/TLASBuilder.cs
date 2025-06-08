using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using BLASInstance = DyBVHSys_core.BLASBuilder.BLASInstance;
using Unity.Collections;
using System;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

namespace DyBVHSys_core
{
    public static class TLASBuilder
    {
        private readonly struct Node
        {
            internal readonly Vector3 min;
            internal readonly short leftBlasI;
            internal readonly short rightBlasI;
            internal readonly Vector3 max;
            internal readonly int blasI;

            internal bool IsLeaf() { return leftBlasI == 0 || rightBlasI == 0; }

            internal Node(in Vector3 min, in Vector3 max, int blasI, short leftBlasI, short rightBlasI)
            {
                this.min = min;
                this.leftBlasI = leftBlasI;
                this.rightBlasI = rightBlasI;
                this.max = max;
                this.blasI = blasI;
            }

            internal static int SizeOf()
            {
                return UnsafeUtility.SizeOf<Node>();
                //return (sizeof(float) * 6) + (sizeof(short) * 2) + sizeof(int);
            }
        }

        [BurstCompile]
        public readonly unsafe struct TLASScene
        {
            #region TlasScene build
            private readonly Node* nodes;
            private readonly int nodesLenght;
            private readonly BLASInstance* blasInstances;
            private readonly int* blasInstanceLocks;

            /// <summary>
            /// Input nativeArrays MUST be Allocator.Persistent and never disposed unless the TLASScene is disposed simeultaneously
            /// </summary>
            public TLASScene(NativeArray<BLASInstance> blasInstances, NativeArray<int> blasInstanceLocks, int activeBlasCount)
            {
                nodesLenght = activeBlasCount * 2;
                var nodes = this.nodes = (Node*)UnsafeUtility.Malloc(nodesLenght * Node.SizeOf(), UnsafeUtility.AlignOf<Node>(), Allocator.Persistent);
                this.blasInstances = (BLASInstance*)blasInstances.GetUnsafePtr();
                this.blasInstanceLocks = (int*)blasInstances.GetUnsafePtr();

                //Setup tlas nodes
                NativeArray<short> nodeIndexs = new(activeBlasCount, Allocator.Temp);//nodeIdx
                int blasCount = blasInstances.Length;
                short nodesUsed = 1;

                for (int i = 0; i < blasCount; i++)
                {
                    BLASInstance blas = blasInstances[i];
                    if (blas.nodesLenght < 1)
                    {
                        continue;//Inactive
                    }

                    nodeIndexs[nodesUsed - 1] = nodesUsed;
                    nodes[nodesUsed] = new(blas.min, blas.max, i, 0, 0);
                    nodesUsed++;
                }

                //Agglomerate clustering
                int nodesLeft = activeBlasCount;//nodeIndices 
                int a = 0;
                int b = FindBestMatch(a);//https://jacco.ompf2.com/2022/05/13/how-to-build-a-bvh-part-6-all-together-now/

                while (nodesLeft > 1)
                {
                    int c = FindBestMatch(b);
                    if (a == c)
                    {
                        short nodeIndexsA = nodeIndexs[a];
                        short nodeIndexsB = nodeIndexs[b];
                        Node nodeA = nodes[nodeIndexsA];
                        Node nodeB = nodes[nodeIndexsB];

                        nodes[nodesUsed] = new(HelpMethods.Min(nodeA.min, nodeB.min), HelpMethods.Max(nodeA.max, nodeB.max), -1, nodeIndexsA, nodeIndexsB);
                        nodeIndexs[a] = nodesUsed++;
                        nodesLeft--;
                        nodeIndexs[b] = nodeIndexs[nodesLeft];
                        b = FindBestMatch(a);
                    }
                    else
                    {
                        a = b;
                        b = c;
                    }
                }

                nodes[0] = nodes[nodeIndexs[a]];

                int FindBestMatch(int a)
                {
                    Node nodeA = nodes[nodeIndexs[a]];
                    float smallest = float.MaxValue;
                    int bestB = -1;

                    for (int b = 0; b < nodesLeft; b++)
                    {
                        if (b == a) continue;

                        Node nodeB = nodes[nodeIndexs[b]];
                        Vector3 bMax = HelpMethods.Max(nodeA.max, nodeB.max);
                        Vector3 bMin = HelpMethods.Min(nodeA.min, nodeB.min);
                        Vector3 e = bMax - bMin;
                        float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;

                        if (surfaceArea > smallest) continue;
                        smallest = surfaceArea;
                        bestB = b;
                    }

                    return bestB;
                }
            }

            /// <summary>
            /// blasInstances and blasInstanceLocks are not disposed, no checks if its already disposed
            /// </summary>
            public void Dispose()
            {
                if (nodesLenght > 0) UnsafeUtility.Free(nodes, Allocator.Persistent);
            }

            public void GetBlasInstance(int blasI, out BLASInstance blas)
            {
                blas = blasInstances[blasI];//Fixlater: lock
            }

            #endregion TlasScene build

            #region TlasScene raycast

            [BurstCompile]
            public void Raycast(in Ray ray, out Hit hit)
            {
                Node node = nodes[0];
                float bestHitDis = int.MaxValue;
                NativeArray<int> stack = new(64, Allocator.Temp);
                int stackI = 0;
                Vector3 orgin = ray.orgin;
                Vector3 dir = ray.direction;
                hit = new(ray);

                //GetBlasInstance(0, out BLASInstance blas);
                //if (blas.Raycast(ray, out Hit newHit) == true)
                //{
                //    hit = newHit;
                //}




                while (true)
                {
                    if (node.IsLeaf() == true)
                    {
                        GetBlasInstance(node.blasI, out BLASInstance blas);
                        if (blas.Raycast(ray, out Hit newHit) == true && newHit.dis < bestHitDis)
                        {
                            bestHitDis = newHit.dis;
                            hit = newHit;
                        }

                        if (stackI == 0) break;
                        node = nodes[stack[--stackI]];
                        continue;
                    }

                    Node child1 = nodes[node.leftBlasI];
                    Node child2 = nodes[node.rightBlasI];
                    float dist1 = IntersectAABB(child1.min, child1.max);
                    float dist2 = IntersectAABB(child2.min, child2.max);

                    //2 is main
                    if (dist1 > dist2)
                    {
                        if (dist2 == float.MaxValue)
                        {
                            if (stackI == 0) break;
                            node = nodes[stack[--stackI]];
                            continue;
                        }

                        if (dist1 != float.MaxValue) stack[stackI++] = node.leftBlasI;
                        node = child2;
                        continue;
                    }

                    //1 is main
                    if (dist1 == float.MaxValue)
                    {
                        if (stackI == 0) break;
                        node = nodes[stack[--stackI]];
                        continue;
                    }

                    if (dist2 != float.MaxValue) stack[stackI++] = node.rightBlasI;
                    node = child1;
                }

                float IntersectAABB(in Vector3 min, in Vector3 max)
                {
                    float tx1 = (min.x - orgin.x) / dir.x, tx2 = (max.x - orgin.x) / dir.x;
                    float tmin = Math.Min(tx1, tx2), tmax = Math.Max(tx1, tx2);
                    float ty1 = (min.y - orgin.y) / dir.y, ty2 = (max.y - orgin.y) / dir.y;
                    tmin = Math.Max(tmin, Math.Min(ty1, ty2)); tmax = Math.Min(tmax, Math.Max(ty1, ty2));
                    float tz1 = (min.z - orgin.z) / dir.z, tz2 = (max.z - orgin.z) / dir.z;
                    tmin = Math.Max(tmin, Math.Min(tz1, tz2)); tmax = Math.Min(tmax, Math.Max(tz1, tz2));

                    //return tmax >= tmin && tmin < hitDisL && tmax > 0;
                    return tmax >= tmin && tmin < bestHitDis && tmax > 0 ? tmin : float.MaxValue;
                }
            }

            #endregion TlasScene raycast

            #region Debug

            public void Debug_drawGizmos()
            {

                for (int i = 0; i < nodesLenght; i++)
                {
                    var node = nodes[i];

                    Vector3 e = node.max - node.min;
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawWireCube(node.min + (e * 0.5f), e);

                    if (node.IsLeaf() == true)
                    {
                        GetBlasInstance(node.blasI, out BLASInstance blas);
                        blas.Debug_drawGismos();
                    }
                    else
                    {
                        //GetBlasInstance(node.leftBlasI, out BLASInstance blas);
                        //blas.Debug_drawGismos();
                        //GetBlasInstance(node.rightBlasI, out blas);
                        //blas.Debug_drawGismos();
                    }
                }
            }

            #endregion Debug
        }
    }
}

