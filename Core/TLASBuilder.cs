using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using BLASInstance = BLASBuilder.BLASInstance;
using Unity.Collections;

public static class TLASBuilder
{
    internal readonly struct Node
    {
        internal Node(BLASInstance blasInstance, int blasInstanceI)
        {
            min = blasInstance.min;
            max = blasInstance.max;
            this.blasInstanceI = blasInstanceI;
            leftKidI = 0;
            rightKidI = 0;
        }

        internal readonly Vector3 min;
        internal readonly Vector3 max;
        internal readonly ushort leftKidI;
        internal readonly ushort rightKidI;
        internal readonly int blasInstanceI;
    }

    public readonly struct TLASScene
    {
        internal readonly NativeArray<BLASInstance> blasInstances;
        internal readonly NativeArray<short> blasInstanceLocks;
        internal readonly NativeArray<Node> nodes;

        /// <summary>
        /// Input nativeArrays MUST be Allocator.Persistent and never disposed outside this.Dispose()
        /// </summary>
        internal TLASScene(NativeArray<BLASInstance> blasInstances, NativeArray<short> blasInstanceLocks)
        {
            //Without refitting, we would simply need to lock blasInstances index while getting it from array. (Because the BLAS nodes will never change)
            //With refitting we would need to lock the blasInstance durring the tracing. Because the nodex + tris may change.

            int blasInstanceCount = blasInstances.Length;
            int nodesUsed = 1;
            int nodeIndices = blasInstanceCount;

            this.blasInstances = blasInstances;
            this.blasInstanceLocks = blasInstanceLocks;
            NativeArray<Node> nodes = this.nodes = new(blasInstanceCount + 1, Allocator.Persistent);
            NativeArray<int> nodeIndexs = new(blasInstanceCount, Allocator.Temp);

            for (int i = 0; i < blasInstanceCount; i++)
            {
                nodeIndexs[i] = nodesUsed;
                nodes[nodesUsed] = new(blasInstances[i], i);
                nodesUsed++;
            }

            ////##########Fixme compile error, I currently dont fully understand this logic, read more about it later
            //int A = 0, B = FindBestMatch(nodeIndices, A);
            //while (nodeIndices > 1)
            //{
            //    int C = FindBestMatch(nodeIndices, B);
            //
            //    if (A == C)
            //    {
            //        int nodeIdxA = nodeIndexs[A], nodeIdxB = nodeIndexs[B];
            //        Node nodeA = nodes[nodeIdxA];
            //        Node nodeB = nodes[nodeIdxB];
            //        Node newNode = nodes[nodesUsed];
            //        newNode.leftKidI = nodeIdxA;
            //        newNode.rightKidI = nodeIdxB;
            //        newNode.min = fminf(nodeA.min, nodeB.min);
            //        newNode.max = fmaxf(nodeA.max, nodeB.max);
            //        nodeIndexs[A] = nodesUsed++;
            //        nodeIndexs[B] = nodeIndexs[nodeIndices - 1];
            //        B = FindBestMatch(--nodeIndices, A);
            //    }
            //    else
            //    {
            //        A = B; B = C;
            //    }
            //}
            //
            //nodes[0] = nodes[nodeIndexs[A]];

            int FindBestMatch(int N, int A)
            {
                float smallest = 1e30f;
                int bestB = -1;

                for (int B = 0; B < N; B++) if (B != A)
                {
                    Node nodeA = nodes[nodeIndexs[A]];
                    Node nodeB = nodes[nodeIndexs[B]];

                    Vector3 bmax = HelpMethods.Max(nodeA.max, nodeB.max);
                    Vector3 bmin = HelpMethods.Min(nodeA.min, nodeB.min);
                    Vector3 e = bmax - bmin;
                    float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;

                    if (surfaceArea < smallest)
                    {
                        smallest = surfaceArea; bestB = B;
                    }
                }

                return bestB;
            }
        }
    }
}
