using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using BLASObject = BLASBuilder.BLASObject;
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
        internal readonly NativeArray<BLASObject> blasObjects;
        internal readonly NativeArray<BLASInstance> blasInstances;
        internal readonly NativeArray<short> blasInstanceLocks;
        internal readonly NativeArray<Node> nodes;

        /// <summary>
        /// Input nativeArray MUST be Allocator.Persistent and never disposed outside this.Dispose()
        /// </summary>
        internal TLASScene(NativeArray<BLASObject> blasObjects, NativeArray<BLASInstance> blasInstances)
        {
            int blasInstanceCount = blasInstances.Length;
            int nodesUsed = 1;
            int nodeIndices = blasInstanceCount;

            this.blasObjects = blasObjects;
            this.blasInstances = blasInstances;
            blasInstanceLocks = new(blasInstanceCount, Allocator.Persistent);
            NativeArray<Node> nodes = this.nodes = new(blasInstanceCount + 1, Allocator.Persistent);
            NativeArray<int> nodeIndexs = new(blasInstanceCount, Allocator.Temp);

            for (int i = 0; i < blasInstanceCount; i++)
            {
                nodeIndexs[i] = nodesUsed;
                nodes[nodesUsed] = new(blasInstances[i], i);
                nodesUsed++;
            }

            //##########Fixme compile error, I currently dont fully understand this logic, read more about it later
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
