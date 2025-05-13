using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BVHNode : MonoBehaviour
{
    //A node in the BVH tree
    internal struct Node
    {
        internal Vector3 min;
        internal Vector3 max;
        internal uint leftKid;
        internal uint rightKid;
        internal int objI;//UShort probably enough
        internal uint triStartI;
        internal uint triCount;
    }
}
