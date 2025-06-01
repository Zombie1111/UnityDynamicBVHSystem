using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace DyBVHSys_core
{
    public static class Globals
    {
        public const bool _registerAllCollidersOnLoad = true;

        internal const int _maxRegisteredBlasInstances = 8192;
        internal const int _maxRegisteredBlasObjects = 1024;
    }

    [BurstCompile]
    public readonly struct Triangle
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]//Works on constructors?
        internal Triangle(in Extended extendedTri)
        {
            v0 = extendedTri.v0;
            v1 = extendedTri.v1;
            v2 = extendedTri.v2;
            matI = extendedTri.matID;
        }

        internal static int SizeOf()
        {
            return UnsafeUtility.SizeOf<Triangle>();
            //return (sizeof(float) * 9) + sizeof(short);
        }

        public readonly Vector3 v0;
        public readonly Vector3 v1;
        public readonly Vector3 v2;
        /// <summary>
        /// The index of the material this tri uses
        /// </summary>
        public readonly short matI;

        public readonly struct Extended
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]//Works on constructors?
            public Extended(Vector3 ver0, Vector3 ver1, Vector3 ver2, short newMatI)
            {
                v0 = ver0;
                v1 = ver1;
                v2 = ver2;
                center = (ver0 + ver1 + ver2) / 3.0f;
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
    public readonly struct Hit
    {
        internal Hit(in Triangle tri, Vector3 pos, float dis)
        {
            this.tri = tri;
            this.pos = pos;
            this.dis = dis;
        }

        internal Hit(in Ray ray)
        {
            this.tri = new();
            this.pos = ray.orgin + (ray.direction * ray.maxDistance);
            this.dis = ray.maxDistance;
        }

        public readonly Triangle tri;
        public readonly Vector3 pos;
        public readonly float dis;
    }


    [BurstCompile]
    public readonly struct Ray
    {
        private const float defaultRayMaxDistance = 100.0f;

        /// <summary>
        /// Direction must always be normalized
        /// </summary>
        public Ray(in Vector3 orgin, in Vector3 direction, float maxDistance = defaultRayMaxDistance)
        {
            this.orgin = orgin;
            this.direction = direction;
            this.maxDistance = maxDistance;
        }

        public readonly Vector3 orgin;
        /// <summary>
        /// Always normalized
        /// </summary>
        public readonly Vector3 direction;

        public readonly float maxDistance;
    }
}




