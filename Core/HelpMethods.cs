using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

public static class HelpMethods
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Min(Vector3 a, Vector3 b)
    {
        return new Vector3(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3 Max(Vector3 a, Vector3 b)
    {
        return new Vector3(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Area(Vector3 e)
    {
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }

    /// <summary>
    /// One should be null
    /// </summary>
    internal static int GetShapeIDFrom(Collider col, MeshFilter mf)
    {
        Vector3 offset;
        Vector3 shape;

        if (mf != null)
        {
            Mesh mesh = mf.sharedMesh;
            offset = new Vector3(mesh.vertexCount, 0, 0);
            shape = mesh.bounds.extents;
        }
        else if (col is MeshCollider meshC)
        {
            Mesh mesh = meshC.sharedMesh;
            offset = new Vector3(mesh.vertexCount, 0, 0);
            shape = mesh.bounds.extents;
        }
        else if (col is BoxCollider boxC)
        {
            offset = boxC.center;
            shape = boxC.size;
        }
        else if (col is SphereCollider sphereC)
        {
            offset = sphereC.center;
            shape = new Vector3(sphereC.radius, 0, 0);
        }
        else if (col is CapsuleCollider capsuleC)
        {
            offset = capsuleC.center;
            shape = new Vector3(capsuleC.radius, capsuleC.height, capsuleC.direction);
        }
        else
        {
            Debug.LogError("Unsupported collider type " + col.GetType());
            return 0;
        }

        int id = 17;
        id *= 31 + (int)Math.Round(offset.x * 1000);
        id *= 31 + (int)Math.Round(offset.y * 1000);
        id *= 31 + (int)Math.Round(offset.z * 1000);
        id *= 31 + (int)Math.Round(shape.x * 1000);
        id *= 31 + (int)Math.Round(shape.y * 1000);
        id *= 31 + (int)Math.Round(shape.z * 1000);
        return id;
    }


    private static readonly System.Diagnostics.Stopwatch stopwatch = new();

    public static void Debug_toggleTimer(string note = "")
    {
        if (stopwatch.IsRunning == false)
        {
            stopwatch.Restart();
        }
        else
        {
            stopwatch.Stop();
            Debug.Log(note + " time: " + stopwatch.Elapsed.TotalMilliseconds + "ms");
        }
    }
}
