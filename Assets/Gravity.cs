using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Gravity
{
    private class OctreeNode
    {
        public Vector3 CenterOfMass;
        public float TotalMass;
        public Vector3 BoundsMin;
        public Vector3 BoundsMax;
        public List<int> ParticleIndices;
        public OctreeNode[] Children;

        public OctreeNode(Vector3 boundsMin, Vector3 boundsMax)
        {
            BoundsMin = boundsMin;
            BoundsMax = boundsMax;
            ParticleIndices = new List<int>();
            Children = new OctreeNode[8];
        }
    }

    private OctreeNode root;
    private GameObject[] gameObjects;
    private float[] masses;

    public Gravity(GameObject[] gameObjects, float[] masses, Vector3 boundsMin, Vector3 boundsMax)
    {
        this.gameObjects = gameObjects;
        this.masses = masses;
        root = new OctreeNode(boundsMin, boundsMax);
        BuildTree(root, gameObjects);
    }

    private void BuildTree(OctreeNode node, GameObject[] gameObjects)
    {
        if (gameObjects.Length == 1)
        {
            int particleIndex = System.Array.IndexOf(this.gameObjects, gameObjects[0]);
            node.ParticleIndices.Add(particleIndex);
            node.CenterOfMass = gameObjects[0].transform.position;
            node.TotalMass = masses[particleIndex]; // Use the mass of the particle
            return;
        }

        Vector3 center = (node.BoundsMin + node.BoundsMax) * 0.5f;
        List<GameObject>[] octants = new List<GameObject>[8];

        for (int i = 0; i < 8; i++)
        {
            octants[i] = new List<GameObject>();
        }

        foreach (GameObject obj in gameObjects)
        {
            int octant = GetOctant(center, obj.transform.position);
            octants[octant].Add(obj);
        }

        node.TotalMass = 0.0f;
        node.CenterOfMass = Vector3.zero;

        for (int i = 0; i < 8; i++)
        {
            if (octants[i].Count > 0)
            {
                Vector3 newMin = node.BoundsMin;
                Vector3 newMax = center;

                if ((i & 1) != 0) newMin.x = center.x; else newMax.x = center.x;
                if ((i & 2) != 0) newMin.y = center.y; else newMax.y = center.y;
                if ((i & 4) != 0) newMin.z = center.z; else newMax.z = center.z;

                node.Children[i] = new OctreeNode(newMin, newMax);
                BuildTree(node.Children[i], octants[i].ToArray());

                node.TotalMass += node.Children[i].TotalMass;
                node.CenterOfMass += node.Children[i].CenterOfMass * node.Children[i].TotalMass;
            }
        }

        if (node.TotalMass > 0)
        {
            node.CenterOfMass /= node.TotalMass;
        }
    }

    private int GetOctant(Vector3 center, Vector3 position)
    {
        int octant = 0;
        if (position.x >= center.x) octant |= 1;
        if (position.y >= center.y) octant |= 2;
        if (position.z >= center.z) octant |= 4;
        return octant;
    }

    // Additional methods for gravitational force calculation, traversal, etc., can be added here.
}