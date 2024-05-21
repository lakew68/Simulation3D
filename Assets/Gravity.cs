using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Gravity
{
    private class OctreeNode
    {
        public Vector3 CenterOfMass;
        public double TotalMass;
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
    private double[] masses;
    public Vector3[] LastAccelerations;
    public Vector3[] GravAccelerations;
    private double G = Constants.G_in_units;
    private double softeningScale = Constants.softeningScale;

    public Gravity(GameObject[] gameObjects, double[] masses, Vector3 boundsMin, Vector3 boundsMax)
    {
        this.gameObjects = gameObjects;
        this.masses = masses;
        this.GravAccelerations = new Vector3[gameObjects.Length];
        this.LastAccelerations = new Vector3[gameObjects.Length];

        for(int i = 0; i < gameObjects.Length; i++)
        {
            GravAccelerations[i] = new Vector3(0.0f,0.0f,0.0f);
            LastAccelerations[i] = new Vector3(0.0f,0.0f,0.0f);
        }

        CalculateInitialAccelerations();

        root = new OctreeNode(boundsMin, boundsMax);
        BuildTree(root, new List<int>(Enumerable.Range(0, gameObjects.Length)));
    }

    private void BuildTree(OctreeNode node, List<int> indices)
    {
        if (indices.Count == 1)
        {
            // Leaf node
            node.ParticleIndices.Add(indices[0]);
            node.CenterOfMass = gameObjects[indices[0]].transform.position;
            node.TotalMass = masses[indices[0]];
            return;
        }

        // Non-leaf node, compute the center of mass and total mass
        Vector3 centerOfMass = Vector3.zero;
        double totalMass = 0;
        foreach (int index in indices)
        {
            centerOfMass += gameObjects[index].transform.position * (float)masses[index];
            totalMass += masses[index];
        }
        centerOfMass /= (float)totalMass;

        node.CenterOfMass = centerOfMass;
        node.TotalMass = totalMass;

        // Divide particles into octants
        Vector3 halfSize = (node.BoundsMax - node.BoundsMin) * 0.5f;
        Vector3 nodeCenter = node.CenterOfMass;

        List<int>[] octantIndices = new List<int>[8];
        for (int i = 0; i < 8; i++)
        {
            octantIndices[i] = new List<int>();
        }

        foreach (int index in indices)
        {
            int octant = GetOctant(nodeCenter, gameObjects[index].transform.position);
            octantIndices[octant].Add(index);
        }

        // Create children nodes
        for (int i = 0; i < 8; i++)
        {
            if (octantIndices[i].Count == 0) continue;

            Vector3 offset = GetOctantOffset(i, halfSize);

            Vector3 childCenter = nodeCenter + offset;
            Vector3 childBoundsMin = childCenter - halfSize;
            Vector3 childBoundsMax = childCenter + halfSize;
            node.Children[i] = new OctreeNode(childBoundsMin, childBoundsMax);

            BuildTree(node.Children[i], octantIndices[i]);
        }
    }

    private int GetOctant(Vector3 center, Vector3 position)
    {
        int octant = 0;
        if (position.x >= center.x) octant |= 4;
        if (position.y >= center.y) octant |= 2;
        if (position.z >= center.z) octant |= 1;
        return octant;
    }

    private Vector3 GetOctantOffset(int octant, Vector3 halfSize)
    {
        return new Vector3(
            ((octant & 4) == 0 ? -1 : 1) * halfSize.x,
            ((octant & 2) == 0 ? -1 : 1) * halfSize.y,
            ((octant & 1) == 0 ? -1 : 1) * halfSize.z);
    }

    public void DoEuler(Vector3[] velocities,float dt)
    {
        CalculateAccelerations();
        for (int i = 0; i < velocities.Length; i++)
        {
            gameObjects[i].transform.position += velocities[i] * dt;
            velocities[i] += GravAccelerations[i] * dt;
        }
        return;
    }
    public void DoRK4(Vector3[] velocities, float dt)
    {
        int n = gameObjects.Length;
        Vector3[] positions = new Vector3[n];
        Vector3[] k1_pos = new Vector3[n];
        Vector3[] k1_vel = new Vector3[n];
        Vector3[] k2_pos = new Vector3[n];
        Vector3[] k2_vel = new Vector3[n];
        Vector3[] k3_pos = new Vector3[n];
        Vector3[] k3_vel = new Vector3[n];
        Vector3[] k4_pos = new Vector3[n];
        Vector3[] k4_vel = new Vector3[n];

        // Save initial positions
        for (int i = 0; i < n; i++)
        {
            positions[i] = gameObjects[i].transform.position;
        }

        // Calculate k1
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k1_pos[i] = velocities[i] * dt;
            k1_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k2
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 0.5f * k1_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k2_pos[i] = (velocities[i] + 0.5f * k1_vel[i]) * dt;
            k2_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k3
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 0.5f * k2_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k3_pos[i] = (velocities[i] + 0.5f * k2_vel[i]) * dt;
            k3_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k4
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + k3_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k4_pos[i] = (velocities[i] + k3_vel[i]) * dt;
            k4_vel[i] = GravAccelerations[i] * dt;
        }
        
        // Update positions and velocities
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + (1.0f / 6.0f) * (k1_pos[i] + 2.0f * k2_pos[i] + 2.0f * k3_pos[i] + k4_pos[i]);
            velocities[i] += (1.0f / 6.0f) * (k1_vel[i] + 2.0f * k2_vel[i] + 2.0f * k3_vel[i] + k4_vel[i]);
        }
    }
    public void DoDormandPrince(Vector3[] velocities, float dt)
    {
        int n = gameObjects.Length;
        Vector3[] positions = new Vector3[n];
        Vector3[] k1_pos = new Vector3[n];
        Vector3[] k1_vel = new Vector3[n];
        Vector3[] k2_pos = new Vector3[n];
        Vector3[] k2_vel = new Vector3[n];
        Vector3[] k3_pos = new Vector3[n];
        Vector3[] k3_vel = new Vector3[n];
        Vector3[] k4_pos = new Vector3[n];
        Vector3[] k4_vel = new Vector3[n];
        Vector3[] k5_pos = new Vector3[n];
        Vector3[] k5_vel = new Vector3[n];
        Vector3[] k6_pos = new Vector3[n];
        Vector3[] k6_vel = new Vector3[n];
        Vector3[] k7_pos = new Vector3[n];
        Vector3[] k7_vel = new Vector3[n];
        Vector3[] k8_pos = new Vector3[n];
        Vector3[] k8_vel = new Vector3[n];

        // Save initial positions
        for (int i = 0; i < n; i++)
        {
            positions[i] = gameObjects[i].transform.position;
        }

        // Calculate k1
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k1_pos[i] = velocities[i] * dt;
            k1_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k2
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 1.0f / 5.0f * k1_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k2_pos[i] = (velocities[i] + 1.0f / 5.0f * k1_vel[i]) * dt;
            k2_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k3
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 3.0f / 40.0f * k1_pos[i] + 9.0f / 40.0f * k2_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k3_pos[i] = (velocities[i] + 3.0f / 40.0f * k1_vel[i] + 9.0f / 40.0f * k2_vel[i]) * dt;
            k3_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k4
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 44.0f / 45.0f * k1_pos[i] - 56.0f / 15.0f * k2_pos[i] + 32.0f / 9.0f * k3_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k4_pos[i] = (velocities[i] + 44.0f / 45.0f * k1_vel[i] - 56.0f / 15.0f * k2_vel[i] + 32.0f / 9.0f * k3_vel[i]) * dt;
            k4_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k5
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 19372.0f / 6561.0f * k1_pos[i] - 25360.0f / 2187.0f * k2_pos[i] + 64448.0f / 6561.0f * k3_pos[i] - 212.0f / 729.0f * k4_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k5_pos[i] = (velocities[i] + 19372.0f / 6561.0f * k1_vel[i] - 25360.0f / 2187.0f * k2_vel[i] + 64448.0f / 6561.0f * k3_vel[i] - 212.0f / 729.0f * k4_vel[i]) * dt;
            k5_vel[i] = GravAccelerations[i] * dt;
        }

        // Calculate k6
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 9017.0f / 3168.0f * k1_pos[i] - 355.0f / 33.0f * k2_pos[i] + 46732.0f / 5247.0f * k3_pos[i] + 49.0f / 176.0f * k4_pos[i] - 5103.0f / 18656.0f * k5_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k6_pos[i] = (velocities[i] + 9017.0f / 3168.0f * k1_vel[i] - 355.0f / 33.0f * k2_vel[i] + 46732.0f / 5247.0f * k3_vel[i] + 49.0f / 176.0f * k4_vel[i] - 5103.0f / 18656.0f * k5_vel[i]) * dt;
            k6_vel[i] = GravAccelerations[i] * dt;
        }
        // Calculate k7
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 35.0f / 384.0f * k1_pos[i] + 0.0f * k2_pos[i] + 500.0f / 1113.0f * k3_pos[i] + 125.0f / 192.0f * k4_pos[i] - 2187.0f / 6784.0f * k5_pos[i] + 11.0f / 84.0f * k6_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k7_pos[i] = (velocities[i] + 35.0f / 384.0f * k1_vel[i] + 0.0f * k2_vel[i] + 500.0f / 1113.0f * k3_vel[i] + 125.0f / 192.0f * k4_vel[i] - 2187.0f / 6784.0f * k5_vel[i] + 11.0f / 84.0f * k6_vel[i]) * dt;
            k7_vel[i] = GravAccelerations[i] * dt;
        }
        // Calculate k8
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] - 5103.0f / 18656.0f * k1_pos[i] + 0.0f * k2_pos[i] - 2187.0f / 6784.0f * k3_pos[i] + 8640.0f / 18656.0f * k4_pos[i] - 1728.0f / 6784.0f * k5_pos[i] + 6561.0f / 18656.0f * k6_pos[i] + 1.0f / 16.0f * k7_pos[i];
        }
        CalculateAccelerations();
        for (int i = 0; i < n; i++)
        {
            k8_pos[i] = (velocities[i] - 5103.0f / 18656.0f * k1_vel[i] + 0.0f * k2_vel[i] - 2187.0f / 6784.0f * k3_vel[i] + 8640.0f / 18656.0f * k4_vel[i] - 1728.0f / 6784.0f * k5_vel[i] + 6561.0f / 18656.0f * k6_vel[i] + 1.0f / 16.0f * k7_vel[i] + 35.0f / 384.0f * k8_pos[i]) * dt;
            k8_vel[i] = GravAccelerations[i] * dt;
        }

        // Update positions and velocities
        for (int i = 0; i < n; i++)
        {
            gameObjects[i].transform.position = positions[i] + 35.0f / 384.0f * k1_pos[i] + 500.0f / 1113.0f * k3_pos[i] + 125.0f / 192.0f * k4_pos[i] - 2187.0f / 6784.0f * k5_pos[i] + 11.0f / 84.0f * k6_pos[i] + 35.0f / 384.0f * k8_pos[i];
            velocities[i] += 35.0f / 384.0f * k1_vel[i] + 500.0f / 1113.0f * k3_vel[i] + 125.0f / 192.0f * k4_vel[i] - 2187.0f / 6784.0f * k5_vel[i] + 11.0f / 84.0f * k6_vel[i] + 35.0f / 384.0f * k8_vel[i];
        }
    }
    public void CalculateAccelerations()
    {
        Vector3 deltaRVector;
        float deltaR;
        LastAccelerations[0] = GravAccelerations[0];
        GravAccelerations[0] = new Vector3(0.0f,0.0f,0.0f);
        for (int i = 1; i < gameObjects.Length; i ++)
        {
            LastAccelerations[i] = GravAccelerations[i];
            GravAccelerations[i] = new Vector3(0.0f,0.0f,0.0f);
            for (int j = 0; j < i; j++)
            {
                
                deltaR = Vector3.Distance(gameObjects[j].transform.position, gameObjects[i].transform.position);
                deltaRVector = gameObjects[j].transform.position - gameObjects[i].transform.position;
                if (deltaR < softeningScale)
                {
                    GravAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(softeningScale,2) / deltaR) * deltaRVector;
                    LastAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(softeningScale,2) / deltaR) * deltaRVector;
                    GravAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(softeningScale,2) / deltaR) * deltaRVector;
                    LastAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(softeningScale,2) / deltaR) * deltaRVector;
                }
                else{
                    GravAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(deltaR,3)) * deltaRVector;
                    LastAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(deltaR,3)) * deltaRVector;
                    GravAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(deltaR,3)) * deltaRVector;
                    LastAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(deltaR,3)) * deltaRVector;
                }
                
            }
        }
    }
    public void CalculateInitialAccelerations()
    {

        Vector3 deltaRVector;
        float deltaR;
        
        for (int i = 1; i < gameObjects.Length; i ++)
        {
            for (int j = 0; j < i; j++)
            {
                
                deltaR = Vector3.Distance(gameObjects[j].transform.position, gameObjects[i].transform.position);
                deltaRVector = gameObjects[j].transform.position - gameObjects[i].transform.position;
                GravAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(deltaR,3)) * deltaRVector;
                LastAccelerations[i] += (float)(G * masses[j] / System.Math.Pow(deltaR,3)) * deltaRVector;
                GravAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(deltaR,3)) * deltaRVector;
                LastAccelerations[j] += (float)(-G * masses[i] / System.Math.Pow(deltaR,3)) * deltaRVector;
            }
        }
    }
}