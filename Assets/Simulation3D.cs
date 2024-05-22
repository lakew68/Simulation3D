using UnityEngine;
using Unity.Mathematics;
using Random = UnityEngine.Random;
using System.Linq.Expressions;
using System;
using Unity.Burst.CompilerServices;
using UnityEditor.ShaderGraph.Internal;

public class Simulation3D : MonoBehaviour
{
    float radius;
    double mass;
    double[] masses;
    Vector3 position;
    Vector3[] velocities;
    Vector3 velocity;
    Material sphereMaterial;
    GameObject[] spheres;
    GameObject sphere;
    int numSpheres;
    int step;
    float maxVel;
    Vector3 minBounds;
    Vector3 maxBounds;
    Gravity g;

    void Start()
    {
        numSpheres = 2;
        radius = 0.05f;
        mass = 1e-1;
        step = 0;
        maxVel = 0.02f;
        velocities = new Vector3[numSpheres];
        masses = new double[numSpheres];
        spheres = new GameObject[numSpheres];
        sphereMaterial = new Material(Shader.Find("Standard"));
        sphereMaterial.color = Color.red;

        CalculateBounds();

        for (int i = 0; i < numSpheres; i++)
        {
            position = new Vector3((float)Random.Range(minBounds.x, maxBounds.x)*Constants.initScale, (float)Random.Range(minBounds.y, maxBounds.y)*Constants.initScale, (minBounds.z+ maxBounds.z) * 0.5f);
            velocity = new Vector3((float)Random.Range(-maxVel, maxVel), (float)Random.Range(-maxVel, maxVel), (float)Random.Range(-maxVel, maxVel));
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = position;
            velocities[i] = velocity;
            masses[i] = mass;
            sphere.transform.localScale = 2 * (float)radius * Vector3.one; // radius to diameter
            sphere.GetComponent<Renderer>().material = sphereMaterial;
            spheres[i] = sphere;
        }
        masses[0] = 1.0;
        masses[1] = 1e-3;
        spheres[0].transform.position = new Vector3(0.0f,0.0f,(minBounds.z+ maxBounds.z) * 0.5f);
        velocities[0] = new Vector3(0.0f,0.0f,0.0f);
        Material SunMaterial = new Material(Shader.Find("Standard"));
        SunMaterial.color = Color.yellow;
        spheres[0].GetComponent<Renderer>().material = SunMaterial;
        spheres[1].transform.position = new Vector3(1.0f,0.0f,(minBounds.z+ maxBounds.z) * 0.5f);
        velocities[1] = new Vector3(0.0f,0.17f,0.0f);
        g = new Gravity(spheres, masses, minBounds, maxBounds);
    }

    void Update()
    {
        g.DoRK4(velocities,Time.deltaTime,step);

        for (int i = 0; i < numSpheres; i++)
        {
            CheckCollisions();
        }

        step += 1;
    }

    void CheckCollisions()
    {
        for (int i = 0; i < numSpheres; i++)
        {
            // Check for collisions with bounds and bounce

            if (spheres[i].transform.position.x - radius < minBounds.x)
            {
                velocities[i].x = Math.Abs(velocities[i].x); // Invert X velocity
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.x = 2.0f * (minBounds.x + radius) - newPosition.x; // Correct position -- (x_min + r) + ((x_min + r) - (x_curr)) = 2 (x_min + r) - x_curr
                spheres[i].transform.position = newPosition;
            }
            else if (spheres[i].transform.position.x + radius > maxBounds.x)
            {
                velocities[i].x = -Math.Abs(velocities[i].x); // Invert X velocity
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.x = 2.0f * (maxBounds.x - radius) - newPosition.x; // Correct position -- 2 (x_max - r) - x_curr
                spheres[i].transform.position = newPosition;
            }
            if (spheres[i].transform.position.y - radius < minBounds.y)
            {
                velocities[i].y = -velocities[i].y;
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.y = 2.0f * (minBounds.y + radius) - newPosition.y;
                spheres[i].transform.position = newPosition;
            }
            else if (spheres[i].transform.position.y + radius > maxBounds.y)
            {
                velocities[i].y = -velocities[i].y;
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.y = 2.0f * (maxBounds.y - radius) - newPosition.y;
                spheres[i].transform.position = newPosition;
            }
            if (spheres[i].transform.position.z - radius < minBounds.z)
            {
                velocities[i].z = -velocities[i].z;
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.z = 2.0f * (minBounds.z + radius) - newPosition.z;
                spheres[i].transform.position = newPosition;
            }
            else if (spheres[i].transform.position.z + radius > maxBounds.z)
            {
                velocities[i].z = -velocities[i].z;
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.z = 2.0f * (maxBounds.z - radius) - newPosition.z;
                spheres[i].transform.position = newPosition;
            }
        }
    }

    void CalculateBounds()
    {
        Camera cam = Camera.main;

        if (cam == null)
        {
            Debug.LogError("Main camera not found. Please tag the main camera as 'Main Camera'.");
            return;
        }

        // Calculate min and max bounds based on the camera's viewport
        minBounds = cam.ViewportToWorldPoint(new Vector3(0, 0, cam.nearClipPlane));
        maxBounds = cam.ViewportToWorldPoint(new Vector3(1, 1, cam.nearClipPlane));
        float camCenter = (cam.nearClipPlane + cam.farClipPlane) * 0.5f;
        minBounds.z = minBounds.x + camCenter;
        maxBounds.z = maxBounds.x + camCenter;
    }
}