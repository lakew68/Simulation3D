using UnityEngine;
using Unity.Mathematics;
using Random=UnityEngine.Random;

public class Simulation3D : MonoBehaviour
{
    public float radius;
    public Vector3 position;
    public Vector3[] velocities;
    public Vector3 velocity;
    public Material sphereMaterial;
    public GameObject[] spheres;
    public GameObject sphere;
    public int numSpheres;
    public float maxVel;
    public Vector3 minBounds;
    public Vector3 maxBounds;

    void Start()
    {
        numSpheres = 100;
        radius = 0.05f;
        maxVel = 1.0f;
        velocities = new Vector3[numSpheres];
        spheres = new GameObject[numSpheres];
        sphereMaterial = new Material(Shader.Find("Standard"));
        sphereMaterial.color = Color.red;

        CalculateBounds();

        for(int i = 0; i < numSpheres; i++)
        {

            position = new Vector3(Random.Range(minBounds.x, maxBounds.x), Random.Range(minBounds.y, maxBounds.y),Random.Range(minBounds.z, maxBounds.z));
            velocity = new Vector3(Random.Range(-maxVel, maxVel), Random.Range(-maxVel, maxVel),Random.Range(-maxVel, maxVel));
            sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = position;
            velocities[i] = velocity;
            sphere.transform.localScale = 2 * radius * Vector3.one; // radius to diameter
            sphere.GetComponent<Renderer>().material = sphereMaterial;
            spheres[i] = sphere;
        }
        
        
    }

    void Update()
    {
        for(int i = 0; i < numSpheres;i++)
        {
            spheres[i].transform.position += velocities[i] * Time.deltaTime;

            CheckCollisions();
        }
        
    }

    void CheckCollisions()
    {
        for(int i = 0; i < numSpheres;i++)
        {
            // Check for collisions with bounds and bounce

            if (spheres[i].transform.position.x - radius < minBounds.x)
            {
                velocities[i].x = -velocities[i].x; // Invert X velocity
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.x = 2.0f * (minBounds.x + radius) - newPosition.x; // Correct position -- (x_min + r) + ((x_min + r) - (x_curr)) = 2 (x_min + r) - x_curr
                spheres[i].transform.position = newPosition;
            }
            else if (spheres[i].transform.position.x + radius > maxBounds.x)
            {
                velocities[i].x = -velocities[i].x; // Invert X velocity
                Vector3 newPosition = spheres[i].transform.position;
                newPosition.x = 2.0f * (maxBounds.x - radius) - newPosition.x; // Correct position --  2 (x_max - r) - x_curr
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
                velocities[i].y = -velocities[i].y; // Invert X velocity
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
                velocities[i].z = -velocities[i].z; // Invert X velocity
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
                Debug.LogError("Main camera not found. Please tag the main camera as 'MainCamera'.");
                return;
            }

            // Calculate min and max bounds based on the camera's viewport
            minBounds = cam.ViewportToWorldPoint(new Vector3(0, 0, cam.nearClipPlane));
            maxBounds = cam.ViewportToWorldPoint(new Vector3(1, 1, cam.nearClipPlane));

            // Adjust bounds to account for the depth of the objects (spheres)
            minBounds.z = cam.transform.position.z + cam.nearClipPlane;
            maxBounds.z = cam.transform.position.z + cam.farClipPlane;
        }

}