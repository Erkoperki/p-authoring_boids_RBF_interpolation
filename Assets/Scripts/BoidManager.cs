using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class BoidManager : MonoBehaviour
{
    // Container of boids
    //References: https://learn.microsoft.com/en-us/dotnet/api/system.collections.generic?view=net-8.0
    private List<Boid> m_boids;
    public ComputeShader shader;
    public ComputeBuffer PositionsBuffer;
    public ComputeBuffer VelocitiesBuffer;
    public ComputeBuffer ResForcesBuffer;
    public ComputeBuffer InterpolatedVectorForcesBuffer;
    public ComputeBuffer SourcePointsBuffer;
    public ComputeBuffer LambdasBuffer;
    
    [SerializeField] private GridRenderer gr;
    public GPUToggle gpu;

    private int frameCount = 0;
    private int updateCounter = 0;
    private int updateCounter2  = 0;

    private float totalCompTime = 0;
    private bool avgTimePrinted = false;

    void Start()
    {
        //TODO separation of concerns, create a new method to create boids
        m_boids = new List<Boid>();

        //Get ALL the instances of the Flock class on the scene.
        var flocks = FindObjectsByType<Flock>();

        if (!gr) gr = FindAnyObjectByType<GridRenderer>();
        if (!gpu) gpu = FindAnyObjectByType<GPUToggle>();

        foreach (var flock in flocks)
        {
            //For each flock instance, add a reference to THIS manager
            flock.BoidManager = this;


            //? void List<Boid>.AddRange(IEnumerable<Boid> collection)
            //Add a collection of Objects(Boid) to the List<T>, it uses the 
            //IEnumerable<T> interface.
            m_boids.AddRange(flock.SpawnBirds());
        }

        if (gpu.UseGPU())
        {
            // Steering Forces Setup
            Flock tempflock = flocks[0];
            shader.SetInt("numBoids", m_boids.Count);
            shader.SetFloat("neighborRadius", tempflock.NeighborRadius);
            shader.SetFloat("SeparationForceFactor", tempflock.SeparationForceFactor);
            shader.SetFloat("AlignmentForceFactor", tempflock.AlignmentForceFactor);
            shader.SetFloat("CohesionForceFactor", tempflock.CohesionForceFactor);
            shader.SetFloat("SeparationRadius", tempflock.SeparationRadius);
            shader.SetFloat("AlignmentRadius", tempflock.AlignmentRadius);
            shader.SetFloat("CohesionRadius", tempflock.CohesionRadius);

            PositionsBuffer = new ComputeBuffer(m_boids.Count, 3 * sizeof(float));
            VelocitiesBuffer = new ComputeBuffer(m_boids.Count, 3 * sizeof(float));
            ResForcesBuffer = new ComputeBuffer(m_boids.Count, 3 * sizeof(float));

            // Vector Field Kernel Setup
            shader.SetInt("m_kernel", gr.m_kernel);

            InterpolatedVectorForcesBuffer = new ComputeBuffer(m_boids.Count, 3 * sizeof(float));
            SourcePointsBuffer = new ComputeBuffer(gr.GetNumSourcePoints(), 3 * sizeof(float));
            LambdasBuffer = new ComputeBuffer(gr.GetNumSourcePoints(), 3 * sizeof(float));
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Debug.Log("Frame time: " + Time.deltaTime);
        if (5 < Time.realtimeSinceStartup && Time.realtimeSinceStartup < 20)
        {
            updateCounter2++;
        }
    }

    void FixedUpdate()
    {
        updateCounter++;
        if (frameCount < Time.frameCount) 
        {
            frameCount = Time.frameCount;
            // Debug.Log("Frame #" + Time.frameCount);
            // Debug.Log("Last frame had " + updateCounter + " updates.");
            // updateCounter = 0;
        }

        float startTime = Time.realtimeSinceStartup;

        if (gpu.UseGPU())
        {
            Vector3[] positions = new Vector3[m_boids.Count];
            Vector3[] velocities = new Vector3[m_boids.Count];
            Vector3[] forces = new Vector3[m_boids.Count];
            for (int boid = 0; boid < m_boids.Count; boid++)
            {
                positions[boid] = m_boids[boid].Position;
                velocities[boid] = m_boids[boid].Velocity;
                forces[boid] = Vector3.zero;  
            }

            PositionsBuffer.SetData(positions);
            VelocitiesBuffer.SetData(velocities);
            ResForcesBuffer.SetData(forces);
            shader.SetBuffer(0, "Positions", PositionsBuffer);
            shader.SetBuffer(1, "Positions", PositionsBuffer);
            shader.SetBuffer(0, "Velocities", VelocitiesBuffer);
            shader.SetBuffer(0, "ResForces", ResForcesBuffer);

            int nGroups = m_boids.Count / 32 + 1;
            shader.Dispatch(0, nGroups, 1, 1);

            ResForcesBuffer.GetData(forces);

            for (int i = 0; i < m_boids.Count; i++)
            {
                m_boids[i].steeringForce = forces[i];
            }

            // Set buffers for vector field calcs
        
            Vector3[] InterpolatedVectorForces = new Vector3[m_boids.Count];
            for (int i = 0; i < m_boids.Count; i++)
            {
                InterpolatedVectorForces[i] = Vector3.zero;
            }

            shader.SetBuffer(1, "InterpolatedVectorForces", InterpolatedVectorForcesBuffer);

            Vector3[] SourcePoints = new Vector3[gr.GetNumSourcePoints()];
            Vector3[] Lambdas = new Vector3[gr.GetNumSourcePoints()];
            SourcePoints = gr.GetSourcePoints().ToArray();
            for (int spoints = 0; spoints < gr.GetNumSourcePoints(); spoints++)
            {
                Lambdas[spoints] = gr.GetLambdaAsVector(spoints);
            }

            SourcePointsBuffer.SetData(SourcePoints);
            LambdasBuffer.SetData(Lambdas);
            
            shader.SetBuffer(1, "SourcePoints", SourcePointsBuffer);
            shader.SetBuffer(1, "m_lambdas", LambdasBuffer);

            shader.Dispatch(1, nGroups, 1, 1);

            InterpolatedVectorForcesBuffer.GetData(InterpolatedVectorForces);
            for (int i = 0; i < m_boids.Count; i++)
            {
                m_boids[i].InterpolatedVectorForce = InterpolatedVectorForces[i];
            }
        }
        
        foreach (Boid boid in m_boids)
        {
            boid.UpdateSimulation(Time.fixedDeltaTime);
        }

        float endTime = Time.realtimeSinceStartup;
        float deltaTime = endTime - startTime;
        float averageTime = deltaTime / m_boids.Count;

        if (!avgTimePrinted)
        {
            // Debug.Log("Time to update " + m_boids.Count + " boids: " + deltaTime);
            // Debug.Log("Average: " + averageTime);
            // Debug.Log("Overhead time: " + overheadTime);
        }
        
        totalCompTime += deltaTime;
        if (endTime > 20 && !avgTimePrinted)
        {
            avgTimePrinted = true;
            float avgCompTime = totalCompTime / updateCounter;
            Debug.Log("Average boid computation time over 20s: " + avgCompTime + " seconds.");
            Debug.Log("Average FPS over 15s: " + updateCounter2/15 + " FPS.");
        }
    }

    //* IEnumerable<T>
    /*
    IEnmerable is an interface that makes enables iteration over a collection,
    by exposing the enumerator. Here, GetNeighbors is retunrning a collection
    of all the boids that fit the condition of being a neighbor of the current boid
    */
    //TODO check for yield 
    //TODO check if boids from flock A and change to flock B
    public IEnumerable<Boid> GetNeighbors(Boid boid, float radius)
    {
        float radiusSq = radius * radius;
        foreach (var other in m_boids)
        {
            if (other != boid && (other.Position - boid.Position).sqrMagnitude < radiusSq)
                yield return other; //The yield return is providing the collection with the next boid in the interation
        }
    }

    public int GetNeighborsCount()
    {
        return m_boids.Count;
    }
}
