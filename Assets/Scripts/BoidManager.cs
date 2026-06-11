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
    void Start()
    {
        //TODO separation of concerns, create a new method to create boids
        m_boids = new List<Boid>();

        //Get ALL the instances of the Flock class on the scene.
        var flocks = GameObject.FindObjectsOfType<Flock>();

        foreach (var flock in flocks)
        {
            //For each flock instance, add a reference to THIS manager
            flock.BoidManager = this;


            //? void List<Boid>.AddRange(IEnumerable<Boid> collection)
            //Add a collection of Objects(Boid) to the List<T>, it uses the 
            //IEnumerable<T> interface.
            m_boids.AddRange(flock.SpawnBirds());
        }

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
    }

    // Update is called once per frame
    void Update()
    {

    }

    void FixedUpdate()
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
        shader.SetBuffer(0, "Velocities", VelocitiesBuffer);
        shader.SetBuffer(0, "ResForces", ResForcesBuffer);

        int nGroups = m_boids.Count / 32 + 1;
        shader.Dispatch(0, nGroups, 1, 1);

        ResForcesBuffer.GetData(forces);

        for (int i = 0; i < m_boids.Count; i++)
        {
            m_boids[i].steeringForce = forces[i];
        }

        foreach (Boid boid in m_boids)
        {
            boid.UpdateSimulation(Time.fixedDeltaTime);
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
