using BepuPhysics;
using BepuUtilities.Memory;
using BepuWrapper.Api;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Util;

namespace BepuWrapper
{
    public sealed class BepuWorld : IDisposable
    {
        private readonly ICoreAPI api;

        public readonly BufferPool pool = new BufferPool();
        public Simulation sim;

        private double accumulator;
        private const float FixedDt = 1f / 20f;

        private Dictionary<string, CachedCompound> ComputedShapes = new Dictionary<string, CachedCompound>();

        private List<BodyHandle> bodies = new List<BodyHandle>();

        public BepuWorld(ICoreAPI api)
        {
            this.api = api;

            // Narrow phase callbacks control filtering/materials.
            // Keep it minimal first.
            var narrowPhase = new SimpleNarrowPhaseCallbacks();
            var poseIntegrator = new SimplePoseIntegratorCallbacks(new Vector3(0, -9.81f, 0));

            sim = Simulation.Create(pool, narrowPhase, poseIntegrator, new SolveDescription(8, 1));

            api.Event.RegisterGameTickListener(Tick, 0);
        }

        public CachedCompound AddCompoundShape(string shapeCode, BuiltCompound shape) 
        {
            var handle = sim.Shapes.Add(shape.Compound);
            var cacheItem = new CachedCompound()
            {
                BroadphaseRadius = shape.BroadphaseRadius,
                CompoundIndex = handle,
                Inertia = shape.Inertia,
                LocalCenterOfMassOffset = shape.LocalCenterOfMassOffset,
                ManualChildBoxes = shape.ManualChildBoxes
            };

            ComputedShapes.TryAdd(shapeCode, cacheItem);
            
            return cacheItem;
        }

        public CachedCompound? TryGetCompoundShape(string shapeCode) 
        { 
            return ComputedShapes.ContainsKey(shapeCode)? ComputedShapes.Get(shapeCode) : null;
        }

        public void Tick(float dt)
        {
            accumulator += dt;
            while (accumulator >= FixedDt)
            {
                sim.Timestep(FixedDt);
                accumulator -= FixedDt;
            }

            foreach (var handle in bodies)
            {
                var body = sim.Bodies.GetBodyReference(handle);
                var bodyPose = body.Pose;
                Console.WriteLine($"dynamic body {handle.Value} position: {bodyPose.Position.ToString()} orientation: {bodyPose.Orientation.ToString()}");
                
            }
        }

        public void Dispose()
        {
            sim.Dispose();
            pool.Clear();
        }

        internal BodyHandle RegisterEntityBody(Entity entity, BodyDescription bodyDescription, Vector3 localCenterOfMassOffset)
        {
            return sim.Bodies.Add(bodyDescription);
        }
    }
}
