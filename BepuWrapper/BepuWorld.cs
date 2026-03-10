using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using BepuWrapper.Entities.Behaviours;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.API.Util;
using static BepuWrapper.Entities.Behaviours.BepuPhysicsBehaviour;
using Sphere = BepuPhysics.Collidables.Sphere;

namespace BepuWrapper
{
    public sealed class BepuWorld : IDisposable
    {
        private readonly ICoreServerAPI api;

        public readonly BufferPool pool = new BufferPool();
        public Simulation sim;

        private double accumulator;
        private const float FixedDt = 1f / 20f;

        // Keep statics by chunk so you can rebuild incrementally.
        private readonly Dictionary<long, List<StaticHandle>> chunkStatics = new Dictionary<long, List<StaticHandle>>();
        private readonly HashSet<long> dirtyChunks = new HashSet<long>();

        // Cache common shapes (full blocks). Non-full boxes can be cached too, but start simple.
        private TypedIndex unitBoxShape;
        private Sphere unitSphere = new Sphere(1f);
        private TypedIndex unitSphereShape;
        private Dictionary<string, CachedCompound> ComputedShapes = new Dictionary<string, CachedCompound>();

        private List<BodyHandle> bodies = new List<BodyHandle>();

        public BepuWorld(ICoreServerAPI api)
        {
            this.api = api;

            // Narrow phase callbacks control filtering/materials.
            // Keep it minimal first.
            var narrowPhase = new SimpleNarrowPhaseCallbacks();
            var poseIntegrator = new SimplePoseIntegratorCallbacks(new Vector3(0, -9.81f, 0));

            sim = Simulation.Create(pool, narrowPhase, poseIntegrator, new SolveDescription(8, 1));

            unitBoxShape = sim.Shapes.Add(new Box(1f, 1f, 1f));
            unitSphereShape = sim.Shapes.Add(unitSphere);

            // Initial build around spawn or first player.

            api.Event.PlayerJoin += (player) => RebuildRegionAroundPlayer(player, radiusChunks: 2);
            //api.Event.PlayerDeath += (player, src) => BounceABall(player);

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

        public void BounceABall(IServerPlayer player)
        {
            Vec3f playerPos = player.Entity.Pos.XYZFloat;

            var sphereHandle = sim.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(playerPos.X, playerPos.Y + 10, playerPos.Z), unitSphere.ComputeInertia(1f), unitSphereShape, 0.01f));

            bodies.Add(sphereHandle);
        }

        public void Tick(float dt)
        {
            // Rebuild dirty chunks (throttle if needed).
            if (dirtyChunks.Count > 0)
            {
                // Simple: rebuild all dirty chunks immediately.
                // Better: cap rebuilds per tick to avoid spikes.
                var toRebuild = new List<long>(dirtyChunks);
                dirtyChunks.Clear();

                for (int i = 0; i < toRebuild.Count; i++)
                    RebuildChunkStatics(toRebuild[i]);
            }

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

            //Console.WriteLine($"Simulation static body count: {sim.Statics.Count}");
        }

        public void MarkChunkDirty(BlockPos pos)
        {
            // Vintage Story chunk size is typically 32 in X/Z (verify for your target),
            // but BlockAccessor provides chunk coords utilities too.
            int cx = pos.X >> 5; // /32
            int cz = pos.Z >> 5;
            int cy = pos.Y >> 5; // if you want vertical chunking; otherwise ignore Y for columns.

            dirtyChunks.Add(PackChunkKey(cx, cy, cz));
        }

        private void RebuildRegionAroundPlayer(IServerPlayer player, int radiusChunks)
        {
            var centerPos = player != null ? player.Entity.Pos.AsBlockPos : api.World.DefaultSpawnPosition.AsBlockPos;

            int centerCx = centerPos.X >> 5;
            int centerCz = centerPos.Z >> 5;
            int centerCy = centerPos.Y >> 5;

            for (int dx = -radiusChunks; dx <= radiusChunks; dx++)
                for (int dz = -radiusChunks; dz <= radiusChunks; dz++)
                {
                    long key = PackChunkKey(centerCx + dx, centerCy, centerCz + dz);
                    RebuildChunkStatics(key);
                }
        }

        private void RebuildChunkStatics(long chunkKey)
        {
            // Remove old statics for this chunk.
            if (chunkStatics.TryGetValue(chunkKey, out var existing))
            {
                for (int i = 0; i < existing.Count; i++)
                    sim.Statics.Remove(existing[i]);
                existing.Clear();
            }
            else
            {
                existing = new List<StaticHandle>(256);
                chunkStatics[chunkKey] = existing;
            }

            UnpackChunkKey(chunkKey, out int cx, out int cy, out int cz);

            // Chunk bounds in blocks (assuming 32). Adjust if your chunk size differs.
            int minX = cx << 5;
            int minY = cy << 5;
            int minZ = cz << 5;
            int maxX = minX + 31;
            int maxY = minY + 31;
            int maxZ = minZ + 31;

            var blockAcc = api.World.BlockAccessor;
            

            // Iterate blocks; for performance later use chunk data directly.
            for (int x = minX; x <= maxX; x++)
                for (int y = minY; y <= maxY; y++)
                    for (int z = minZ; z <= maxZ; z++)
                    {
                        var bpos = new BlockPos(x, y, z);
                        Block block = blockAcc.GetBlock(bpos);
                        if (block == null || block.IsLiquid() || block.BlockMaterial.Equals(EnumBlockMaterial.Air)) continue;

                        var boxes = block.CollisionBoxes;
                        if (boxes == null || boxes.Length == 0) continue;

                        // Full cube fast path.
                        if (boxes.Length == 1 && IsFullCube(boxes[0]))
                        {
                            // Centered at block center.
                            var p = new Vector3(x + 0.5f, y + 0.5f, z + 0.5f);
                            existing.Add(sim.Statics.Add(new StaticDescription(p, Quaternion.Identity, unitBoxShape)));
                            continue;
                        }

                        // Non-full: add each collision box as its own static.
                        for (int i = 0; i < boxes.Length; i++)
                        {
                            var aabb = boxes[i];

                            float sx = (float)(aabb.X2 - aabb.X1);
                            float sy = (float)(aabb.Y2 - aabb.Y1);
                            float sz = (float)(aabb.Z2 - aabb.Z1);

                            // Guard against degenerate boxes.
                            if (sx <= 0 || sy <= 0 || sz <= 0) continue;

                            var shapeIndex = sim.Shapes.Add(new Box(sx, sy, sz));

                            float px = x + (float)(aabb.X1 + aabb.X2) * 0.5f;
                            float py = y + (float)(aabb.Y1 + aabb.Y2) * 0.5f;
                            float pz = z + (float)(aabb.Z1 + aabb.Z2) * 0.5f;

                            existing.Add(sim.Statics.Add(new StaticDescription(new Vector3(px, py, pz), Quaternion.Identity, shapeIndex)));
                        }
                    }
        }

        private static bool IsFullCube(Cuboidf c) =>
            c.X1 == 0 && c.Y1 == 0 && c.Z1 == 0 &&
            c.X2 == 1 && c.Y2 == 1 && c.Z2 == 1;

        // ---- Chunk key packing (simple) ----
        private static long PackChunkKey(int cx, int cy, int cz)
        {
            // Pack 3 signed 21-bit coords into 64-bit (simple approach).
            // You can swap to a better hash if you like.
            long x = (long)(cx & 0x1FFFFF);
            long y = (long)(cy & 0x1FFFFF);
            long z = (long)(cz & 0x1FFFFF);
            return x | (y << 21) | (z << 42);
        }

        private static void UnpackChunkKey(long key, out int cx, out int cy, out int cz)
        {
            cx = (int)(key & 0x1FFFFF);
            cy = (int)((key >> 21) & 0x1FFFFF);
            cz = (int)((key >> 42) & 0x1FFFFF);

            // Sign extend 21-bit.
            if ((cx & (1 << 20)) != 0) cx |= unchecked((int)0xFFE00000);
            if ((cy & (1 << 20)) != 0) cy |= unchecked((int)0xFFE00000);
            if ((cz & (1 << 20)) != 0) cz |= unchecked((int)0xFFE00000);
        }

        public void Dispose()
        {
            // Remove all statics.
            foreach (var kv in chunkStatics)
            {
                var list = kv.Value;
                for (int i = 0; i < list.Count; i++)
                    sim.Statics.Remove(list[i]);
            }
            chunkStatics.Clear();
            dirtyChunks.Clear();

            sim.Dispose();
            pool.Clear();
        }

        internal BodyHandle RegisterEntityBody(Entity entity, BodyDescription bodyDescription, Vector3 localCenterOfMassOffset)
        {
            return sim.Bodies.Add(bodyDescription);
        }
    }
}
