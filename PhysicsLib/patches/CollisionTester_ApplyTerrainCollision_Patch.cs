using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Entities.Behaviours;
using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace PhysicsLib.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.ApplyTerrainCollision))]
    public static class CollisionTester_ApplyTerrainCollision_Patch
    {
        public static IDynamicCollisionSource? DynamicCollisionSource;

        private sealed class SupportState
        {
            public long SupportEntityId;
            public Vector3 LocalAnchorPoint;
        }

        private sealed class SupportCandidate
        {
            public Entity SupportEntity = null!;
            public DynamicCollisionBox DynamicBox = null!;
            public double TopY;
        }

        private static readonly Dictionary<long, SupportState> SupportStates = new();

        private const double MotionBiasThreshold = 0.0001;

        private const double SupportSnapAbove = 0.025;
        private const double SupportSnapBelow = 0.09;

        // Tightened: only skip horizontal collision when essentially flush with the floor surface
        private const double FloorIgnoreTopTolerance = 0.005;
        private const double SupportHorizontalPadding = 0.015;
        private const double SupportFootInset = 0.025;
        private const double MinSupportUpY = 0.65;

        private const double SweepSkin = 0.00001;

        // Minimum half-extent enforced on thin dynamic colliders to prevent tunneling.
        // Masts and other thin geometry can be narrower than one tick of player motion,
        // causing the AABB to start the next frame already overlapping.
        private const float MinDynamicColliderHalfExtent = 0.05f;

        [HarmonyPrefix]
        public static bool Prefix(
            CollisionTester __instance,
            Entity entity,
            EntityPos entityPos,
            float dtFactor,
            ref Vec3d newPosition,
            float stepHeight = 1f,
            float yExtra = 1f)
        {
            ApplyTerrainAndDynamicCollision(
                __instance,
                entity,
                entityPos,
                dtFactor,
                ref newPosition,
                stepHeight,
                yExtra
            );

            return false;
        }

        public static void ClearStandingOnEntity(Entity entity)
        {
            if (entity == null) return;
            SupportStates.Remove(entity.EntityId);
        }

        private static void SetStandingOnEntity(Entity entity, Entity supportEntity, Vector3 localAnchorPoint)
        {
            if (entity == null || supportEntity == null) return;

            SupportStates[entity.EntityId] = new SupportState
            {
                SupportEntityId = supportEntity.EntityId,
                LocalAnchorPoint = localAnchorPoint
            };
        }

        private static Entity? ResolveSupportEntity(Entity entity, out SupportState? supportState)
        {
            supportState = null;
            if (entity == null) return null;
            if (!SupportStates.TryGetValue(entity.EntityId, out supportState) || supportState == null)
                return null;
            return entity.World.GetEntityById(supportState.SupportEntityId);
        }

        private static void ApplyTerrainAndDynamicCollision(
            CollisionTester tester,
            Entity entity,
            EntityPos entityPos,
            float dtFactor,
            ref Vec3d newPosition,
            float stepHeight,
            float yExtra)
        {
            tester.minPos.dimension = entityPos.Dimension;

            IWorldAccessor world = entity.World;
            Vec3d pos = tester.pos;
            Cuboidd entityBox = tester.entityBox;
            BlockPos collBlockPos = new BlockPos(entityPos.Dimension);

            pos.X = entityPos.X;
            pos.Y = entityPos.Y;
            pos.Z = entityPos.Z;

            entityBox.SetAndTranslate(entity.CollisionBox, pos.X, pos.Y, pos.Z);

            // --- Carry delta: collect it but apply only to final position, not to the sweep start ---
            // This prevents the one-frame lag where collision is resolved against stale terrain positions.
            Entity? previousSupportEntity = ResolveSupportEntity(entity, out SupportState? previousSupportState);
            DynamicPhysicsBehaviour? previousSupportPhysics = previousSupportEntity?.GetBehavior<DynamicPhysicsBehaviour>();

            Vec3d carryDelta = new Vec3d(0, 0, 0);
            if (previousSupportEntity != null && previousSupportPhysics != null && previousSupportState != null)
            {
                if (previousSupportPhysics.TryGetPointVelocityDelta(previousSupportState.LocalAnchorPoint, out Vec3d delta))
                {
                    carryDelta = delta;
                }
            }

            double motionX = entityPos.Motion.X * dtFactor + carryDelta.X;
            double motionY = entityPos.Motion.Y * dtFactor + carryDelta.Y;
            double motionZ = entityPos.Motion.Z * dtFactor + carryDelta.Z;

            double motionBiasX = 0.0;
            double motionBiasY = 0.0;
            double motionBiasZ = 0.0;

            if (motionX > MotionBiasThreshold) motionBiasX = MotionBiasThreshold;
            if (motionX < -MotionBiasThreshold) motionBiasX = -MotionBiasThreshold;
            if (motionY > MotionBiasThreshold) motionBiasY = MotionBiasThreshold;
            if (motionY < -MotionBiasThreshold) motionBiasY = -MotionBiasThreshold;
            if (motionZ > MotionBiasThreshold) motionBiasZ = MotionBiasThreshold;
            if (motionZ < -MotionBiasThreshold) motionBiasZ = -MotionBiasThreshold;

            motionX += motionBiasX;
            motionY += motionBiasY;
            motionZ += motionBiasZ;

            GenerateTerrainCollisionBoxList(
                tester,
                world.BlockAccessor,
                motionX,
                motionY,
                motionZ,
                stepHeight,
                yExtra,
                entityPos.Dimension
            );

            List<DynamicCollisionBox> dynamicBoxes = CollectDynamicCollisionBoxes(
                entity,
                entityPos.Dimension,
                entityBox,
                motionX,
                motionY,
                motionZ,
                stepHeight,
                yExtra
            );

            bool collidedVertically = false;
            bool collidedHorizontally = false;

            int terrainCount = tester.CollisionBoxList.Count;
            Cuboidd[] terrainBoxes = tester.CollisionBoxList.cuboids;

            // --- Y pass ---
            for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
            {
                EnumPushDirection direction = EnumPushDirection.None;
                motionY = terrainBoxes[i].pushOutY(entityBox, motionY, ref direction);

                if (direction != EnumPushDirection.None)
                {
                    collidedVertically = true;
                    collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                    tester.CollisionBoxList.blocks[i].OnEntityCollide(
                        world, entity, collBlockPos,
                        direction == EnumPushDirection.Negative ? BlockFacing.UP : BlockFacing.DOWN,
                        tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                        !entity.CollidedVertically
                    );
                }
            }

            SupportCandidate? standingSupport = null;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dynamicBox = dynamicBoxes[i];

                EnumPushDirection dynamicDirection = EnumPushDirection.None;
                double pushedY = PushOutYObbAabb(dynamicBox, entityBox, motionY, ref dynamicDirection);

                if (dynamicDirection == EnumPushDirection.None)
                    continue;

                motionY = pushedY;
                collidedVertically = true;

                if (dynamicDirection == EnumPushDirection.Negative &&
                    dynamicBox.CanSupport &&
                    dynamicBox.SourceEntity != null)
                {
                    Cuboidd movedYBox = OffsetEntityBox(entityBox, 0.0, pushedY, 0.0);

                    if (TryGetSupportTopUnderFeet(movedYBox, dynamicBox, out double supportTopY))
                    {
                        double feetDelta = movedYBox.Y1 - supportTopY;

                        if (feetDelta >= -SupportSnapAbove && feetDelta <= SupportSnapBelow)
                        {
                            if (standingSupport == null || supportTopY > standingSupport.TopY)
                            {
                                standingSupport = new SupportCandidate
                                {
                                    SupportEntity = dynamicBox.SourceEntity,
                                    DynamicBox = dynamicBox,
                                    TopY = supportTopY
                                };
                            }
                        }
                    }
                }
            }

            // Advance entity box in Y before X/Z passes so horizontal collision uses the correct position
            entityBox.Translate(0.0, motionY, 0.0);
            entity.CollidedVertically = collidedVertically;

            // Resolved support entity used to decide whether to skip horizontal collisions from the floor surface
            Entity? resolvedSupportEntity = standingSupport?.SupportEntity ?? previousSupportEntity;

            // --- X pass (uses Y-translated entityBox) ---
            for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
            {
                EnumPushDirection direction = EnumPushDirection.None;
                motionX = terrainBoxes[i].pushOutX(entityBox, motionX, ref direction);

                if (direction != EnumPushDirection.None)
                {
                    collidedHorizontally = true;
                    collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                    tester.CollisionBoxList.blocks[i].OnEntityCollide(
                        world, entity, collBlockPos,
                        direction == EnumPushDirection.Negative ? BlockFacing.EAST : BlockFacing.WEST,
                        tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                        !entity.CollidedHorizontally
                    );
                }
            }

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dynamicBox = dynamicBoxes[i];

                if (ShouldIgnoreHorizontalDynamicCollision(dynamicBox, resolvedSupportEntity, entityBox))
                    continue;

                EnumPushDirection dynamicDirection = EnumPushDirection.None;
                double pushedX = PushOutXObbAabb(dynamicBox, entityBox, motionX, ref dynamicDirection);

                if (dynamicDirection != EnumPushDirection.None)
                {
                    motionX = pushedX;
                    collidedHorizontally = true;
                }
            }

            entityBox.Translate(motionX, 0.0, 0.0);

            // --- Z pass (uses Y+X-translated entityBox) ---
            for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
            {
                EnumPushDirection direction = EnumPushDirection.None;
                motionZ = terrainBoxes[i].pushOutZ(entityBox, motionZ, ref direction);

                if (direction != EnumPushDirection.None)
                {
                    collidedHorizontally = true;
                    collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                    tester.CollisionBoxList.blocks[i].OnEntityCollide(
                        world, entity, collBlockPos,
                        direction == EnumPushDirection.Negative ? BlockFacing.SOUTH : BlockFacing.NORTH,
                        tester.tmpPosDelta.Set(motionX, motionY, motionZ),
                        !entity.CollidedHorizontally
                    );
                }
            }

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dynamicBox = dynamicBoxes[i];

                if (ShouldIgnoreHorizontalDynamicCollision(dynamicBox, resolvedSupportEntity, entityBox))
                    continue;

                EnumPushDirection dynamicDirection = EnumPushDirection.None;
                double pushedZ = PushOutZObbAabb(dynamicBox, entityBox, motionZ, ref dynamicDirection);

                if (dynamicDirection != EnumPushDirection.None)
                {
                    motionZ = pushedZ;
                    collidedHorizontally = true;
                }
            }

            entity.CollidedHorizontally = collidedHorizontally;

            // Remove bias before applying ladder fix (order matters)
            motionX -= motionBiasX;
            motionY -= motionBiasY;
            motionZ -= motionBiasZ;

            // Ladder fix applied after bias removal
            if (motionY > 0.0 && entity.CollidedVertically)
            {
                motionY -= entity.LadderFixDelta;
            }

            double finalX = pos.X + motionX;
            double finalY = pos.Y + motionY;
            double finalZ = pos.Z + motionZ;

            // Single final support snap pass (removed the mid-sweep duplicate)
            Cuboidd finalEntityBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);

            List<DynamicCollisionBox> finalDynamicBoxes = CollectDynamicCollisionBoxes(
                entity,
                entityPos.Dimension,
                finalEntityBox,
                0.0, 0.0, 0.0,
                stepHeight,
                yExtra
            );

            SupportCandidate? finalSupport = FindBestSupportBelowFeet(finalEntityBox, finalDynamicBoxes);

            if (finalSupport != null)
            {
                double snapDelta = finalSupport.TopY - finalEntityBox.Y1;

                if (snapDelta >= -SupportSnapBelow && snapDelta <= SupportSnapAbove)
                {
                    finalY += snapDelta;
                    finalEntityBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);

                    if (entityPos.Motion.Y < 0.0)
                        entityPos.Motion.Y = 0.0;

                    entity.CollidedVertically = true;
                    entity.OnGround = true;
                }
            }

            if (finalSupport != null)
            {
                DynamicPhysicsBehaviour? supportPhysics = finalSupport.SupportEntity.GetBehavior<DynamicPhysicsBehaviour>();
                if (supportPhysics != null &&
                    supportPhysics.TryTransformWorldPointToLocal(GetFeetCenter(finalEntityBox), out Vector3 localAnchor))
                {
                    SetStandingOnEntity(entity, finalSupport.SupportEntity, localAnchor);
                    entity.OnGround = true;
                }
                else
                {
                    ClearStandingOnEntity(entity);
                }
            }
            else
            {
                ClearStandingOnEntity(entity);
            }

            newPosition.Set(finalX, finalY, finalZ);
        }

        private static bool ShouldIgnoreHorizontalDynamicCollision(
            DynamicCollisionBox dynamicBox,
            Entity? resolvedSupportEntity,
            Cuboidd entityBox)
        {
            if (resolvedSupportEntity == null) return false;
            if (dynamicBox.SourceEntity != resolvedSupportEntity) return false;
            if (!dynamicBox.CanSupport) return false;

            if (!TryGetSupportTopUnderFeet(entityBox, dynamicBox, out double supportTopY))
                return false;

            double feetDelta = entityBox.Y1 - supportTopY;

            // Only ignore horizontal collision when the entity is sitting on top of this surface,
            // not when approaching from the side (feetDelta < -tolerance means entity is below the surface top)
            return feetDelta >= -FloorIgnoreTopTolerance && feetDelta <= FloorIgnoreTopTolerance;
        }

        private static Vec3d GetFeetCenter(Cuboidd entityBox)
        {
            return new Vec3d(
                (entityBox.X1 + entityBox.X2) * 0.5,
                entityBox.Y1,
                (entityBox.Z1 + entityBox.Z2) * 0.5
            );
        }

        private static SupportCandidate? FindBestSupportBelowFeet(
            Cuboidd entityBox,
            List<DynamicCollisionBox> dynamicBoxes)
        {
            SupportCandidate? best = null;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dyn = dynamicBoxes[i];
                if (!dyn.CanSupport || dyn.SourceEntity == null) continue;

                if (!TryGetSupportTopUnderFeet(entityBox, dyn, out double supportTopY))
                    continue;

                double delta = entityBox.Y1 - supportTopY;
                if (delta < -SupportSnapAbove || delta > SupportSnapBelow) continue;

                if (best == null || supportTopY > best.TopY)
                {
                    best = new SupportCandidate
                    {
                        SupportEntity = dyn.SourceEntity,
                        DynamicBox = dyn,
                        TopY = supportTopY
                    };
                }
            }

            return best;
        }

        private static bool TryGetSupportTopUnderFeet(
            Cuboidd entityBox,
            DynamicCollisionBox dynamicBox,
            out double supportTopY)
        {
            supportTopY = double.NegativeInfinity;

            Vector3 upAxis = SafeNormalize(
                Vector3.Transform(Vector3.UnitY, dynamicBox.Orientation),
                Vector3.UnitY
            );

            if (upAxis.Y < MinSupportUpY) return false;

            Quaternion inverseOrientation = Quaternion.Inverse(dynamicBox.Orientation);

            double x1 = entityBox.X1 + SupportFootInset;
            double x2 = entityBox.X2 - SupportFootInset;
            double z1 = entityBox.Z1 + SupportFootInset;
            double z2 = entityBox.Z2 - SupportFootInset;

            if (x1 > x2) { double mid = (entityBox.X1 + entityBox.X2) * 0.5; x1 = mid; x2 = mid; }
            if (z1 > z2) { double mid = (entityBox.Z1 + entityBox.Z2) * 0.5; z1 = mid; z2 = mid; }

            double cx = (entityBox.X1 + entityBox.X2) * 0.5;
            double cz = (entityBox.Z1 + entityBox.Z2) * 0.5;
            double fy = entityBox.Y1;

            Vector3[] samples =
            {
                new Vector3((float)cx,  (float)fy, (float)cz),
                new Vector3((float)x1,  (float)fy, (float)z1),
                new Vector3((float)x1,  (float)fy, (float)z2),
                new Vector3((float)x2,  (float)fy, (float)z1),
                new Vector3((float)x2,  (float)fy, (float)z2)
            };

            bool found = false;

            for (int i = 0; i < samples.Length; i++)
            {
                Vector3 local = Vector3.Transform(samples[i] - dynamicBox.Center, inverseOrientation);

                if (MathF.Abs(local.X) > dynamicBox.HalfExtents.X + (float)SupportHorizontalPadding) continue;
                if (MathF.Abs(local.Z) > dynamicBox.HalfExtents.Z + (float)SupportHorizontalPadding) continue;

                Vector3 topLocal = new Vector3(local.X, dynamicBox.HalfExtents.Y, local.Z);
                Vector3 topWorld = dynamicBox.Center + Vector3.Transform(topLocal, dynamicBox.Orientation);

                if (!found || topWorld.Y > supportTopY)
                {
                    supportTopY = topWorld.Y;
                    found = true;
                }
            }

            return found;
        }

        private static void GenerateTerrainCollisionBoxList(
            CollisionTester tester,
            IBlockAccessor blockAccessor,
            double motionX, double motionY, double motionZ,
            float stepHeight, float yExtra, int dimension)
        {
            Cuboidd entityBox = tester.entityBox;

            bool minUnchanged = tester.minPos.SetAndEquals(
                (int)(entityBox.X1 + Math.Min(0.0, motionX)),
                (int)(entityBox.Y1 + Math.Min(0.0, motionY) - yExtra),
                (int)(entityBox.Z1 + Math.Min(0.0, motionZ))
            );

            double maxY = Math.Max(entityBox.Y1 + stepHeight, entityBox.Y2);

            bool maxUnchanged = tester.maxPos.SetAndEquals(
                (int)(entityBox.X2 + Math.Max(0.0, motionX)),
                (int)(maxY + Math.Max(0.0, motionY)),
                (int)(entityBox.Z2 + Math.Max(0.0, motionZ))
            );

            if (minUnchanged && maxUnchanged) return;

            tester.CollisionBoxList.Clear();
            tester.tmpPos.dimension = dimension;

            blockAccessor.WalkBlocks(
                tester.minPos,
                tester.maxPos,
                (block, x, y, z) =>
                {
                    Cuboidf[] collisionBoxes = block.GetCollisionBoxes(blockAccessor, tester.tmpPos.Set(x, y, z));
                    if (collisionBoxes != null)
                        tester.CollisionBoxList.Add(collisionBoxes, x, y, z, block);
                },
                centerOrder: true
            );
        }

        private static List<DynamicCollisionBox> CollectDynamicCollisionBoxes(
            Entity movingEntity, int dimension,
            Cuboidd movingEntityBox,
            double motionX, double motionY, double motionZ,
            float stepHeight, float yExtra)
        {
            List<DynamicCollisionBox> results = new List<DynamicCollisionBox>();
            if (DynamicCollisionSource == null) return results;

            Cuboidd queryBox = movingEntityBox.Clone();
            queryBox.X1 += Math.Min(0.0, motionX);
            queryBox.Y1 += Math.Min(0.0, motionY) - yExtra;
            queryBox.Z1 += Math.Min(0.0, motionZ);
            queryBox.X2 += Math.Max(0.0, motionX);
            queryBox.Y2 = Math.Max(queryBox.Y1 + stepHeight, queryBox.Y2 + Math.Max(0.0, motionY));
            queryBox.Z2 += Math.Max(0.0, motionZ);

            DynamicCollisionSource.CollectCollisionBoxes(movingEntity, queryBox, results);

            // FIX: Enforce a minimum half-extent on thin colliders (e.g. masts) so the swept
            // test can catch them before the AABB tunnels fully inside in one frame.
            for (int i = 0; i < results.Count; i++)
            {
                DynamicCollisionBox box = results[i];
                Vector3 he = box.HalfExtents;
                bool changed = false;

                if (he.X < MinDynamicColliderHalfExtent) { he.X = MinDynamicColliderHalfExtent; changed = true; }
                if (he.Z < MinDynamicColliderHalfExtent) { he.Z = MinDynamicColliderHalfExtent; changed = true; }

                if (changed)
                    box.HalfExtents = he;
            }

            return results;
        }

        private static Cuboidd OffsetEntityBox(Cuboidd box, double x, double y, double z)
        {
            return new Cuboidd(
                box.X1 + x, box.Y1 + y, box.Z1 + z,
                box.X2 + x, box.Y2 + y, box.Z2 + z
            );
        }

        private static bool IntersectsObbAabb(DynamicCollisionBox obb, Cuboidd aabb)
        {
            Vector3 aabbCenter = new Vector3(
                (float)((aabb.X1 + aabb.X2) * 0.5),
                (float)((aabb.Y1 + aabb.Y2) * 0.5),
                (float)((aabb.Z1 + aabb.Z2) * 0.5)
            );

            Vector3 aabbHalf = new Vector3(
                (float)((aabb.X2 - aabb.X1) * 0.5),
                (float)((aabb.Y2 - aabb.Y1) * 0.5),
                (float)((aabb.Z2 - aabb.Z1) * 0.5)
            );

            Vector3[] a =
            {
                SafeNormalize(Vector3.Transform(Vector3.UnitX, obb.Orientation), Vector3.UnitX),
                SafeNormalize(Vector3.Transform(Vector3.UnitY, obb.Orientation), Vector3.UnitY),
                SafeNormalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation), Vector3.UnitZ)
            };

            float[] ea = { obb.HalfExtents.X, obb.HalfExtents.Y, obb.HalfExtents.Z };
            float[] eb = { aabbHalf.X, aabbHalf.Y, aabbHalf.Z };

            float[,] r = new float[3, 3];
            float[,] absR = new float[3, 3];

            const float epsilon = 1e-6f;

            for (int i = 0; i < 3; i++)
            {
                r[i, 0] = a[i].X; r[i, 1] = a[i].Y; r[i, 2] = a[i].Z;
                absR[i, 0] = MathF.Abs(r[i, 0]) + epsilon;
                absR[i, 1] = MathF.Abs(r[i, 1]) + epsilon;
                absR[i, 2] = MathF.Abs(r[i, 2]) + epsilon;
            }

            Vector3 tWorld = aabbCenter - obb.Center;
            float[] t = { Vector3.Dot(tWorld, a[0]), Vector3.Dot(tWorld, a[1]), Vector3.Dot(tWorld, a[2]) };

            for (int i = 0; i < 3; i++)
            {
                float ra = ea[i];
                float rb = eb[0] * absR[i, 0] + eb[1] * absR[i, 1] + eb[2] * absR[i, 2];
                if (MathF.Abs(t[i]) > ra + rb) return false;
            }

            // AABB face axes
            for (int j = 0; j < 3; j++)
            {
                float ra = ea[0] * absR[0, j] + ea[1] * absR[1, j] + ea[2] * absR[2, j];
                float rb = eb[j];
                float projectedT = MathF.Abs(t[0] * r[0, j] + t[1] * r[1, j] + t[2] * r[2, j]);
                if (projectedT > ra + rb) return false;
            }

            // Cross product axes
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    float ra =
                        ea[(i + 1) % 3] * absR[(i + 2) % 3, j] +
                        ea[(i + 2) % 3] * absR[(i + 1) % 3, j];

                    float rb =
                        eb[(j + 1) % 3] * absR[i, (j + 2) % 3] +
                        eb[(j + 2) % 3] * absR[i, (j + 1) % 3];

                    float projectedT = MathF.Abs(
                        t[(i + 2) % 3] * r[(i + 1) % 3, j] -
                        t[(i + 1) % 3] * r[(i + 2) % 3, j]
                    );

                    if (projectedT > ra + rb) return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Computes the signed penetration depth of the AABB into the OBB along a given world axis.
        /// Returns 0 if not overlapping on this axis.
        /// Positive result = AABB must move in +axis direction to escape.
        /// </summary>
        private static double ComputePenetrationDepthAlongAxis(
            DynamicCollisionBox obb,
            Cuboidd aabb,
            int axis)
        {
            // Project both shapes onto the world axis
            double aabbMin = axis == 0 ? aabb.X1 : axis == 1 ? aabb.Y1 : aabb.Z1;
            double aabbMax = axis == 0 ? aabb.X2 : axis == 1 ? aabb.Y2 : aabb.Z2;

            // Project OBB onto world axis using support mapping
            Vector3 worldAxis = axis == 0 ? Vector3.UnitX : axis == 1 ? Vector3.UnitY : Vector3.UnitZ;

            Vector3 ax = SafeNormalize(Vector3.Transform(Vector3.UnitX, obb.Orientation), Vector3.UnitX);
            Vector3 ay = SafeNormalize(Vector3.Transform(Vector3.UnitY, obb.Orientation), Vector3.UnitY);
            Vector3 az = SafeNormalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation), Vector3.UnitZ);

            float obbCenterProj = Vector3.Dot(obb.Center, worldAxis);
            float obbRadius =
                MathF.Abs(Vector3.Dot(ax, worldAxis)) * obb.HalfExtents.X +
                MathF.Abs(Vector3.Dot(ay, worldAxis)) * obb.HalfExtents.Y +
                MathF.Abs(Vector3.Dot(az, worldAxis)) * obb.HalfExtents.Z;

            double obbMin = obbCenterProj - obbRadius;
            double obbMax = obbCenterProj + obbRadius;

            // No overlap → no penetration
            if (aabbMax <= obbMin || aabbMin >= obbMax) return 0.0;

            // Penetration from positive side (push AABB in +axis)
            double overlapPos = obbMax - aabbMin;
            // Penetration from negative side (push AABB in -axis)
            double overlapNeg = aabbMax - obbMin;

            // Return the smallest correction (signed)
            if (overlapPos < overlapNeg)
                return overlapPos;   // positive: move AABB in +axis
            else
                return -overlapNeg;  // negative: move AABB in -axis
        }

        private static double PushOutAxisObbAabb(
            DynamicCollisionBox obb,
            Cuboidd entityBox,
            double motion,
            int axis,
            ref EnumPushDirection direction)
        {
            direction = EnumPushDirection.None;

            Cuboidd movedBox = axis switch
            {
                0 => OffsetEntityBox(entityBox, motion, 0.0, 0.0),
                1 => OffsetEntityBox(entityBox, 0.0, motion, 0.0),
                2 => OffsetEntityBox(entityBox, 0.0, 0.0, motion),
                _ => entityBox
            };

            bool startsIntersecting = IntersectsObbAabb(obb, entityBox);
            bool endsIntersecting = IntersectsObbAabb(obb, movedBox);

            // Moving away from an overlap — let it pass through to resolve
            if (startsIntersecting && !endsIntersecting)
                return motion;

            // Already overlapping and staying overlapping — depenetrate along this axis.
            //
            // This branch is reached when:
            //   (a) The entity tunnelled into a thin collider in one frame (mast, rope, etc.)
            //   (b) The min-extent padding caused a marginal pre-overlap at rest
            //   (c) Floating-point drift left the entity just inside a surface
            //
            // Rules:
            //   1. Always register the collision (set direction) so callers know contact occurred.
            //   2. If motion and pen agree in sign: clamp to pen (push out, don't overshoot).
            //   3. If motion and pen disagree in sign: the entity is moving INTO the collider
            //      from a pre-existing overlap — stop it at 0.0 (wall it off) but still flag
            //      the direction so CollidedHorizontally / CollidedVertically get set.
            //   4. If motion is zero (standing still in overlap): nudge by pen but cap the nudge
            //      to a small depenetration budget so we don't launch the entity on flat ground.
            if (startsIntersecting && endsIntersecting)
            {
                double pen = ComputePenetrationDepthAlongAxis(obb, entityBox, axis);
                if (Math.Abs(pen) < 1e-10) return motion;

                direction = pen > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;

                if (motion == 0.0)
                {
                    // Standing still — tiny nudge only, never more than 1/4 of a block per frame
                    const double MaxRestNudge = 0.25;
                    return Math.Clamp(pen, -MaxRestNudge, MaxRestNudge);
                }

                if (Math.Sign(pen) != Math.Sign(motion))
                {
                    // Moving into a pre-existing overlap — stop here, wall blocked
                    return 0.0;
                }

                // Moving away from (or parallel to) the overlap — clamp to pen, never overshoot
                return Math.Abs(pen) < Math.Abs(motion) ? pen : motion;
            }

            // Not intersecting at start — do a swept test
            if (!endsIntersecting)
                return motion;

            Vector3 sweep = axis switch
            {
                0 => new Vector3((float)motion, 0f, 0f),
                1 => new Vector3(0f, (float)motion, 0f),
                2 => new Vector3(0f, 0f, (float)motion),
                _ => Vector3.Zero
            };

            if (!TrySweepAabbAgainstObb(entityBox, sweep, obb, out double hitFraction))
                return motion;

            double safeFraction = hitFraction;

            if (Math.Abs(motion) > SweepSkin)
                safeFraction -= SweepSkin / Math.Abs(motion);
            else
                safeFraction = 0.0;

            safeFraction = Math.Clamp(safeFraction, 0.0, 1.0);

            double pushed = motion * safeFraction;

            direction = motion > 0.0 ? EnumPushDirection.Positive : EnumPushDirection.Negative;

            return pushed;
        }

        private static double PushOutXObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionX, ref EnumPushDirection direction)
            => PushOutAxisObbAabb(obb, entityBox, motionX, 0, ref direction);

        private static double PushOutYObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionY, ref EnumPushDirection direction)
            => PushOutAxisObbAabb(obb, entityBox, motionY, 1, ref direction);

        private static double PushOutZObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionZ, ref EnumPushDirection direction)
            => PushOutAxisObbAabb(obb, entityBox, motionZ, 2, ref direction);

        private static bool TrySweepAabbAgainstObb(
            Cuboidd movingAabb,
            Vector3 sweep,
            DynamicCollisionBox staticObb,
            out double hitFraction)
        {
            hitFraction = 1.0;

            Vector3 movingCenter = new Vector3(
                (float)((movingAabb.X1 + movingAabb.X2) * 0.5),
                (float)((movingAabb.Y1 + movingAabb.Y2) * 0.5),
                (float)((movingAabb.Z1 + movingAabb.Z2) * 0.5)
            );

            Vector3 movingHalf = new Vector3(
                (float)((movingAabb.X2 - movingAabb.X1) * 0.5),
                (float)((movingAabb.Y2 - movingAabb.Y1) * 0.5),
                (float)((movingAabb.Z2 - movingAabb.Z1) * 0.5)
            );

            Vector3[] obbAxes =
            {
                SafeNormalize(Vector3.Transform(Vector3.UnitX, staticObb.Orientation), Vector3.UnitX),
                SafeNormalize(Vector3.Transform(Vector3.UnitY, staticObb.Orientation), Vector3.UnitY),
                SafeNormalize(Vector3.Transform(Vector3.UnitZ, staticObb.Orientation), Vector3.UnitZ)
            };

            Vector3[] worldAxes = { Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ };

            double enter = 0.0;
            double exit = 1.0;

            for (int i = 0; i < 3; i++)
            {
                if (!SweepTestAxis(obbAxes[i], movingCenter, movingHalf, sweep,
                    staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;
            }

            for (int i = 0; i < 3; i++)
            {
                if (!SweepTestAxis(worldAxes[i], movingCenter, movingHalf, sweep,
                    staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                    return false;
            }

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vector3 crossAxis = Vector3.Cross(obbAxes[i], worldAxes[j]);
                    if (crossAxis.LengthSquared() <= 1e-8f) continue;
                    crossAxis = Vector3.Normalize(crossAxis);

                    if (!SweepTestAxis(crossAxis, movingCenter, movingHalf, sweep,
                        staticObb.Center, staticObb.HalfExtents, obbAxes, ref enter, ref exit))
                        return false;
                }
            }

            if (exit < 0.0 || enter > 1.0) return false;

            hitFraction = Math.Clamp(enter, 0.0, 1.0);
            return true;
        }

        private static bool SweepTestAxis(
            Vector3 axis,
            Vector3 movingCenter,
            Vector3 movingHalf,
            Vector3 sweep,
            Vector3 staticCenter,
            Vector3 staticHalf,
            Vector3[] staticAxes,
            ref double enter,
            ref double exit)
        {
            const double epsilon = 1e-9;

            double movingProjection = Vector3.Dot(movingCenter, axis);
            double staticProjection = Vector3.Dot(staticCenter, axis);
            double velocityProjection = Vector3.Dot(sweep, axis);

            double movingRadius =
                Math.Abs(axis.X) * movingHalf.X +
                Math.Abs(axis.Y) * movingHalf.Y +
                Math.Abs(axis.Z) * movingHalf.Z;

            double staticRadius =
                Math.Abs(Vector3.Dot(axis, staticAxes[0])) * staticHalf.X +
                Math.Abs(Vector3.Dot(axis, staticAxes[1])) * staticHalf.Y +
                Math.Abs(Vector3.Dot(axis, staticAxes[2])) * staticHalf.Z;

            double movingMin = movingProjection - movingRadius;
            double movingMax = movingProjection + movingRadius;
            double staticMin = staticProjection - staticRadius;
            double staticMax = staticProjection + staticRadius;

            if (Math.Abs(velocityProjection) <= epsilon)
                return movingMax >= staticMin && movingMin <= staticMax;

            double axisEnter, axisExit;

            if (velocityProjection > 0.0)
            {
                axisEnter = (staticMin - movingMax) / velocityProjection;
                axisExit = (staticMax - movingMin) / velocityProjection;
            }
            else
            {
                axisEnter = (staticMax - movingMin) / velocityProjection;
                axisExit = (staticMin - movingMax) / velocityProjection;
            }

            if (axisEnter > enter) enter = axisEnter;
            if (axisExit < exit) exit = axisExit;

            return enter <= exit;
        }

        private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
        {
            if (value.LengthSquared() <= 1e-10f) return fallback;
            return Vector3.Normalize(value);
        }
    }
}