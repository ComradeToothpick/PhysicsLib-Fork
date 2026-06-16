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
        private const double SupportSnapAbove = 0.08;
        private const double SupportSnapBelow = 0.18;
        private const double FloorIgnoreTopTolerance = 0.06;
        private const double PreviousSupportRetainAbove = 0.12;
        private const double PreviousSupportRetainBelow = 0.30;
        private const double SupportHorizontalPadding = 0.03;

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

            Entity? previousSupportEntity = ResolveSupportEntity(entity, out SupportState? previousSupportState);
            DynamicPhysicsBehaviour? previousSupportPhysics = previousSupportEntity?.GetBehavior<DynamicPhysicsBehaviour>();

            if (previousSupportEntity != null && previousSupportPhysics != null && previousSupportState != null)
            {
                if (previousSupportPhysics.TryGetPointVelocityDelta(previousSupportState.LocalAnchorPoint, out Vec3d carryDelta))
                {
                    pos.X += carryDelta.X;
                    pos.Y += carryDelta.Y;
                    pos.Z += carryDelta.Z;

                    entityBox.Translate(carryDelta.X, carryDelta.Y, carryDelta.Z);
                }
            }

            double motionX = entityPos.Motion.X * dtFactor;
            double motionY = entityPos.Motion.Y * dtFactor;
            double motionZ = entityPos.Motion.Z * dtFactor;

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
            EnumPushDirection direction = EnumPushDirection.None;

            int terrainCount = tester.CollisionBoxList.Count;
            Cuboidd[] terrainBoxes = tester.CollisionBoxList.cuboids;

            for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
            {
                motionY = terrainBoxes[i].pushOutY(entityBox, motionY, ref direction);

                if (direction != EnumPushDirection.None)
                {
                    collidedVertically = true;
                    collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                    tester.CollisionBoxList.blocks[i].OnEntityCollide(
                        world,
                        entity,
                        collBlockPos,
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

                if (dynamicDirection == EnumPushDirection.Negative && dynamicBox.CanSupport)
                {
                    double contactTopY = entityBox.Y1 + pushedY;

                    if (standingSupport == null || contactTopY > standingSupport.TopY)
                    {
                        standingSupport = new SupportCandidate
                        {
                            SupportEntity = dynamicBox.SourceEntity,
                            DynamicBox = dynamicBox,
                            TopY = contactTopY
                        };
                    }
                }
            }

            entityBox.Translate(0.0, motionY, 0.0);
            entity.CollidedVertically = collidedVertically;

            if (entityPos.Motion.Y <= 0.001)
            {
                SupportCandidate? nearbySupport = FindBestSupportBelowFeet(entityBox, dynamicBoxes);
                if (nearbySupport != null)
                {
                    double feetY = entityBox.Y1;
                    double snapDelta = nearbySupport.TopY - feetY;

                    if (snapDelta >= -SupportSnapBelow && snapDelta <= SupportSnapAbove)
                    {
                        entityBox.Translate(0.0, snapDelta, 0.0);
                        motionY += snapDelta;

                        if (entityPos.Motion.Y < 0.0)
                        {
                            entityPos.Motion.Y = 0.0;
                        }

                        standingSupport = nearbySupport;
                        entity.CollidedVertically = true;
                    }
                }
            }

            bool horizontalIntersects = false;

            entityBox.Translate(motionX, 0.0, motionZ);

            foreach (Cuboidd terrainBox in tester.CollisionBoxList)
            {
                if (terrainBox.Intersects(entityBox))
                {
                    horizontalIntersects = true;
                    break;
                }
            }

            if (!horizontalIntersects)
            {
                for (int i = 0; i < dynamicBoxes.Count; i++)
                {
                    if (IntersectsObbAabb(dynamicBoxes[i], entityBox))
                    {
                        horizontalIntersects = true;
                        break;
                    }
                }
            }

            entityBox.Translate(-motionX, 0.0, -motionZ);

            Entity? resolvedSupportEntity = standingSupport?.SupportEntity ?? previousSupportEntity;

            if (horizontalIntersects)
            {
                for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
                {
                    motionX = terrainBoxes[i].pushOutX(entityBox, motionX, ref direction);

                    if (direction != EnumPushDirection.None)
                    {
                        collidedHorizontally = true;
                        collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                        tester.CollisionBoxList.blocks[i].OnEntityCollide(
                            world,
                            entity,
                            collBlockPos,
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

                for (int i = 0; i < terrainBoxes.Length && i < terrainCount; i++)
                {
                    motionZ = terrainBoxes[i].pushOutZ(entityBox, motionZ, ref direction);

                    if (direction != EnumPushDirection.None)
                    {
                        collidedHorizontally = true;
                        collBlockPos.Set(tester.CollisionBoxList.positions[i]);

                        tester.CollisionBoxList.blocks[i].OnEntityCollide(
                            world,
                            entity,
                            collBlockPos,
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
            }

            entity.CollidedHorizontally = collidedHorizontally;

            if (motionY > 0.0 && entity.CollidedVertically)
            {
                motionY -= entity.LadderFixDelta;
            }

            motionX -= motionBiasX;
            motionY -= motionBiasY;
            motionZ -= motionBiasZ;

            double finalX = pos.X + motionX;
            double finalY = pos.Y + motionY;
            double finalZ = pos.Z + motionZ;

            Cuboidd finalEntityBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);

            SupportCandidate? finalSupport = FindBestSupportBelowFeet(
                finalEntityBox,
                CollectDynamicCollisionBoxes(
                    entity,
                    entityPos.Dimension,
                    finalEntityBox,
                    0.0,
                    0.0,
                    0.0,
                    stepHeight,
                    yExtra
                )
            );

            if (finalSupport == null && previousSupportPhysics != null && previousSupportEntity != null)
            {
                if (previousSupportPhysics.TryGetSupportTopYUnderBox(finalEntityBox, SupportHorizontalPadding, out double retainedTopY))
                {
                    double verticalOffset = finalEntityBox.Y1 - retainedTopY;

                    if (verticalOffset >= -PreviousSupportRetainBelow &&
                        verticalOffset <= PreviousSupportRetainAbove)
                    {
                        finalY -= verticalOffset;
                        finalEntityBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);

                        finalSupport = new SupportCandidate
                        {
                            SupportEntity = previousSupportEntity,
                            DynamicBox = null!,
                            TopY = retainedTopY
                        };

                        if (entityPos.Motion.Y < 0.0)
                        {
                            entityPos.Motion.Y = 0.0;
                        }

                        entity.CollidedVertically = true;
                        entity.OnGround = true;
                    }
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
            if (resolvedSupportEntity == null)
                return false;

            if (dynamicBox.SourceEntity != resolvedSupportEntity)
                return false;

            double supportTopY = FindSupportTopByVerticalSweep(entityBox, dynamicBox);
            return supportTopY <= entityBox.Y1 + FloorIgnoreTopTolerance;
        }

        private static Vec3d GetFeetCenter(Cuboidd entityBox)
        {
            return new Vec3d(
                (entityBox.X1 + entityBox.X2) * 0.5,
                entityBox.Y1,
                (entityBox.Z1 + entityBox.Z2) * 0.5
            );
        }

        private static SupportCandidate? FindBestSupportBelowFeet(Cuboidd entityBox, List<DynamicCollisionBox> dynamicBoxes)
        {
            SupportCandidate? best = null;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dyn = dynamicBoxes[i];
                if (!dyn.CanSupport)
                    continue;

                double supportTopY = FindSupportTopByVerticalSweep(entityBox, dyn);
                double delta = entityBox.Y1 - supportTopY;

                if (delta < -SupportSnapBelow || delta > SupportSnapAbove)
                    continue;

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

        private static double FindSupportTopByVerticalSweep(Cuboidd entityBox, DynamicCollisionBox dynamicBox)
        {
            double downMotion = -SupportSnapBelow;
            EnumPushDirection direction = EnumPushDirection.None;

            double pushedY = PushOutYObbAabb(dynamicBox, entityBox, downMotion, ref direction);

            if (direction == EnumPushDirection.None)
                return double.NegativeInfinity;

            return entityBox.Y1 + pushedY;
        }

        private static void GenerateTerrainCollisionBoxList(
            CollisionTester tester,
            IBlockAccessor blockAccessor,
            double motionX,
            double motionY,
            double motionZ,
            float stepHeight,
            float yExtra,
            int dimension)
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

            if (minUnchanged && maxUnchanged)
                return;

            tester.CollisionBoxList.Clear();
            tester.tmpPos.dimension = dimension;

            blockAccessor.WalkBlocks(
                tester.minPos,
                tester.maxPos,
                (block, x, y, z) =>
                {
                    Cuboidf[] collisionBoxes = block.GetCollisionBoxes(blockAccessor, tester.tmpPos.Set(x, y, z));
                    if (collisionBoxes != null)
                    {
                        tester.CollisionBoxList.Add(collisionBoxes, x, y, z, block);
                    }
                },
                centerOrder: true
            );
        }

        private static List<DynamicCollisionBox> CollectDynamicCollisionBoxes(
            Entity movingEntity,
            int dimension,
            Cuboidd movingEntityBox,
            double motionX,
            double motionY,
            double motionZ,
            float stepHeight,
            float yExtra)
        {
            List<DynamicCollisionBox> results = new List<DynamicCollisionBox>();

            if (DynamicCollisionSource == null)
                return results;

            Cuboidd queryBox = movingEntityBox.Clone();

            queryBox.X1 += Math.Min(0.0, motionX);
            queryBox.Y1 += Math.Min(0.0, motionY) - yExtra;
            queryBox.Z1 += Math.Min(0.0, motionZ);

            queryBox.X2 += Math.Max(0.0, motionX);
            queryBox.Y2 = Math.Max(queryBox.Y1 + stepHeight, queryBox.Y2 + Math.Max(0.0, motionY));
            queryBox.Z2 += Math.Max(0.0, motionZ);

            DynamicCollisionSource.CollectCollisionBoxes(
                movingEntity,
                queryBox,
                results
            );

            return results;
        }

        private static Cuboidd OffsetEntityBox(Cuboidd box, double x, double y, double z)
        {
            return new Cuboidd(
                box.X1 + x,
                box.Y1 + y,
                box.Z1 + z,
                box.X2 + x,
                box.Y2 + y,
                box.Z2 + z
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

            Vector3[] a = new Vector3[3];
            a[0] = Vector3.Normalize(Vector3.Transform(Vector3.UnitX, obb.Orientation));
            a[1] = Vector3.Normalize(Vector3.Transform(Vector3.UnitY, obb.Orientation));
            a[2] = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, obb.Orientation));

            Vector3[] b = new Vector3[3];
            b[0] = Vector3.UnitX;
            b[1] = Vector3.UnitY;
            b[2] = Vector3.UnitZ;

            float[] ea = { obb.HalfExtents.X, obb.HalfExtents.Y, obb.HalfExtents.Z };
            float[] eb = { aabbHalf.X, aabbHalf.Y, aabbHalf.Z };

            float[,] r = new float[3, 3];
            float[,] absR = new float[3, 3];

            const float epsilon = 1e-6f;

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    r[i, j] = Vector3.Dot(a[i], b[j]);
                    absR[i, j] = MathF.Abs(r[i, j]) + epsilon;
                }
            }

            Vector3 tWorld = aabbCenter - obb.Center;

            float[] t =
            {
                Vector3.Dot(tWorld, a[0]),
                Vector3.Dot(tWorld, a[1]),
                Vector3.Dot(tWorld, a[2])
            };

            for (int i = 0; i < 3; i++)
            {
                float ra = ea[i];
                float rb = eb[0] * absR[i, 0] + eb[1] * absR[i, 1] + eb[2] * absR[i, 2];

                if (MathF.Abs(t[i]) > ra + rb)
                    return false;
            }

            for (int j = 0; j < 3; j++)
            {
                float ra = ea[0] * absR[0, j] + ea[1] * absR[1, j] + ea[2] * absR[2, j];
                float rb = eb[j];

                float projectedT = MathF.Abs(
                    t[0] * r[0, j] +
                    t[1] * r[1, j] +
                    t[2] * r[2, j]
                );

                if (projectedT > ra + rb)
                    return false;
            }

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

                    if (projectedT > ra + rb)
                        return false;
                }
            }

            return true;
        }

        private static double PushOutAxisObbAabb(
            DynamicCollisionBox obb,
            Cuboidd entityBox,
            double motion,
            int axis,
            ref EnumPushDirection direction)
        {
            direction = EnumPushDirection.None;

            if (Math.Abs(motion) <= 1e-12)
                return motion;

            Cuboidd movedBox = axis switch
            {
                0 => OffsetEntityBox(entityBox, motion, 0.0, 0.0),
                1 => OffsetEntityBox(entityBox, 0.0, motion, 0.0),
                2 => OffsetEntityBox(entityBox, 0.0, 0.0, motion),
                _ => entityBox
            };

            if (!IntersectsObbAabb(obb, movedBox))
                return motion;

            double safe = 0.0;
            double blocked = motion;

            for (int i = 0; i < 24; i++)
            {
                double mid = (safe + blocked) * 0.5;

                Cuboidd testBox = axis switch
                {
                    0 => OffsetEntityBox(entityBox, mid, 0.0, 0.0),
                    1 => OffsetEntityBox(entityBox, 0.0, mid, 0.0),
                    2 => OffsetEntityBox(entityBox, 0.0, 0.0, mid),
                    _ => entityBox
                };

                if (IntersectsObbAabb(obb, testBox))
                {
                    blocked = mid;
                }
                else
                {
                    safe = mid;
                }
            }

            direction = motion > 0.0
                ? EnumPushDirection.Positive
                : EnumPushDirection.Negative;

            return safe;
        }

        private static double PushOutXObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionX, ref EnumPushDirection direction)
        {
            return PushOutAxisObbAabb(obb, entityBox, motionX, 0, ref direction);
        }

        private static double PushOutYObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionY, ref EnumPushDirection direction)
        {
            return PushOutAxisObbAabb(obb, entityBox, motionY, 1, ref direction);
        }

        private static double PushOutZObbAabb(DynamicCollisionBox obb, Cuboidd entityBox, double motionZ, ref EnumPushDirection direction)
        {
            return PushOutAxisObbAabb(obb, entityBox, motionZ, 2, ref direction);
        }
    }
}