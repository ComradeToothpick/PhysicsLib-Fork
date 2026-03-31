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
            public Cuboidd Box = null!;
            public double TopY;
        }

        private static readonly Dictionary<long, SupportState> SupportStates = new();

        private const double MotionBiasThreshold = 0.0001;

        // support snap / retain
        private const double SupportSnapAbove = 0.08;
        private const double SupportSnapBelow = 0.18;

        // only ignore X/Z against support boxes that are clearly floor-ish relative to feet
        private const double FloorIgnoreTopTolerance = 0.06;

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
            var supportEntity = entity.World.GetEntityById(supportState.SupportEntityId);

            return supportEntity;
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

            // Carry before collision resolution, but by frame delta only.
            // This keeps you on the boat without turning edge rotation into insane speed.
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

            // Y first - vanilla style
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

                double pushedY = dynamicBox.Box.pushOutY(entityBox, motionY, ref dynamicDirection);

                if (dynamicDirection == EnumPushDirection.None)
                    continue;

                motionY = pushedY;
                collidedVertically = true;

                if (dynamicDirection == EnumPushDirection.Negative && dynamicBox.CanSupport)
                {
                    if (standingSupport == null || dynamicBox.Box.Y2 > standingSupport.TopY)
                    {
                        standingSupport = new SupportCandidate
                        {
                            SupportEntity = dynamicBox.SourceEntity,
                            Box = dynamicBox.Box,
                            TopY = dynamicBox.Box.Y2
                        };
                    }
                }
            }

            entityBox.Translate(0.0, motionY, 0.0);
            entity.CollidedVertically = collidedVertically;

            // Supplemental support snap for descending/stationary vertical motion
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
                    if (dynamicBoxes[i].Box.Intersects(entityBox))
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
                    double pushedX = dynamicBox.Box.pushOutX(entityBox, motionX, ref dynamicDirection);

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
                    double pushedZ = dynamicBox.Box.pushOutZ(entityBox, motionZ, ref dynamicDirection);

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

            Cuboidd b = dynamicBox.Box;

            // Only ignore support boxes whose top is at or below the rider's feet.
            // This preserves wall/blocking collisions inside the boat.
            return b.Y2 <= entityBox.Y1 + FloorIgnoreTopTolerance;
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
            double feetY = entityBox.Y1;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox dyn = dynamicBoxes[i];
                if (!dyn.CanSupport)
                    continue;

                Cuboidd b = dyn.Box;

                bool overlapsHorizontally =
                    entityBox.X2 > b.X1 &&
                    entityBox.X1 < b.X2 &&
                    entityBox.Z2 > b.Z1 &&
                    entityBox.Z1 < b.Z2;

                if (!overlapsHorizontally)
                    continue;

                double delta = feetY - b.Y2;
                if (delta < -SupportSnapBelow || delta > SupportSnapAbove)
                    continue;

                if (best == null || b.Y2 > best.TopY)
                {
                    best = new SupportCandidate
                    {
                        SupportEntity = dyn.SourceEntity,
                        Box = b,
                        TopY = b.Y2
                    };
                }
            }

            return best;
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
    }
}