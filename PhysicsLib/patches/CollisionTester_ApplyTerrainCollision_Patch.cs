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

        private class SupportState
        {
            public Entity? SupportEntity;
            public int GraceTicks;
        }

        private static readonly Dictionary<long, SupportState> SupportStates = new();

        // 20 is quite sticky. Reduce it.
        private const int SupportGraceTicks = 6;

        // Tighten these. Your old values were far too generous for jumping.
        private const double SupportRetainHorizontalTolerance = 0.08;
        private const double SupportRetainVerticalToleranceAbove = 0.08;
        private const double SupportRetainVerticalToleranceBelow = 0.12;

        private const double SupportSnapToleranceAbove = 0.08;
        private const double SupportSnapToleranceBelow = 0.12;

        public static bool TryGetStandingOnEntity(Entity entity, out Entity standingOnEntity)
        {
            standingOnEntity = null!;

            if (entity == null) return false;

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState? state) || state == null)
                return false;

            if (state.SupportEntity == null)
            {
                SupportStates.Remove(entity.EntityId);
                return false;
            }

            standingOnEntity = state.SupportEntity;
            return true;
        }

        private static bool TryGetSupportTopY(
            Entity supportEntity,
            List<DynamicCollisionBox> dynamicBoxes,
            Cuboidd entityBox,
            out double supportTopY)
        {
            supportTopY = 0.0;
            bool found = false;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox box = dynamicBoxes[i];
                if (box.SourceEntity != supportEntity || !box.CanSupport)
                    continue;

                Cuboidd b = box.Box;
                if (b == null)
                    continue;

                bool overlapsHorizontally =
                    entityBox.X2 > b.X1 &&
                    entityBox.X1 < b.X2 &&
                    entityBox.Z2 > b.Z1 &&
                    entityBox.Z1 < b.Z2;

                if (!overlapsHorizontally)
                    continue;

                if (!found || b.Y2 > supportTopY)
                {
                    supportTopY = b.Y2;
                    found = true;
                }
            }

            return found;
        }

        public static void ClearStandingOnEntity(Entity entity)
        {
            if (entity == null) return;
            SupportStates.Remove(entity.EntityId);
        }

        private static void SetStandingOnEntity(Entity entity, Entity standingOnEntity)
        {
            if (entity == null) return;

            if (standingOnEntity == null)
            {
                SupportStates.Remove(entity.EntityId);
                return;
            }

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState? state) || state == null)
            {
                state = new SupportState();
                SupportStates[entity.EntityId] = state;
            }

            state.SupportEntity = standingOnEntity;
            state.GraceTicks = SupportGraceTicks;
        }

        private static void DecayStandingOnEntity(Entity entity)
        {
            if (entity == null) return;

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState? state) || state == null)
                return;

            state.GraceTicks--;
            if (state.GraceTicks <= 0 || state.SupportEntity == null)
            {
                SupportStates.Remove(entity.EntityId);
            }
        }

        private static bool IsEntityStillContainedBySupport(
            Entity entity,
            Entity supportEntity,
            List<DynamicCollisionBox> dynamicBoxes,
            Cuboidd entityBox)
        {
            if (entity == null || supportEntity == null || dynamicBoxes == null || dynamicBoxes.Count == 0)
                return false;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox box = dynamicBoxes[i];
                if (box.SourceEntity != supportEntity)
                    continue;

                Cuboidd b = box.Box;
                if (b == null)
                    continue;

                bool overlapsHorizontally =
                    entityBox.X2 > b.X1 - SupportRetainHorizontalTolerance &&
                    entityBox.X1 < b.X2 + SupportRetainHorizontalTolerance &&
                    entityBox.Z2 > b.Z1 - SupportRetainHorizontalTolerance &&
                    entityBox.Z1 < b.Z2 + SupportRetainHorizontalTolerance;

                if (!overlapsHorizontally)
                    continue;

                double feetY = entityBox.Y1;
                double topY = b.Y2;
                double verticalOffset = feetY - topY;

                if (verticalOffset >= -SupportRetainVerticalToleranceBelow &&
                    verticalOffset <= SupportRetainVerticalToleranceAbove)
                {
                    return true;
                }
            }

            return false;
        }

        private static Vec3d GetCarryPoint(Cuboidd entityBox)
        {
            return new Vec3d(
                (entityBox.X1 + entityBox.X2) * 0.5,
                entityBox.Y1,
                (entityBox.Z1 + entityBox.Z2) * 0.5
            );
        }

        // Use yaw only. Full quaternion on movement is wrong for a deck/floor carrier.
        private static Quaternion ExtractYawOnly(Quaternion q)
        {
            double sinyCosp = 2.0 * (q.W * q.Y + q.X * q.Z);
            double cosyCosp = 1.0 - 2.0 * (q.Y * q.Y + q.Z * q.Z);
            float yaw = (float)Math.Atan2(sinyCosp, cosyCosp);
            return Quaternion.CreateFromAxisAngle(Vector3.UnitY, yaw);
        }

        private static bool ShouldStickToSupport(double feetY, double supportTopY, EntityPos entityPos, double motionY)
        {
            double verticalOffset = feetY - supportTopY;

            // Do not stick while the entity is moving upward / jumping.
            if (entityPos.Motion.Y > 0.001 || motionY > 0.001)
                return false;

            return verticalOffset >= -SupportSnapToleranceBelow &&
                   verticalOffset <= SupportSnapToleranceAbove;
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

            if (!dynamicBox.CanSupport || dynamicBox.Box == null)
                return false;

            // Ignore the support floor in X/Z resolution when we're essentially standing on it.
            return entityBox.Y1 >= dynamicBox.Box.Y2 - 0.05;
        }

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

            pos.X = entityPos.X;
            pos.Y = entityPos.Y;
            pos.Z = entityPos.Z;

            EnumPushDirection direction = EnumPushDirection.None;

            entityBox.SetAndTranslate(entity.CollisionBox, pos.X, pos.Y, pos.Z);

            Entity previousSupportEntity = null!;
            bool hadPreviousSupport = TryGetStandingOnEntity(entity, out previousSupportEntity) && previousSupportEntity != null;

            if (hadPreviousSupport)
            {
                DynamicPhysicsBehaviour supportPhysics = previousSupportEntity.GetBehavior<DynamicPhysicsBehaviour>()!;
                if (supportPhysics != null)
                {
                    Vec3d carryPoint = GetCarryPoint(entityBox);

                    if (supportPhysics.TryGetCarryDeltaForPoint(carryPoint, out Vec3d carryDelta))
                    {
                        pos.X += carryDelta.X;
                        pos.Y += carryDelta.Y;
                        pos.Z += carryDelta.Z;

                        entityBox.SetAndTranslate(entity.CollisionBox, pos.X, pos.Y, pos.Z);
                    }

                    // IMPORTANT: only apply yaw rotation to player movement.
                    if (entity is EntityPlayer && supportPhysics.TryGetCarryRotationDelta(out Quaternion rotationDelta))
                    {
                        Quaternion yawOnly = ExtractYawOnly(rotationDelta);

                        Vector3 horizontalMotion = new Vector3(
                            (float)entityPos.Motion.X,
                            0f,
                            (float)entityPos.Motion.Z
                        );

                        horizontalMotion = Vector3.Transform(horizontalMotion, yawOnly);

                        entityPos.Motion.X = horizontalMotion.X;
                        entityPos.Motion.Z = horizontalMotion.Z;
                    }
                }
            }

            double motionX = entityPos.Motion.X * dtFactor;
            double motionY = entityPos.Motion.Y * dtFactor;
            double motionZ = entityPos.Motion.Z * dtFactor;

            double motionBiasThreshold = 0.0001;
            double motionBiasX = 0.0;
            double motionBiasY = 0.0;
            double motionBiasZ = 0.0;

            if (motionX > motionBiasThreshold) motionBiasX = motionBiasThreshold;
            if (motionX < -motionBiasThreshold) motionBiasX = -motionBiasThreshold;
            if (motionY > motionBiasThreshold) motionBiasY = motionBiasThreshold;
            if (motionY < -motionBiasThreshold) motionBiasY = -motionBiasThreshold;
            if (motionZ > motionBiasThreshold) motionBiasZ = motionBiasThreshold;
            if (motionZ < -motionBiasThreshold) motionBiasZ = -motionBiasThreshold;

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

            float dynamicQueryStepHeight = stepHeight;
            float dynamicQueryYExtra = yExtra;

            if (hadPreviousSupport)
            {
                dynamicQueryStepHeight = Math.Max(dynamicQueryStepHeight, 2f);
                dynamicQueryYExtra = Math.Max(dynamicQueryYExtra, 2f);
            }

            List<DynamicCollisionBox> dynamicBoxes = CollectDynamicCollisionBoxes(
                entity,
                entityPos.Dimension,
                entityBox,
                motionX,
                motionY,
                motionZ,
                dynamicQueryStepHeight,
                dynamicQueryYExtra
            );

            // Early snap only if descending / stationary vertically.
            if (hadPreviousSupport)
            {
                if (TryGetSupportTopY(previousSupportEntity, dynamicBoxes, entityBox, out double supportTopY))
                {
                    double feetY = entityBox.Y1;

                    if (ShouldStickToSupport(feetY, supportTopY, entityPos, motionY))
                    {
                        double snapDelta = supportTopY - feetY;
                        pos.Y += snapDelta;
                        entityBox.Translate(0.0, snapDelta, 0.0);

                        if (entityPos.Motion.Y < 0.0)
                        {
                            entityPos.Motion.Y = 0.0;
                        }

                        if (motionY < 0.0)
                        {
                            motionY = 0.0;
                        }
                    }
                }
            }

            bool collidedVertically = false;
            bool standingOnDynamicEntity = false;
            Entity supportEntity = null!;

            int terrainCount = tester.CollisionBoxList.Count;
            Cuboidd[] terrainBoxes = tester.CollisionBoxList.cuboids;
            BlockPos collBlockPos = new BlockPos(entityPos.Dimension);

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

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                EnumPushDirection dynamicDirection = EnumPushDirection.None;
                double pushedY = dynamicBoxes[i].Box.pushOutY(entityBox, motionY, ref dynamicDirection);

                if (dynamicDirection != EnumPushDirection.None)
                {
                    motionY = pushedY;
                    collidedVertically = true;

                    if (dynamicDirection == EnumPushDirection.Negative && dynamicBoxes[i].CanSupport)
                    {
                        standingOnDynamicEntity = true;
                        supportEntity = dynamicBoxes[i].SourceEntity;
                        entity.OnGround = true;
                    }
                }
            }

            entityBox.Translate(0.0, motionY, 0.0);
            entity.CollidedVertically = collidedVertically;

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

            bool collidedHorizontally = false;

            Entity resolvedSupportEntity = null!;

            bool keepPreviousSupport =
                !standingOnDynamicEntity &&
                hadPreviousSupport &&
                entityPos.Motion.Y <= 0.001 && // do not retain support while moving upward
                IsEntityStillContainedBySupport(entity, previousSupportEntity, dynamicBoxes, entityBox);

            if (standingOnDynamicEntity && supportEntity != null)
            {
                resolvedSupportEntity = supportEntity;
                SetStandingOnEntity(entity, supportEntity);
            }
            else if (keepPreviousSupport)
            {
                resolvedSupportEntity = previousSupportEntity;
                SetStandingOnEntity(entity, previousSupportEntity);
            }
            else
            {
                DecayStandingOnEntity(entity);
            }

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
                    if (ShouldIgnoreHorizontalDynamicCollision(dynamicBoxes[i], resolvedSupportEntity, entityBox))
                        continue;

                    EnumPushDirection dynamicDirection = EnumPushDirection.None;
                    double pushedX = dynamicBoxes[i].Box.pushOutX(entityBox, motionX, ref dynamicDirection);

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
                    if (ShouldIgnoreHorizontalDynamicCollision(dynamicBoxes[i], resolvedSupportEntity, entityBox))
                        continue;

                    EnumPushDirection dynamicDirection = EnumPushDirection.None;
                    double pushedZ = dynamicBoxes[i].Box.pushOutZ(entityBox, motionZ, ref dynamicDirection);

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

            // Final clamp only when genuinely settling onto support.
            if (resolvedSupportEntity != null)
            {
                Cuboidd finalEntityBox = entity.CollisionBox.ToDouble().OffsetCopy(finalX, finalY, finalZ);

                float finalQueryStepHeight = Math.Max(stepHeight, 2f);
                float finalQueryYExtra = Math.Max(yExtra, 2f);

                List<DynamicCollisionBox> finalDynamicBoxes = CollectDynamicCollisionBoxes(
                    entity,
                    entityPos.Dimension,
                    finalEntityBox,
                    0.0,
                    0.0,
                    0.0,
                    finalQueryStepHeight,
                    finalQueryYExtra
                );

                if (TryGetSupportTopY(resolvedSupportEntity, finalDynamicBoxes, finalEntityBox, out double supportTopY))
                {
                    double feetY = finalEntityBox.Y1;

                    if (ShouldStickToSupport(feetY, supportTopY, entityPos, motionY))
                    {
                        double clampDelta = supportTopY - feetY;
                        finalY += clampDelta;

                        if (entityPos.Motion.Y < 0.0)
                        {
                            entityPos.Motion.Y = 0.0;
                        }

                        if (motionY < 0.0)
                        {
                            motionY = 0.0;
                        }

                        entity.CollidedVertically = true;
                        entity.OnGround = true;
                    }
                }
            }

            newPosition.Set(finalX, finalY, finalZ);
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