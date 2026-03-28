using BepuWrapper.Api;
using BepuWrapper.Api.CollisionSource;
using BepuWrapper.Entities.Behaviours;
using HarmonyLib;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace BepuWrapper.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.ApplyTerrainCollision))]
    public static class CollisionTester_ApplyTerrainCollision_Patch
    {
        public static IBepuDynamicCollisionSource DynamicCollisionSource;

        private class SupportState
        {
            public Entity SupportEntity;
            public int GraceTicks;
        }

        private static readonly Dictionary<long, SupportState> SupportStates = new();
        private const int SupportGraceTicks = 20;

        public static bool TryGetStandingOnEntity(Entity entity, out Entity standingOnEntity)
        {
            standingOnEntity = null;

            if (entity == null)
                return false;

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState state) || state == null)
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
            if (entity == null)
                return;

            SupportStates.Remove(entity.EntityId);
        }

        private static void SetStandingOnEntity(Entity entity, Entity standingOnEntity)
        {
            if (entity == null)
                return;

            if (standingOnEntity == null)
            {
                SupportStates.Remove(entity.EntityId);
                return;
            }

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState state) || state == null)
            {
                state = new SupportState();
                SupportStates[entity.EntityId] = state;
            }

            state.SupportEntity = standingOnEntity;
            state.GraceTicks = SupportGraceTicks;
        }

        private static void DecayStandingOnEntity(Entity entity)
        {
            if (entity == null)
                return;

            if (!SupportStates.TryGetValue(entity.EntityId, out SupportState state) || state == null)
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

            const double horizontalTolerance = 0.1;
            const double verticalToleranceAbove = 0.35;
            const double verticalToleranceBelow = 0.25;

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                DynamicCollisionBox box = dynamicBoxes[i];
                if (box.SourceEntity != supportEntity)
                    continue;

                Cuboidd b = box.Box;
                if (b == null)
                    continue;

                bool overlapsHorizontally =
                    entityBox.X2 > b.X1 - horizontalTolerance &&
                    entityBox.X1 < b.X2 + horizontalTolerance &&
                    entityBox.Z2 > b.Z1 - horizontalTolerance &&
                    entityBox.Z1 < b.Z2 + horizontalTolerance;

                if (!overlapsHorizontally)
                    continue;

                bool overlapsVertically =
                    entityBox.Y2 > b.Y1 - verticalToleranceBelow &&
                    entityBox.Y1 < b.Y2 + verticalToleranceAbove;

                if (overlapsVertically)
                    return true;

                double feetY = entityBox.Y1;
                double topY = b.Y2;
                double verticalOffset = feetY - topY;

                if (verticalOffset >= -verticalToleranceBelow && verticalOffset <= verticalToleranceAbove)
                    return true;
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

            if (TryGetStandingOnEntity(entity, out Entity previousSupportEntity) && previousSupportEntity != null)
            {
                BepuPhysicsBehaviour supportPhysics = previousSupportEntity.GetBehavior<BepuPhysicsBehaviour>();
                if (supportPhysics != null)
                {
                    Vec3d carryPoint = GetCarryPoint(entityBox);
                    Vec3d carryDelta;

                    if (supportPhysics.TryGetCarryDeltaForPoint(carryPoint, out carryDelta))
                    {
                        pos.X += carryDelta.X;
                        pos.Y += carryDelta.Y;
                        pos.Z += carryDelta.Z;

                        entityBox.SetAndTranslate(entity.CollisionBox, pos.X, pos.Y, pos.Z);
                    }

                    if (supportPhysics.TryGetCarryRotationDelta(out Quaternion rotationDelta))
                    {
                        Vector3 horizontalMotion = new Vector3(
                            (float)entityPos.Motion.X,
                            0f,
                            (float)entityPos.Motion.Z
                        );

                        horizontalMotion = Vector3.Transform(horizontalMotion, rotationDelta);

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

            if (TryGetStandingOnEntity(entity, out Entity querySupportEntity) && querySupportEntity != null)
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

            if (TryGetStandingOnEntity(entity, out Entity snapSupportEntity) && snapSupportEntity != null)
            {
                if (TryGetSupportTopY(snapSupportEntity, dynamicBoxes, entityBox, out double supportTopY))
                {
                    double feetY = entityBox.Y1;
                    double snapToleranceAbove = 0.25;
                    double snapToleranceBelow = 0.1;

                    double verticalOffset = feetY - supportTopY;

                    if (verticalOffset >= -snapToleranceBelow && verticalOffset <= snapToleranceAbove)
                    {
                        double snapDelta = supportTopY - feetY;
                        pos.Y += snapDelta;
                        entityBox.Translate(0.0, snapDelta, 0.0);

                        if (entityPos.Motion.Y < 0.0)
                        {
                            entityPos.Motion.Y = 0.0;
                        }
                    }
                }
            }

            bool collidedVertically = false;
            bool standingOnDynamicEntity = false;
            Entity supportEntity = null;

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

            bool keepPreviousSupport =
                !standingOnDynamicEntity &&
                previousSupportEntity != null &&
                IsEntityStillContainedBySupport(entity, previousSupportEntity, dynamicBoxes, entityBox);

            Entity resolvedSupportEntity = null;

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

            double finalX = pos.X + motionX;
            double finalY = pos.Y + motionY;
            double finalZ = pos.Z + motionZ;

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