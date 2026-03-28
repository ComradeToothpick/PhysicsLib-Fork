using BepuWrapper.Api;
using BepuWrapper.Api.CollisionSource;
using HarmonyLib;
using System;
using System.Collections.Generic;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace BepuWrapper.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.ApplyTerrainCollision))]
    public static class CollisionTester_ApplyTerrainCollision_Patch
    {
        public static IBepuDynamicCollisionSource DynamicCollisionSource;

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

            newPosition.Set(
                pos.X + motionX,
                pos.Y + motionY,
                pos.Z + motionZ
            );
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
