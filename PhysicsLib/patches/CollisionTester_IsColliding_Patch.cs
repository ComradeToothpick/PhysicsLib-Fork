using HarmonyLib;
using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;

namespace PhysicsLib.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.IsColliding))]
    public static class CollisionTester_IsColliding_Patch
    {
        public static IDynamicCollisionSource? DynamicCollisionSource;

        [HarmonyPostfix]
        public static void Postfix(
            IBlockAccessor blockAccessor,
            Cuboidf entityBoxRel,
            Vec3d pos,
            bool alsoCheckTouch,
            ref bool __result)
        {
            if (__result || DynamicCollisionSource == null)
                return;

            Cuboidd queryBox = entityBoxRel.ToDouble();
            queryBox.Translate(pos);

            List<DynamicCollisionBox> dynamicBoxes = new();

            DynamicCollisionSource.CollectCollisionBoxes(
                null!,
                queryBox,
                ref dynamicBoxes
            );

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                if (IntersectsObbAabb(dynamicBoxes[i], queryBox))
                {
                    __result = true;
                    return;
                }
            }
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
    }
}