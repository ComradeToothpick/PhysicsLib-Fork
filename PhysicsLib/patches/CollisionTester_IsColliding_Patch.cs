using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using HarmonyLib;
using System.Collections.Generic;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;

namespace PhysicsLib.patches
{
    [HarmonyPatch(typeof(CollisionTester), nameof(CollisionTester.IsColliding))]
    public static class CollisionTester_IsColliding_Patch
    {
        public static IBepuDynamicCollisionSource? DynamicCollisionSource;

        [HarmonyPostfix]
        public static void Postfix(
            IBlockAccessor blockAccessor,
            Cuboidf entityBoxRel,
            Vec3d pos,
            bool alsoCheckTouch,
            ref bool __result)
        {
            if (__result)
                return;

            if (DynamicCollisionSource == null)
                return;

            Cuboidd queryBox = entityBoxRel.ToDouble();
            queryBox.Translate(pos);

            List<DynamicCollisionBox> dynamicBoxes =
                new List<DynamicCollisionBox>();

            DynamicCollisionSource.CollectCollisionBoxes(
                null!,
                queryBox,
                dynamicBoxes
            );

            for (int i = 0; i < dynamicBoxes.Count; i++)
            {
                Cuboidd otherBox = dynamicBoxes[i].Box;

                bool hit = alsoCheckTouch
                    ? otherBox.IntersectsOrTouches(queryBox)
                    : otherBox.Intersects(queryBox);

                if (hit)
                {
                    __result = true;
                    return;
                }
            }
        }
    }
}
