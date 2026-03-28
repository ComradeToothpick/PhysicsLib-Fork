using BepuWrapper.Entities.Behaviours;
using HarmonyLib;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;

namespace BepuWrapper.patches
{
    //[HarmonyPatch(typeof(PModuleOnGround), nameof(PModuleOnGround.DoApply))]
    //public static class PModuleOnGround_DoApply_CarryMotion
    //{
    //    [HarmonyPostfix]
    //    public static void Postfix(float dt, Entity entity, EntityPos pos)
    //    {
    //        if (!(entity is EntityPlayer player)) return;

    //        var api = entity.Api;
    //        var physics = api.ModLoader.GetModSystem<BepuWrapperModSystem>();

    //        if (physics == null) return;

    //        if (CollisionTester_ApplyTerrainCollision_Patch.TryGetStandingOnEntity(entity, out Entity supportEntity))
    //        {
    //            Vec3f supportVelocity = supportEntity.GetBehavior<BepuPhysicsBehaviour>().velocity;
    //            // Apply carry motion
    //            pos.Motion.X += supportVelocity.X;
    //            pos.Motion.Z += supportVelocity.Z;

    //            // Optional: Y if elevators/boats bobbing
    //            pos.Motion.Y += supportVelocity.Y;
    //        }
    //    }
    //}
}
