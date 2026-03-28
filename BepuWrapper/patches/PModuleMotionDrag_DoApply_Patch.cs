using HarmonyLib;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;

namespace BepuWrapper.patches
{
    [HarmonyPatch(typeof(PModuleMotionDrag), "DoApply")]
    public static class PModuleMotionDrag_DoApply_Patch
    {
        static void Prefix(float dt, Entity entity, EntityPos pos, EntityControls controls)
        {
            if (entity.WatchedAttributes.GetInt("physcoll") == 1) return;
        }
    }
}
