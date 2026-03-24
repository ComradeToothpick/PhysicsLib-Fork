using HarmonyLib;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;

namespace BepuWrapper.patches
{
    [HarmonyPatch(typeof(PModuleInAir), "DoApply")]
    public static class PModuleInAir_DoApply_Patch
    {
        static void Prefix(float dt, Entity entity, EntityPos pos, EntityControls controls)
        {
            if (entity.WatchedAttributes.GetInt("physcoll") == 1) return;
        }
    }
}
