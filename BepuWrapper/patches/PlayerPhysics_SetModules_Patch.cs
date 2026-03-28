using BepuWrapper.Entities;
using HarmonyLib;
using System.Collections.Generic;
using Vintagestory.API.Common.Entities;
using Vintagestory.GameContent;

namespace BepuWrapper.patches
{
    [HarmonyPatch(typeof(EntityBehaviorPlayerPhysics), "SetModules")]
    public static class PlayerPhysics_SetModules_Patch
    {
        static void Postfix(EntityBehaviorPlayerPhysics __instance)
        {
            var modules = (List<PModule>)AccessTools
                .Field(typeof(EntityBehaviorPlayerPhysics), "physicsModules")
                .GetValue(__instance);

            modules.Add(new PModuleRigidbody());
        }
    }
}
