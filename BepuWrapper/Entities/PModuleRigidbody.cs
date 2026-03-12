using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;

namespace BepuWrapper.Entities
{
    internal class PModuleRigidbody : PModule
    {
        public override void Initialize(JsonObject config, Entity entity)
        {
        }

        public override bool Applicable(Entity entity, EntityPos pos, EntityControls controls)
        {
            return true;
        }

        public override void DoApply(float dt, Entity entity, EntityPos pos, EntityControls controls)
        {
            if (entity.Api.Side == EnumAppSide.Client) 
            {
                Console.WriteLine("dooting client");
            } else
            {
                Console.WriteLine("dooting server");
            }
            if (entity.WatchedAttributes.GetInt("physcoll") == 1)
            {
                double motionX = entity.WatchedAttributes.GetDouble("rbodirX");
                double motionY = entity.WatchedAttributes.GetDouble("rbodirY");
                double motionZ = entity.WatchedAttributes.GetDouble("rbodirZ");
                float bodyDeltaX = entity.WatchedAttributes.GetFloat("bodyDeltaX");
                float bodyDeltaY = entity.WatchedAttributes.GetFloat("bodyDeltaY");
                float bodyDeltaZ = entity.WatchedAttributes.GetFloat("bodyDeltaZ");
                bool pushedUp = entity.WatchedAttributes.GetBool("pushedUp");

                Vector3 bodyDelta = new Vector3(bodyDeltaX, bodyDeltaY, bodyDeltaZ);
                pos.Motion.Set(motionX, motionY, motionZ);

                // Equivalent to carrying entity along with moving rigid body if standing on it.
                if (pushedUp && bodyDelta.LengthSquared() > 1e-9f)
                {
                    Vector3 horizontalDelta = new Vector3(bodyDelta.X, 0f, bodyDelta.Z) * 0.5f;
                    Vector3 verticalDelta = new Vector3(0f, bodyDelta.Y, 0f);

                    //pos.Add(horizontalDelta.X + verticalDelta.X, horizontalDelta.Y + verticalDelta.Y, horizontalDelta.Z + verticalDelta.Z);
                    pos.SetPos(pos.X + horizontalDelta.X + verticalDelta.X, pos.Y + horizontalDelta.Y + verticalDelta.Y, pos.Z + horizontalDelta.Z + verticalDelta.Z);
                    pos.Motion.Add(horizontalDelta.X + verticalDelta.X, horizontalDelta.Y + verticalDelta.Y, horizontalDelta.Z + verticalDelta.Z);

                }
                entity.OnGround = true;

                entity.WatchedAttributes.SetInt("physcoll", 0);
            }
        }
    }
}
