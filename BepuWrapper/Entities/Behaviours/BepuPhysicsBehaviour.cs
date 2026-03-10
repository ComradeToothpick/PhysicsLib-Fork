using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;
using Vintagestory.API.Server;
using Vintagestory.Server;

namespace BepuWrapper.Entities.Behaviours
{
    public class BepuPhysicsBehaviour : EntityBehavior
    {
        private ICoreClientAPI capi;

        private ICoreServerAPI sapi;

        private TypedIndex compoundShapeIndex;
        private Compound compound;
        private Vector3 localCenterOfMassOffset;


        // Manual collider proxies for pushing non-BEPU entities out.
        private readonly List<ManualChildBox> manualChildBoxes = new();
        private float compoundBroadphaseRadius;

        public BepuPhysicsBehaviour(Entity entity) : base(entity)
        {
        }

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (entity.World.Side == EnumAppSide.Server)
            {
                sapi = (ICoreServerAPI)entity.Api;

                // Replace this with however you access your simulation/pool wrapper.
                var physics = sapi.ModLoader.GetModSystem<BepuWrapperModSystem>();

                var shape = entity.Properties.Client.Shape; 
                var shapeLoc = shape.Base.Clone();
                shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

                CachedCompound? cachedShape = physics.bepu.TryGetCompoundShape(shapeLoc.Path);

                if (cachedShape == null) 
                {
                    var asset = sapi.Assets.TryGet(shapeLoc);

                    var compoundShape = asset.ToObject<Shape>();

                    //sapi.
                    if (compoundShape == null || compoundShape.Elements == null || compoundShape.Elements.Length == 0)
                    {
                        sapi.Logger.Warning("[bepuwrapper] Entity {0} has no loaded shape elements.", entity.Code);
                        return;
                    }

                    var built = BuildCompoundFromShape(compoundShape, physics.bepu.sim.Shapes, physics.bepu.pool);


                    cachedShape = physics.bepu.AddCompoundShape(shapeLoc.Path, built);
                }
                compoundShapeIndex = cachedShape.Value.CompoundIndex;
                localCenterOfMassOffset = cachedShape.Value.LocalCenterOfMassOffset;

                manualChildBoxes.Clear();
                manualChildBoxes.AddRange(cachedShape.Value.ManualChildBoxes);
                compoundBroadphaseRadius = cachedShape.Value.BroadphaseRadius;

                // Example body creation.
                // Compound.ComputeInertia(..., out centerOfMass) recenters child poses around the COM,
                // so your body pose should usually be placed at entity position + rotated COM offset
                // if you want the rendered model origin and physics origin to stay aligned.
                var bodyPose = new RigidPose(
                    ToBepu(entity.Pos.X, entity.Pos.Y, entity.Pos.Z) + localCenterOfMassOffset,
                    Quaternion.Identity
                );

                var inertia = cachedShape.Value.Inertia;

                var bodyDescription = BodyDescription.CreateDynamic(
                    bodyPose.Position,
                    inertia,
                    compoundShapeIndex,
                    0.05f
                );

                physics.bepu.RegisterEntityBody(entity, bodyDescription, localCenterOfMassOffset);
            }
            else
            {
                capi = (ICoreClientAPI)entity.Api;
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            if (sapi == null || manualChildBoxes.Count == 0) return;
            if (!entity.Alive) return;

            PushNearbyEntitiesOutOfCompound(deltaTime);
        }

        public override string PropertyName() => "bepu-physics";
        private void PushNearbyEntitiesOutOfCompound(float deltaTime)
        {
            // Boat body pose in world space.
            Vector3 boatWorldOrigin = ToBepu(entity.Pos.X, entity.Pos.Y, entity.Pos.Z) + localCenterOfMassOffset;
            Quaternion boatWorldOrientation = Quaternion.Identity;

            // Broadphase query radius around boat.
            float radius = compoundBroadphaseRadius + 2.0f;

            // Replace with your preferred entity query if needed.
            var nearby = entity.World.GetEntitiesAround(
                new Vec3d(entity.Pos.X, entity.Pos.Y, entity.Pos.Z),
                radius,
                radius,
                e => e != null && e.EntityId != entity.EntityId && e.Alive
            );

            if (nearby == null) return;

            foreach (var other in nearby)
            {
                // Skip mounted passengers if desired.
                if (other == entity) continue;

                // Approximate the other entity as a vertical capsule reduced to
                // two sphere samples: feet sphere + torso sphere.
                ResolveEntityAgainstCompound(other, boatWorldOrigin, boatWorldOrientation, deltaTime);
            }
        }

        private void ResolveEntityAgainstCompound(Entity other, Vector3 boatWorldOrigin, Quaternion boatWorldOrientation, float deltaTime)
        {
            // You may want to replace this with the exact collision box dimensions you use elsewhere.
            // This fallback assumes a typical upright entity.
            float halfWidth = 0.35f;
            float height = 1.8f;

            if (other.CollisionBox != null)
            {
                halfWidth = (float)Math.Max(
                    (other.CollisionBox.X2 - other.CollisionBox.X1) * 0.5,
                    (other.CollisionBox.Z2 - other.CollisionBox.Z1) * 0.5
                );
                height = (float)(other.CollisionBox.Y2 - other.CollisionBox.Y1);
            }

            Vector3 basePos = ToBepu(other.Pos.X, other.Pos.Y, other.Pos.Z);

            // Two-sphere capsule approximation.
            float sphereRadius = halfWidth;
            Vector3 p0 = basePos + new Vector3(0f, sphereRadius, 0f);
            Vector3 p1 = basePos + new Vector3(0f, MathF.Max(sphereRadius, height - sphereRadius), 0f);

            Vector3 totalCorrection = Vector3.Zero;
            bool hadHit = false;

            // Multiple iterations help with corner/edge cases.
            for (int iteration = 0; iteration < 2; iteration++)
            {
                Vector3 correction0 = ComputeCompoundPushoutForSphere(p0 + totalCorrection, sphereRadius, boatWorldOrigin, boatWorldOrientation);
                Vector3 correction1 = ComputeCompoundPushoutForSphere(p1 + totalCorrection, sphereRadius, boatWorldOrigin, boatWorldOrientation);

                Vector3 correction = correction0.LengthSquared() > correction1.LengthSquared() ? correction0 : correction1;
                if (correction.LengthSquared() < 1e-8f) break;

                totalCorrection += correction;
                hadHit = true;
            }

            if (!hadHit) return;

            other.SidedPos.X += totalCorrection.X;
            other.SidedPos.Y += totalCorrection.Y + 1;
            other.SidedPos.Z += totalCorrection.Z;

            if (other.GetType() == typeof(EntityPlayer)) 
            {
                EntityPlayer player = (EntityPlayer)other;
                player.SidedPos.X += totalCorrection.X;
                player.SidedPos.Y += totalCorrection.Y + 1;
                player.SidedPos.Z += totalCorrection.Z;
            }

            // Remove inward velocity so they do not immediately re-penetrate.
            //Vector3 motion = new Vector3((float)other.Pos.Motion.X, (float)other.Pos.Motion.Y, (float)other.Pos.Motion.Z);
            //Vector3 normal = SafeNormalize(totalCorrection);

            //float inwardSpeed = Vector3.Dot(motion, normal);
            //if (inwardSpeed < 0f)
            //{
            //    motion -= inwardSpeed * normal;
            //    other.SidedPos.Motion.X = motion.X;
            //    other.SidedPos.Motion.Y = motion.Y;
            //    other.SidedPos.Motion.Z = motion.Z;
            //}

        }

        private static Vector3 SafeNormalize(Vector3 v)
        {
            float lenSq = v.LengthSquared();
            if (lenSq < 1e-10f) return Vector3.UnitY;
            return v / MathF.Sqrt(lenSq);
        }

        private Vector3 ComputeCompoundPushoutForSphere(
            Vector3 sphereCenterWorld,
            float sphereRadius,
            Vector3 boatWorldOrigin,
            Quaternion boatWorldOrientation)
        {
            Vector3 bestPush = Vector3.Zero;
            float bestDepth = 0f;

            Quaternion inverseBoatRotation = Quaternion.Conjugate(boatWorldOrientation);
            Vector3 sphereCenterInBoat = Vector3.Transform(sphereCenterWorld - boatWorldOrigin, inverseBoatRotation);

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                ManualChildBox child = manualChildBoxes[i];

                Vector3 push = ComputeSphereVsChildBoxPushout(
                    sphereCenterInBoat,
                    sphereRadius,
                    child.LocalPosition,
                    child.LocalOrientation,
                    child.HalfExtents
                );

                float depthSq = push.LengthSquared();
                if (depthSq > bestDepth)
                {
                    bestDepth = depthSq;
                    bestPush = push;
                }
            }

            return Vector3.Transform(bestPush, boatWorldOrientation);
        }

        private static Vector3 ComputeSphereVsChildBoxPushout(
            Vector3 sphereCenterInBoat,
            float sphereRadius,
            Vector3 boxLocalPosition,
            Quaternion boxLocalOrientation,
            Vector3 boxHalfExtents)
        {
            Quaternion invBox = Quaternion.Conjugate(boxLocalOrientation);
            Vector3 local = Vector3.Transform(sphereCenterInBoat - boxLocalPosition, invBox);

            Vector3 clamped = new Vector3(
                Math.Clamp(local.X, -boxHalfExtents.X, boxHalfExtents.X),
                Math.Clamp(local.Y, -boxHalfExtents.Y, boxHalfExtents.Y),
                Math.Clamp(local.Z, -boxHalfExtents.Z, boxHalfExtents.Z)
            );

            Vector3 delta = local - clamped;
            float distSq = delta.LengthSquared();

            // Sphere center outside box.
            if (distSq > 1e-10f)
            {
                float dist = MathF.Sqrt(distSq);
                float penetration = sphereRadius - dist;
                if (penetration <= 0f) return Vector3.Zero;

                Vector3 normalLocal = delta / dist;
                Vector3 pushLocal = normalLocal * penetration;
                return Vector3.Transform(pushLocal, boxLocalOrientation);
            }

            // Sphere center inside box: push out through nearest face.
            float dx = boxHalfExtents.X - MathF.Abs(local.X);
            float dy = boxHalfExtents.Y - MathF.Abs(local.Y);
            float dz = boxHalfExtents.Z - MathF.Abs(local.Z);

            Vector3 axisNormal;
            float faceDistance;

            if (dx <= dy && dx <= dz)
            {
                axisNormal = new Vector3(MathF.Sign(local.X == 0f ? 1f : local.X), 0f, 0f);
                faceDistance = dx;
            }
            else if (dy <= dz)
            {
                axisNormal = new Vector3(0f, MathF.Sign(local.Y == 0f ? 1f : local.Y), 0f);
                faceDistance = dy;
            }
            else
            {
                axisNormal = new Vector3(0f, 0f, MathF.Sign(local.Z == 0f ? 1f : local.Z));
                faceDistance = dz;
            }

            Vector3 insidePushLocal = axisNormal * (sphereRadius + faceDistance);
            return Vector3.Transform(insidePushLocal, boxLocalOrientation);
        }
        private static Vector3 ToBepu(double x, double y, double z)
        {
            return new Vector3((float)x, (float)y, (float)z);
        }

        private static Vector3 TransformPoint(Matrix4x4 m, Vector3 p)
        {
            return Vector3.Transform(p, m);
        }

        private static Quaternion ExtractRotation(Matrix4x4 m)
        {
            Matrix4x4.Decompose(m, out _, out var rotation, out _);
            return Quaternion.Normalize(rotation);
        }

        private static Vector3 ExtractScale(Matrix4x4 m)
        {
            Matrix4x4.Decompose(m, out var scale, out _, out _);
            return scale;
        }

        private static Matrix4x4 CreateVsElementLocalMatrix(ShapeElement elem)
        {
            // This mirrors the VS shape transform order for the default/non-anim path:
            // Translate(rotationOrigin) -> Rotate -> Scale -> Translate(from - rotationOrigin)
            // VS source applies From and RotationOrigin in 1/16 units. 
            float ox = elem.RotationOrigin != null ? (float)elem.RotationOrigin[0] / 16f : 0f;
            float oy = elem.RotationOrigin != null ? (float)elem.RotationOrigin[1] / 16f : 0f;
            float oz = elem.RotationOrigin != null ? (float)elem.RotationOrigin[2] / 16f : 0f;

            float fx = elem.From != null ? (float)elem.From[0] / 16f : 0f;
            float fy = elem.From != null ? (float)elem.From[1] / 16f : 0f;
            float fz = elem.From != null ? (float)elem.From[2] / 16f : 0f;

            var tOrigin = Matrix4x4.CreateTranslation(ox, oy, oz);
            var r = Matrix4x4.CreateFromYawPitchRoll(
                DegreesToRadians((float)elem.RotationY),
                DegreesToRadians((float)elem.RotationX),
                DegreesToRadians((float)elem.RotationZ)
            );
            var s = Matrix4x4.CreateScale((float)elem.ScaleX, (float)elem.ScaleY, (float)elem.ScaleZ);
            var tFromMinusOrigin = Matrix4x4.CreateTranslation(fx - ox, fy - oy, fz - oz);

            return tOrigin * r * s * tFromMinusOrigin;
        }

        private static float DegreesToRadians(float degrees) => degrees * (MathF.PI / 180f);

        private static BuiltCompound BuildCompoundFromShape(Shape shape, Shapes shapes, BufferPool bufferPool)
        {
            var children = new List<CompoundChild>();
            var childMasses = new List<float>();
            var childLocalInertias = new List<Symmetric3x3>();
            var manualBoxes = new List<ManualChildBox>();

            foreach (var root in shape.Elements)
            {
                AppendElementRecursive(root, Matrix4x4.Identity, shapes, children, childMasses, childLocalInertias, manualBoxes);
            }

            if (children.Count == 0)
            {
                throw new InvalidOperationException("Shape produced no collider children.");
            }

            Vector3 centerOfMass = ComputeCenterOfMass(children, childMasses);
            BodyInertia inertia = ComputeCompoundInertia(children, childMasses, childLocalInertias, centerOfMass);

            // Recenter children around the COM so the compound's local origin matches the body origin.
            for (int i = 0; i < children.Count; i++)
            {
                var child = children[i];
                child.LocalPose.Position -= centerOfMass;
                children[i] = child;
            }

            float broadphaseRadius = 0f;
            for (int i = 0; i < manualBoxes.Count; i++)
            {
                var box = manualBoxes[i];
                box.LocalPosition -= centerOfMass;
                manualBoxes[i] = box;

                float extent = box.LocalPosition.Length() + box.HalfExtents.Length();
                if (extent > broadphaseRadius) broadphaseRadius = extent;
            }

            bufferPool.Take<CompoundChild>(children.Count, out var childBuffer);
            for (int i = 0; i < children.Count; i++)
            {
                childBuffer[i] = children[i];
            }

            var compound = new BigCompound(childBuffer, shapes, bufferPool);

            return new BuiltCompound
            {
                Compound = compound,
                Inertia = inertia,
                LocalCenterOfMassOffset = centerOfMass,
                ManualChildBoxes = manualBoxes,
                BroadphaseRadius = broadphaseRadius
            };
        }

        private static Vector3 ComputeCenterOfMass(List<CompoundChild> children, List<float> childMasses)
        {
            float totalMass = 0f;
            Vector3 weightedSum = Vector3.Zero;

            for (int i = 0; i < children.Count; i++)
            {
                float mass = childMasses[i];
                totalMass += mass;
                weightedSum += children[i].LocalPose.Position * mass;
            }

            if (totalMass <= 0f)
            {
                throw new InvalidOperationException("Compound total mass must be > 0.");
            }

            return weightedSum / totalMass;
        }

        private static BodyInertia ComputeCompoundInertia(
            List<CompoundChild> children,
            List<float> childMasses,
            List<Symmetric3x3> childLocalInertias,
            Vector3 centerOfMass)
        {
            float totalMass = 0f;
            Symmetric3x3 summedInertia = default;

            for (int i = 0; i < children.Count; i++)
            {
                var child = children[i];
                float mass = childMasses[i];
                totalMass += mass;

                // Rotate the child's inertia tensor into compound space.
                Symmetric3x3 rotatedChildInertia = RotateSymmetric3x3(childLocalInertias[i], child.LocalPose.Orientation);

                // In 2.4, this is the public helper BEPU exposed for the offset term.
                // It gives you the added inertia from moving a point mass / child mass away from the COM.
                Symmetric3x3 offsetContribution;
                CompoundBuilder.GetOffsetInertiaContribution(
                    child.LocalPose.Position - centerOfMass,
                    mass,
                    out offsetContribution
                );

                Symmetric3x3 childContribution;
                Symmetric3x3.Add(rotatedChildInertia, offsetContribution, out childContribution);

                Symmetric3x3 newSum;
                Symmetric3x3.Add(summedInertia, childContribution, out newSum);
                summedInertia = newSum;
            }

            Symmetric3x3 inverseSummedInertia;
            Symmetric3x3.Invert(summedInertia, out inverseSummedInertia);

            return new BodyInertia
            {
                InverseMass = 1f / totalMass,
                InverseInertiaTensor = inverseSummedInertia
            };
        }

        private static Symmetric3x3 RotateSymmetric3x3(Symmetric3x3 tensor, Quaternion rotation)
        {
            Matrix4x4 r = Matrix4x4.CreateFromQuaternion(rotation);

            float r00 = r.M11; float r01 = r.M12; float r02 = r.M13;
            float r10 = r.M21; float r11 = r.M22; float r12 = r.M23;
            float r20 = r.M31; float r21 = r.M32; float r22 = r.M33;

            float i00 = tensor.XX;
            float i01 = tensor.YX;
            float i02 = tensor.ZX;
            float i11 = tensor.YY;
            float i12 = tensor.ZY;
            float i22 = tensor.ZZ;

            // temp = R * I
            float t00 = r00 * i00 + r01 * i01 + r02 * i02;
            float t01 = r00 * i01 + r01 * i11 + r02 * i12;
            float t02 = r00 * i02 + r01 * i12 + r02 * i22;

            float t10 = r10 * i00 + r11 * i01 + r12 * i02;
            float t11 = r10 * i01 + r11 * i11 + r12 * i12;
            float t12 = r10 * i02 + r11 * i12 + r12 * i22;

            float t20 = r20 * i00 + r21 * i01 + r22 * i02;
            float t21 = r20 * i01 + r21 * i11 + r22 * i12;
            float t22 = r20 * i02 + r21 * i12 + r22 * i22;

            // result = temp * R^T
            Symmetric3x3 result;
            result.XX = t00 * r00 + t01 * r01 + t02 * r02;
            result.YX = t10 * r00 + t11 * r01 + t12 * r02;
            result.YY = t10 * r10 + t11 * r11 + t12 * r12;
            result.ZX = t20 * r00 + t21 * r01 + t22 * r02;
            result.ZY = t20 * r10 + t21 * r11 + t22 * r12;
            result.ZZ = t20 * r20 + t21 * r21 + t22 * r22;
            return result;
        }

        private static void AppendElementRecursive(
            ShapeElement elem,
            Matrix4x4 parentWorld,
            Shapes shapes,
            List<CompoundChild> children,
            List<float> childMasses,
            List<Symmetric3x3> childLocalInertias,
            List<ManualChildBox> manualBoxes)
        {
            var local = CreateVsElementLocalMatrix(elem);
            var world = local * parentWorld;

            // A ShapeElement is fundamentally a cuboid from From -> To.
            // Size is also in 1/16 block units.
            if (elem.From != null && elem.To != null)
            {
                float sx = ((float)elem.To[0] - (float)elem.From[0]) / 16f;
                float sy = ((float)elem.To[1] - (float)elem.From[1]) / 16f;
                float sz = ((float)elem.To[2] - (float)elem.From[2]) / 16f;

                if (sx > 0f && sy > 0f && sz > 0f)
                {
                    // Local cuboid center before the element transform.
                    var localCenter = new Vector3(sx * 0.5f, sy * 0.5f, sz * 0.5f);

                    // World-space child pose inside the compound local frame.
                    var childPosition = TransformPoint(world, localCenter);
                    var childOrientation = ExtractRotation(world);

                    // Apply baked scale to box dimensions.
                    // Rotation and translation come from the decomposed world transform.
                    var worldScale = ExtractScale(world);

                    float width = MathF.Abs(sx * worldScale.X);
                    float height = MathF.Abs(sy * worldScale.Y);
                    float length = MathF.Abs(sz * worldScale.Z);

                    var box = new Box(width, height, length);

                    var shapeIndex = shapes.Add(box);

                    children.Add(new CompoundChild() { 
                        LocalPose = new RigidPose(childPosition, childOrientation),
                        ShapeIndex = shapeIndex
                    });

                    manualBoxes.Add(new ManualChildBox
                    {
                        LocalPosition = childPosition,
                        LocalOrientation = childOrientation,
                        HalfExtents = new Vector3(width * 0.5f, height * 0.5f, length * 0.5f)
                    });

                    float mass = width * height * length;
                    childMasses.Add(mass);

                    var childBodyInertia = box.ComputeInertia(mass);

                    Symmetric3x3 childLocalInertia;
                    Symmetric3x3.Invert(childBodyInertia.InverseInertiaTensor, out childLocalInertia);
                    childLocalInertias.Add(childLocalInertia);
                }
            }

            if (elem.Children == null) return;

            for (int i = 0; i < elem.Children.Length; i++)
            {
                AppendElementRecursive(elem.Children[i], world, shapes, children, childMasses, childLocalInertias, manualBoxes);
            }
        }

        public struct ManualChildBox
        {
            public Vector3 LocalPosition;
            public Quaternion LocalOrientation;
            public Vector3 HalfExtents;
        }

        public struct BuiltCompound
        {
            public BigCompound Compound;
            public BodyInertia Inertia;
            public Vector3 LocalCenterOfMassOffset;
            public List<ManualChildBox> ManualChildBoxes;
            public float BroadphaseRadius;
        }

        public struct CachedCompound
        {
            public TypedIndex CompoundIndex;
            public BodyInertia Inertia;
            public Vector3 LocalCenterOfMassOffset;
            public List<ManualChildBox> ManualChildBoxes;
            public float BroadphaseRadius;
        }
    }
}
