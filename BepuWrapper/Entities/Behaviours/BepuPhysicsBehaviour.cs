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

namespace BepuWrapper.Entities.Behaviours
{
    public class BepuPhysicsBehaviour : EntityBehavior
    {
        private ICoreClientAPI capi;
        private ICoreServerAPI sapi;

        private TypedIndex compoundShapeIndex;
        private Vector3 localCenterOfMassOffset;

        private readonly List<ManualChildBox> manualChildBoxes = new();
        private float compoundBroadphaseRadius;

        private float pushAccum;
        private Vector3 lastBodyOrigin;
        private bool hadLastBodyOrigin;

        public BepuPhysicsBehaviour(Entity entity) : base(entity)
        {
        }

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (entity.World.Side == EnumAppSide.Server)
            {
                sapi = (ICoreServerAPI)entity.Api;
                var physics = sapi.ModLoader.GetModSystem<BepuWrapperModSystem>();

                var shape = entity.Properties.Client.Shape;
                var shapeLoc = shape.Base.Clone();
                shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

                CachedCompound? cachedShape = physics.bepu.TryGetCompoundShape(shapeLoc.Path);

                if (cachedShape == null)
                {
                    var asset = sapi.Assets.TryGet(shapeLoc);
                    if (asset == null)
                    {
                        sapi.Logger.Warning("[bepuwrapper] Missing shape asset {0} for entity {1}", shapeLoc, entity.Code);
                        return;
                    }

                    var compoundShape = asset.ToObject<Shape>();
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

                var bodyPose = new RigidPose(
                    ToBepu(entity.Pos.X, entity.Pos.Y, entity.Pos.Z) + localCenterOfMassOffset,
                    Quaternion.Identity
                );

                var bodyDescription = BodyDescription.CreateDynamic(
                    bodyPose.Position,
                    cachedShape.Value.Inertia,
                    compoundShapeIndex,
                    0.05f
                );

                physics.bepu.RegisterEntityBody(entity, bodyDescription, localCenterOfMassOffset);

                lastBodyOrigin = bodyPose.Position;
                hadLastBodyOrigin = true;
            }
            else
            {
                capi = (ICoreClientAPI)entity.Api;
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            if (sapi == null || !entity.Alive || manualChildBoxes.Count == 0)
                return;

            pushAccum += deltaTime;
            if (pushAccum < 0.05f)
                return;

            float dt = pushAccum;
            pushAccum = 0f;

            PushNearbyEntitiesOutOfCompound(dt);
        }

        public override string PropertyName() => "bepu-physics";

        private void PushNearbyEntitiesOutOfCompound(float deltaTime)
        {
            var physics = sapi.ModLoader.GetModSystem<BepuWrapperModSystem>();

            Vector3 boatWorldOrigin = ToBepu(entity.Pos.X, entity.Pos.Y, entity.Pos.Z) + localCenterOfMassOffset;
            Quaternion boatWorldOrientation = Quaternion.Identity;

            // If you have a registered BEPU body for this entity, prefer its actual pose.
            // Adapt these calls to however your wrapper exposes the body handle.
            //if (physics.bepu.TryGetBodyPose(entity, out RigidPose pose))
            //{
            //    boatWorldOrigin = pose.Position;
            //    boatWorldOrientation = pose.Orientation;
            //}

            Vector3 bodyDelta = Vector3.Zero;
            if (hadLastBodyOrigin)
            {
                bodyDelta = boatWorldOrigin - lastBodyOrigin;
            }
            lastBodyOrigin = boatWorldOrigin;
            hadLastBodyOrigin = true;

            float radius = compoundBroadphaseRadius + 1.5f;

            var nearby = entity.World.GetEntitiesAround(
                new Vec3d(entity.Pos.X, entity.Pos.Y, entity.Pos.Z),
                radius,
                radius,
                e => e != null && e.EntityId != entity.EntityId
            );

            if (nearby == null)
                return;

            foreach (var other in nearby)
            {
                ResolveEntityAgainstCompound(other, boatWorldOrigin, boatWorldOrientation, bodyDelta, deltaTime);
            }
        }

        private void ResolveEntityAgainstCompound(
            Entity other,
            Vector3 boatWorldOrigin,
            Quaternion boatWorldOrientation,
            Vector3 bodyDelta,
            float deltaTime)
        {
            Cuboidf box = other.CollisionBox;
            if (box == null)
                return;

            float halfWidth = MathF.Max(
                (box.X2 - box.X1) * 0.5f,
                (box.Z2 - box.Z1) * 0.5f
            );
            float height = box.Y2 - box.Y1;

            if (halfWidth <= 0.001f || height <= 0.001f)
                return;

            Vector3 basePos = ToBepu(other.Pos.X, other.Pos.Y, other.Pos.Z);

            float sphereRadius = halfWidth;
            Vector3 p0 = basePos + new Vector3(0f, box.Y1 + sphereRadius, 0f);
            Vector3 p1 = basePos + new Vector3(0f, MathF.Max(box.Y1 + sphereRadius, box.Y2 - sphereRadius), 0f);

            Vector3 totalCorrection = Vector3.Zero;
            Vector3 bestNormal = Vector3.Zero;
            bool hadHit = false;

            for (int iteration = 0; iteration < 2; iteration++)
            {
                Vector3 correction0 = ComputeCompoundPushoutForSphere(p0 + totalCorrection, sphereRadius, boatWorldOrigin, boatWorldOrientation);
                Vector3 correction1 = ComputeCompoundPushoutForSphere(p1 + totalCorrection, sphereRadius, boatWorldOrigin, boatWorldOrientation);

                Vector3 correction = correction0.LengthSquared() > correction1.LengthSquared() ? correction0 : correction1;
                if (correction.LengthSquared() < 1e-8f)
                    break;

                totalCorrection += correction;
                bestNormal = SafeNormalize(correction);
                hadHit = true;
            }

            if (!hadHit)
                return;

            float correctionLen = totalCorrection.Length();
            if (correctionLen <= 1e-8f)
                return;

            // Small slop to reduce jitter at contact.
            const float slop = 0.001f;
            if (correctionLen > slop)
            {
                totalCorrection = bestNormal * (correctionLen - slop);
            }
            else
            {
                totalCorrection = Vector3.Zero;
            }

            if (totalCorrection.LengthSquared() <= 1e-8f)
                return;

            // Equivalent to entity.setPos(entity.position() + mtv)
            other.SidedPos.X += totalCorrection.X;
            other.SidedPos.Y += totalCorrection.Y;
            other.SidedPos.Z += totalCorrection.Z;

            // Equivalent to zeroing velocity along MTV axes.
            Vector3 motion = new Vector3(
                (float)other.SidedPos.Motion.X,
                (float)other.SidedPos.Motion.Y,
                (float)other.SidedPos.Motion.Z
            );

            const float axisEpsilon = 0.0001f;

            if (MathF.Abs(totalCorrection.X) > axisEpsilon)
                motion.X = 0f;

            bool pushedUp = false;
            if (MathF.Abs(totalCorrection.Y) > axisEpsilon)
            {
                motion.Y = 0f;
                if (totalCorrection.Y > 0f)
                {
                    other.OnGround = true;
                    pushedUp = true;
                }
            }

            if (MathF.Abs(totalCorrection.Z) > axisEpsilon)
                motion.Z = 0f;

            if (other is EntityPlayer)
            {
                other.WatchedAttributes.SetDouble("rbodirX", motion.X);
                other.WatchedAttributes.SetDouble("rbodirY", motion.Y);
                other.WatchedAttributes.SetDouble("rbodirZ", motion.Z);
                other.WatchedAttributes.SetFloat("bodyDeltaX", bodyDelta.X);
                other.WatchedAttributes.SetFloat("bodyDeltaY", bodyDelta.Y);
                other.WatchedAttributes.SetFloat("bodyDeltaZ", bodyDelta.Z);
                other.WatchedAttributes.SetBool("pushedUp", pushedUp);
                other.WatchedAttributes.SetInt("physcoll", 1);
                return;
            }

            other.SidedPos.Motion.Set(motion.X, motion.Y, motion.Z);


            // Equivalent to carrying entity along with moving rigid body if standing on it.
            if (pushedUp && bodyDelta.LengthSquared() > 1e-9f)
            {
                Vector3 horizontalDelta = new Vector3(bodyDelta.X, 0f, bodyDelta.Z) * 0.5f;
                Vector3 verticalDelta = new Vector3(0f, bodyDelta.Y, 0f);

                other.SidedPos.Add(horizontalDelta.X + verticalDelta.X, horizontalDelta.Y + verticalDelta.Y, horizontalDelta.Z + verticalDelta.Z);
                other.SidedPos.Motion.Add(horizontalDelta.X + verticalDelta.X, horizontalDelta.Y + verticalDelta.Y, horizontalDelta.Z + verticalDelta.Z);

                other.OnGround = true;
            }
        }

        private Vector3 ComputeCompoundPushoutForSphere(
            Vector3 sphereCenterWorld,
            float sphereRadius,
            Vector3 boatWorldOrigin,
            Quaternion boatWorldOrientation)
        {
            Vector3 bestPush = Vector3.Zero;
            float bestDepthSq = 0f;

            Quaternion inverseBoatRotation = Quaternion.Conjugate(boatWorldOrientation);
            Vector3 sphereCenterInBoat = Vector3.Transform(sphereCenterWorld - boatWorldOrigin, inverseBoatRotation);

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                ManualChildBox child = manualChildBoxes[i];

                float maxReach = child.HalfExtents.Length() + sphereRadius;
                if (Vector3.DistanceSquared(sphereCenterInBoat, child.LocalPosition) > maxReach * maxReach)
                    continue;

                Vector3 push = ComputeSphereVsChildBoxPushout(
                    sphereCenterInBoat,
                    sphereRadius,
                    child.LocalPosition,
                    child.LocalOrientation,
                    child.HalfExtents
                );

                float depthSq = push.LengthSquared();
                if (depthSq > bestDepthSq)
                {
                    bestDepthSq = depthSq;
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

            if (distSq > 1e-10f)
            {
                float dist = MathF.Sqrt(distSq);
                float penetration = sphereRadius - dist;
                if (penetration <= 0f)
                    return Vector3.Zero;

                Vector3 normalLocal = delta / dist;
                return Vector3.Transform(normalLocal * penetration, boxLocalOrientation);
            }

            float dx = boxHalfExtents.X - MathF.Abs(local.X);
            float dy = boxHalfExtents.Y - MathF.Abs(local.Y);
            float dz = boxHalfExtents.Z - MathF.Abs(local.Z);

            Vector3 axisNormal;
            float faceDistance;

            if (dx <= dy && dx <= dz)
            {
                axisNormal = new Vector3(local.X >= 0f ? 1f : -1f, 0f, 0f);
                faceDistance = dx;
            }
            else if (dy <= dz)
            {
                axisNormal = new Vector3(0f, local.Y >= 0f ? 1f : -1f, 0f);
                faceDistance = dy;
            }
            else
            {
                axisNormal = new Vector3(0f, 0f, local.Z >= 0f ? 1f : -1f);
                faceDistance = dz;
            }

            return Vector3.Transform(axisNormal * (sphereRadius + faceDistance), boxLocalOrientation);
        }

        private static Vector3 SafeNormalize(Vector3 v)
        {
            float lenSq = v.LengthSquared();
            if (lenSq < 1e-10f)
                return Vector3.UnitY;
            return v / MathF.Sqrt(lenSq);
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
                throw new InvalidOperationException("Shape produced no collider children.");

            Vector3 centerOfMass = ComputeCenterOfMass(children, childMasses);
            BodyInertia inertia = ComputeCompoundInertia(children, childMasses, childLocalInertias, centerOfMass);

            for (int i = 0; i < children.Count; i++)
            {
                var child = children[i];
                child.LocalPose.Position -= centerOfMass;
                children[i] = child;
            }

            const int maxManualBoxes = 64;
            if (manualBoxes.Count > maxManualBoxes)
            {
                manualBoxes.Sort((a, b) =>
                    (b.HalfExtents.X * b.HalfExtents.Y * b.HalfExtents.Z)
                    .CompareTo(a.HalfExtents.X * a.HalfExtents.Y * a.HalfExtents.Z));

                manualBoxes.RemoveRange(maxManualBoxes, manualBoxes.Count - maxManualBoxes);
            }

            float broadphaseRadius = 0f;
            for (int i = 0; i < manualBoxes.Count; i++)
            {
                var box = manualBoxes[i];
                box.LocalPosition -= centerOfMass;
                manualBoxes[i] = box;

                float extent = box.LocalPosition.Length() + box.HalfExtents.Length();
                if (extent > broadphaseRadius)
                    broadphaseRadius = extent;
            }

            bufferPool.Take<CompoundChild>(children.Count, out var childBuffer);
            for (int i = 0; i < children.Count; i++)
            {
                childBuffer[i] = children[i];
            }

            var compound = new Compound(childBuffer);

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
                throw new InvalidOperationException("Compound total mass must be > 0.");

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

                Symmetric3x3 rotatedChildInertia = RotateSymmetric3x3(childLocalInertias[i], child.LocalPose.Orientation);

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

            float t00 = r00 * i00 + r01 * i01 + r02 * i02;
            float t01 = r00 * i01 + r01 * i11 + r02 * i12;
            float t02 = r00 * i02 + r01 * i12 + r02 * i22;

            float t10 = r10 * i00 + r11 * i01 + r12 * i02;
            float t11 = r10 * i01 + r11 * i11 + r12 * i12;
            float t12 = r10 * i02 + r11 * i12 + r12 * i22;

            float t20 = r20 * i00 + r21 * i01 + r22 * i02;
            float t21 = r20 * i01 + r21 * i11 + r22 * i12;
            float t22 = r20 * i02 + r21 * i12 + r22 * i22;

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

            if (elem.From != null && elem.To != null)
            {
                float sx = ((float)elem.To[0] - (float)elem.From[0]) / 16f;
                float sy = ((float)elem.To[1] - (float)elem.From[1]) / 16f;
                float sz = ((float)elem.To[2] - (float)elem.From[2]) / 16f;

                if (sx > 0f && sy > 0f && sz > 0f)
                {
                    var localCenter = new Vector3(sx * 0.5f, sy * 0.5f, sz * 0.5f);
                    var childPosition = TransformPoint(world, localCenter);
                    var childOrientation = ExtractRotation(world);
                    var worldScale = ExtractScale(world);

                    float width = MathF.Abs(sx * worldScale.X);
                    float height = MathF.Abs(sy * worldScale.Y);
                    float length = MathF.Abs(sz * worldScale.Z);

                    var box = new Box(width, height, length);
                    var shapeIndex = shapes.Add(box);

                    children.Add(new CompoundChild()
                    {
                        LocalPose = new RigidPose(childPosition, childOrientation),
                        ShapeIndex = shapeIndex
                    });

                    if (width >= 0.05f && height >= 0.05f && length >= 0.05f)
                    {
                        manualBoxes.Add(new ManualChildBox
                        {
                            LocalPosition = childPosition,
                            LocalOrientation = childOrientation,
                            HalfExtents = new Vector3(width * 0.5f, height * 0.5f, length * 0.5f)
                        });
                    }

                    float mass = width * height * length;
                    childMasses.Add(mass);

                    var childBodyInertia = box.ComputeInertia(mass);

                    Symmetric3x3 childLocalInertia;
                    Symmetric3x3.Invert(childBodyInertia.InverseInertiaTensor, out childLocalInertia);
                    childLocalInertias.Add(childLocalInertia);
                }
            }

            if (elem.Children == null)
                return;

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
            public Compound Compound;
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