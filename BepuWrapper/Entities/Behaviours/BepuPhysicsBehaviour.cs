using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using BepuWrapper.Api;
using BepuWrapper.Client;
using BepuWrapper.patches;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace BepuWrapper.Entities.Behaviours
{
    public class BepuPhysicsBehaviour : PhysicsBehaviorBase
    {
        private ICoreAPI api;
        private BepuWrapperModSystem physics;
        private string[] config;

        private BodyHandle body;
        private TypedIndex compoundShapeIndex;
        private Vector3 localCenterOfMassOffset;

        private readonly List<ManualChildBox> manualChildBoxes = new();
        private float compoundBroadphaseRadius;

        private Vector3 lastBodyOrigin;
        private bool hadLastBodyOrigin;

        public BepuPhysicsBehaviour(Entity entity) : base(entity)
        {
        }

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (entity.Api.Side == EnumAppSide.Client)
            {
                capi = entity.Api as ICoreClientAPI;
                capi.Event.RegisterRenderer(new DebugRenderer(this), EnumRenderStage.AfterFinalComposition);
            }

            config = attributes["selectors"].AsArray<string>();

            api = entity.Api;
            physics = api.ModLoader.GetModSystem<BepuWrapperModSystem>();

            var shape = entity.Properties.Client.Shape;
            var shapeLoc = shape.Base.Clone();
            shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

            CachedCompound? cachedShape = physics.bepu.TryGetCompoundShape(shapeLoc.Path);

            if (cachedShape == null)
            {
                var asset = api.Assets.TryGet(shapeLoc);
                if (asset == null)
                {
                    api.Logger.Warning("[bepuwrapper] Missing shape asset {0} for entity {1}", shapeLoc, entity.Code);
                    return;
                }

                var compoundShape = asset.ToObject<Shape>();
                if (compoundShape == null || compoundShape.Elements == null || compoundShape.Elements.Length == 0)
                {
                    api.Logger.Warning("[bepuwrapper] Entity {0} has no loaded shape elements.", entity.Code);
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

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);

            if (api == null || !entity.Alive || manualChildBoxes.Count == 0)
                return;

            //PushNearbyEntitiesOutOfCompound();
        }

        public void DebugRender(ICoreClientAPI capi)
        {
            if (manualChildBoxes == null || manualChildBoxes.Count == 0) return;

            if (!TryGetCollisionPose(out Vector3 bodyPos, out Quaternion bodyRot)) return;

            var camPos = capi.World.Player.Entity.CameraPos;
            var color = ColorUtil.ToRgba(255, 255, 0, 0); // red

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                var child = manualChildBoxes[i];

                Quaternion worldRot = Quaternion.Normalize(bodyRot * child.LocalOrientation);
                Vector3 worldCenter = bodyPos + Vector3.Transform(child.LocalPosition, bodyRot);

                DrawOrientedBox(capi, worldCenter, worldRot, child.HalfExtents, color);
            }
        }

        private void DrawOrientedBox(
            ICoreClientAPI capi,
            Vector3 center,
            Quaternion rot,
            Vector3 halfExtents,
            int color)
        {
            Vector3 right = Vector3.Transform(Vector3.UnitX, rot);
            Vector3 up = Vector3.Transform(Vector3.UnitY, rot);
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, rot);

            Vector3 hx = right * halfExtents.X;
            Vector3 hy = up * halfExtents.Y;
            Vector3 hz = forward * halfExtents.Z;

            Vector3[] corners = new Vector3[8];

            corners[0] = center - hx - hy - hz;
            corners[1] = center + hx - hy - hz;
            corners[2] = center + hx + hy - hz;
            corners[3] = center - hx + hy - hz;

            corners[4] = center - hx - hy + hz;
            corners[5] = center + hx - hy + hz;
            corners[6] = center + hx + hy + hz;
            corners[7] = center - hx + hy + hz;

            BlockPos origin = entity.Pos.AsBlockPos;

            for (int i = 0; i < 8; i++)
            {
                corners[i] -= new Vector3(origin.X, origin.Y, origin.Z);
            }

            Line(capi, origin, corners[0], corners[1], color);
            Line(capi, origin, corners[1], corners[2], color);
            Line(capi, origin, corners[2], corners[3], color);
            Line(capi, origin, corners[3], corners[0], color);

            Line(capi, origin, corners[4], corners[5], color);
            Line(capi, origin, corners[5], corners[6], color);
            Line(capi, origin, corners[6], corners[7], color);
            Line(capi, origin, corners[7], corners[4], color);

            Line(capi, origin, corners[0], corners[4], color);
            Line(capi, origin, corners[1], corners[5], color);
            Line(capi, origin, corners[2], corners[6], color);
            Line(capi, origin, corners[3], corners[7], color);
        }

        private void Line(ICoreClientAPI capi, BlockPos origin, Vector3 a, Vector3 b, int color)
        {
            capi.Render.RenderLine(
                origin,
                a.X, a.Y, a.Z,
                b.X, b.Y, b.Z,
                color
            );
        }

        public override string PropertyName() => "bepu-physics";

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

        private static float DegreesToRadians(float degrees) => degrees * (MathF.PI / 180f);

        private static Matrix4x4 CreateVsElementLocalMatrix(ShapeElement elem)
        {
            float ox = elem.RotationOrigin != null ? (float)elem.RotationOrigin[0] / 16f : 0f;
            float oy = elem.RotationOrigin != null ? (float)elem.RotationOrigin[1] / 16f : 0f;
            float oz = elem.RotationOrigin != null ? (float)elem.RotationOrigin[2] / 16f : 0f;

            float fx = elem.From != null ? (float)elem.From[0] / 16f : 0f;
            float fy = elem.From != null ? (float)elem.From[1] / 16f : 0f;
            float fz = elem.From != null ? (float)elem.From[2] / 16f : 0f;

            var translation = Matrix4x4.CreateTranslation(fx, fy, fz);
            var toOrigin = Matrix4x4.CreateTranslation(-ox, -oy, -oz);
            var fromOrigin = Matrix4x4.CreateTranslation(ox, oy, oz);

            var rotation = Matrix4x4.CreateFromYawPitchRoll(
                DegreesToRadians((float)elem.RotationY),
                DegreesToRadians((float)elem.RotationX),
                DegreesToRadians((float)elem.RotationZ)
            );

            var scale = Matrix4x4.CreateScale(
                (float)elem.ScaleX,
                (float)elem.ScaleY,
                (float)elem.ScaleZ
            );

            return translation * fromOrigin * rotation * scale * toOrigin;
        }

        private BuiltCompound BuildCompoundFromShape(Shape shape, Shapes shapes, BufferPool bufferPool)
        {
            var children = new List<CompoundChild>();
            var childMasses = new List<float>();
            var childLocalInertias = new List<Symmetric3x3>();
            var manualBoxes = new List<ManualChildBox>();

            foreach (var elementSelector in config)
            {
                shape.WalkElements(elementSelector, element =>
                {
                    AppendElement(
                        element,
                        Matrix4x4.Identity,
                        shapes,
                        children,
                        childMasses,
                        childLocalInertias,
                        manualBoxes
                    );
                });
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

        private void AppendElement(
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

                    children.Add(new CompoundChild
                    {
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
        }
        private void AppendElementRecursive(
            ShapeElement elem,
            Matrix4x4 parentWorld,
            Shapes shapes,
            List<CompoundChild> children,
            List<float> childMasses,
            List<Symmetric3x3> childLocalInertias,
            List<ManualChildBox> manualBoxes)
        {
            var local = CreateVsElementLocalMatrix(elem);
            var world = parentWorld * local;

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

                    children.Add(new CompoundChild
                    {
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

            if (elem.Children == null)
                return;

            for (int i = 0; i < elem.Children.Length; i++)
            {
                AppendElementRecursive(
                    elem.Children[i],
                    world,
                    shapes,
                    children,
                    childMasses,
                    childLocalInertias,
                    manualBoxes
                );
            }
        }

        private Vector3 ComputeCenterOfMass(List<CompoundChild> children, List<float> childMasses)
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

        private BodyInertia ComputeCompoundInertia(
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

        private Symmetric3x3 RotateSymmetric3x3(Symmetric3x3 tensor, Quaternion rotation)
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
        public void AppendDynamicCollisionBoxes(
            Cuboidd queryBox,
            List<DynamicCollisionBox> results)
        {
            if (manualChildBoxes == null || manualChildBoxes.Count == 0)
                return;

            if (!TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation))
                return;

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                ManualChildBox child = manualChildBoxes[i];

                Quaternion childWorldOrientation = Quaternion.Normalize(bodyOrientation * child.LocalOrientation);
                Vector3 childWorldCenter = bodyPosition + Vector3.Transform(child.LocalPosition, bodyOrientation);

                Cuboidd worldAabb = CreateAabbFromOrientedBox(
                    childWorldCenter,
                    childWorldOrientation,
                    child.HalfExtents
                );

                if (!worldAabb.IntersectsOrTouches(queryBox))
                    continue;

                results.Add(new DynamicCollisionBox
                {
                    Box = worldAabb,
                    SourceEntity = entity,
                    CanSupport = true
                });
            }
        }

        private bool TryGetCollisionPose(out Vector3 bodyPosition, out Quaternion bodyOrientation)
        {
            bodyPosition = Vector3.Zero;
            bodyOrientation = Quaternion.Identity;

            if (physics?.bepu == null)
                return false;

            //if (physics.bepu.TryGetEntityBodyPose(entity, out bodyPosition, out bodyOrientation))
            //    return true; TODO: [AD] Look at this

            bodyPosition = ToBepu(entity.Pos.X, entity.Pos.Y, entity.Pos.Z) + localCenterOfMassOffset;
            bodyOrientation = Quaternion.Identity;
            return true;
        }

        private static Cuboidd CreateAabbFromOrientedBox(
            Vector3 center,
            Quaternion orientation,
            Vector3 halfExtents)
        {
            Vector3 right = Vector3.Transform(Vector3.UnitX, orientation);
            Vector3 up = Vector3.Transform(Vector3.UnitY, orientation);
            Vector3 forward = Vector3.Transform(Vector3.UnitZ, orientation);

            Vector3 extents = new Vector3(
                MathF.Abs(right.X) * halfExtents.X + MathF.Abs(up.X) * halfExtents.Y + MathF.Abs(forward.X) * halfExtents.Z,
                MathF.Abs(right.Y) * halfExtents.X + MathF.Abs(up.Y) * halfExtents.Y + MathF.Abs(forward.Y) * halfExtents.Z,
                MathF.Abs(right.Z) * halfExtents.X + MathF.Abs(up.Z) * halfExtents.Y + MathF.Abs(forward.Z) * halfExtents.Z
            );

            return new Cuboidd(
                center.X - extents.X,
                center.Y - extents.Y,
                center.Z - extents.Z,
                center.X + extents.X,
                center.Y + extents.Y,
                center.Z + extents.Z
            );
        }
    }
}