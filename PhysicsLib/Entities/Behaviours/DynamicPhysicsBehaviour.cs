using PhysicsLib.Api;
using PhysicsLib.Api.CollisionSource;
using PhysicsLib.Client;
using PhysicsLib.patches;
using System;
using System.Collections.Generic;
using System.Numerics;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.Datastructures;
using Vintagestory.API.MathTools;

namespace PhysicsLib.Entities.Behaviours
{
    public class DynamicPhysicsBehaviour : PhysicsBehaviorBase
    {
        private ICoreAPI? api;
        private PhysicsLibModSystem? physics;
        private string[]? selectors;
        private DebugRenderer? debugRenderer;
        public Entity Entity { get; }
        private Vector3 localCenterOfMassOffset;

        // Shared across all instances of the same entity type — built once, never mutated.
        private List<LocalBox> manualChildBoxes = new List<LocalBox>();
        public List<LocalBox> VehicleChildBoxes = new List<LocalBox>();
        public bool isVehicle = false;
        // Per-instance pose tracking for carry delta.
        private Vec3d previousBodyPositionD = new Vec3d();
        private Vec3d currentBodyPositionD = new Vec3d();
        private Quaternion previousBodyOrientation = Quaternion.Identity;
        private Quaternion currentBodyOrientation = Quaternion.Identity;
        private bool hasPreviousPose;

        public Vec3f velocity = new Vec3f();

        public DynamicPhysicsBehaviour(Entity entity) : base(entity)
        {
        }

        public override void Initialize(EntityProperties properties, JsonObject attributes)
        {
            base.Initialize(properties, attributes);

            if (entity.Api.Side == EnumAppSide.Client)
            {
                capi = entity.Api as ICoreClientAPI;
                debugRenderer = new DebugRenderer(this);
                capi.Event.RegisterRenderer(debugRenderer, EnumRenderStage.AfterFinalComposition);
            }

            selectors = attributes["selectors"].AsArray<string>();

            api = entity.Api;
            physics = api.ModLoader.GetModSystem<PhysicsLibModSystem>();
            AssetLocation shapeLoc = Block.DefaultCubeShape.Base.Clone();//So the compiler stops yelling at me
            (CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource as DynamicCollisionSource)
                ?.Register(this);
            BuiltCompound? cachedShape;
            if (entity is EntityChunky)
            {
                isVehicle = true;
                HandleEntityChunky(entity, out cachedShape);//Do this so that it's easy to harmony patch
                if (cachedShape == null)
                {
                    api.Logger.Event("cachedShape of EntityChunky is null");
                    CompositeShape shape = Block.DefaultCubeShape;
                    shapeLoc = shape.Base.Clone();
                    shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

                    cachedShape = physics.TryGetCompoundShape(shapeLoc.Path);
                }
            }
            else
            {
                CompositeShape shape = entity.Properties.Client.Shape;
                shapeLoc = shape.Base.Clone();
                shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";

                cachedShape = physics.TryGetCompoundShape(shapeLoc.Path);
            }
            

            if (cachedShape == null && !isVehicle)
            {
                var asset = api.Assets.TryGet(shapeLoc);
                if (asset == null)
                {
                    api.Logger.Warning("[physicslib] Missing shape asset {0} for entity {1}", shapeLoc, entity.Code);
                    return;
                }

                var compoundShape = asset.ToObject<Shape>();
                if (compoundShape == null || compoundShape.Elements == null || compoundShape.Elements.Length == 0)
                {
                    api.Logger.Warning("[physicslib] Entity {0} has no loaded shape elements.", entity.Code);
                    return;
                }

                cachedShape = physics.AddCompoundShape(shapeLoc.Path, BuildCompoundFromShape(compoundShape));
            }

            if (isVehicle && cachedShape != null)
            {
                VehicleChildBoxes = cachedShape.Boxes;
            }
            else if (!isVehicle && cachedShape != null)
            {
                manualChildBoxes = cachedShape.Boxes;
            }
            if (cachedShape != null) localCenterOfMassOffset = cachedShape.LocalCenterOfMassOffset;
        }

        public override void OnEntityDespawn(EntityDespawnData despawn)
        {
            base.OnEntityDespawn(despawn);

            (CollisionTester_ApplyTerrainCollision_Patch.DynamicCollisionSource as DynamicCollisionSource)
                ?.Unregister(this);

            if (debugRenderer != null && capi != null)
            {
                capi.Event.UnregisterRenderer(debugRenderer, EnumRenderStage.AfterFinalComposition);
                debugRenderer = null;
            }
        }

        public override void OnGameTick(float deltaTime)
        {
            base.OnGameTick(deltaTime);
            
            if (api == null || !entity.Alive) return;

            if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return;

            if (!hasPreviousPose)
            {
                previousBodyPositionD.Set(bodyPosD);
                currentBodyPositionD.Set(bodyPosD);
                previousBodyOrientation = bodyOrientation;
                currentBodyOrientation = bodyOrientation;
                hasPreviousPose = true;
                velocity.Set(0, 0, 0);
            }
            else
            {
                previousBodyPositionD.Set(currentBodyPositionD);
                previousBodyOrientation = currentBodyOrientation;

                currentBodyPositionD.Set(bodyPosD);
                currentBodyOrientation = bodyOrientation;

                if (deltaTime > 1e-6f)
                {
                    velocity.Set(
                        (float)((currentBodyPositionD.X - previousBodyPositionD.X) / deltaTime),
                        (float)((currentBodyPositionD.Y - previousBodyPositionD.Y) / deltaTime),
                        (float)((currentBodyPositionD.Z - previousBodyPositionD.Z) / deltaTime));
                }
                else
                {
                    velocity.Set(0, 0, 0);
                }
            }
        }

        public override string PropertyName() => "dynamic-physics";

        // Called by DynamicCollisionSource for each registered behaviour.
        // Transforms the shared local boxes to world space on the fly — no cached world list.
        public void AppendDynamicCollisionBoxes(Cuboidd queryBox, ref List<DynamicCollisionBox> results)
        {
            if (!isVehicle)
            {
                if (manualChildBoxes.Count == 0) return;

                if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation))
                    return;

                Matrix4x4 bodyRotationMatrix = Matrix4x4.CreateFromQuaternion(bodyOrientation);

                for (int i = 0; i < manualChildBoxes.Count; i++)
                {
                    LocalBox child = manualChildBoxes[i];

                    Vector3 localOffset = Vector3.Transform(child.LocalPosition, bodyOrientation);
                    Vec3d worldCenterD = new Vec3d(
                        bodyPosD.X + localOffset.X,
                        bodyPosD.Y + localOffset.Y,
                        bodyPosD.Z + localOffset.Z);

                    Cuboidd broadphase = CreateAabbFromOrientedBox(
                        worldCenterD,
                        bodyOrientation, // orientation applied below per child
                        child.HalfExtents);

                    if (!broadphase.IntersectsOrTouches(queryBox)) continue;
                    

                    Matrix4x4 childLocalRot = Matrix4x4.CreateFromQuaternion(child.LocalOrientation);
                    Quaternion childWorldOri = ExtractPureRotation(childLocalRot * bodyRotationMatrix);

                    // Recompute tighter broadphase with correct child orientation.
                    Cuboidd tightBroadphase = CreateAabbFromOrientedBox(worldCenterD, childWorldOri, child.HalfExtents);
                    if (!tightBroadphase.IntersectsOrTouches(queryBox)) continue;
                    

                    Vector3 worldCenter = new Vector3(
                        (float)worldCenterD.X,
                        (float)worldCenterD.Y,
                        (float)worldCenterD.Z);

                    results.Add(new DynamicCollisionBox
                    {
                        Box = tightBroadphase,
                        Center = worldCenter,
                        CenterD = worldCenterD,
                        Orientation = childWorldOri,
                        HalfExtents = child.HalfExtents,
                        SourceEntity = entity,
                        CanSupport = true
                    });
                }
            }
            else
            {
                if (VehicleChildBoxes == null)
                {
                    //entity.Api.Logger.Event("[physicslib] VehicleChildBoxes is null");
                    //HandleEntityChunky(entity, out BuiltCompound? cachedShape);
                    //VehicleChildBoxes = cachedShape.Value.ManualChildBoxes;
                    
                    entity.Api.Logger.Event("[physicslib] VehicleChildBoxes is null. Aborting");
                    return;
                }

                if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation))
                {
                    entity.Api.Logger.Event("[physicslib] Failed to get Collision Pose");
                    return;
                }
                
                Matrix4x4 bodyRotationMatrix = Matrix4x4.CreateFromQuaternion(bodyOrientation);

                foreach (LocalBox child in VehicleChildBoxes)
                {
                    Vector3 localOffset = Vector3.Transform(child.LocalPosition, bodyOrientation);
                    Vec3d worldCenterD = new Vec3d(
                        bodyPosD.X + localOffset.X,
                        bodyPosD.Y + localOffset.Y,
                        bodyPosD.Z + localOffset.Z);

                    Cuboidd broadphase = CreateAabbFromOrientedBox(
                        worldCenterD,
                        bodyOrientation, // orientation applied below per child
                        child.HalfExtents);

                    if (!broadphase.IntersectsOrTouches(queryBox)) continue;
                    
                    Matrix4x4 childLocalRot = Matrix4x4.CreateFromQuaternion(child.LocalOrientation);
                    Quaternion childWorldOri = ExtractPureRotation(childLocalRot * bodyRotationMatrix);

                    // Recompute tighter broadphase with correct child orientation.
                    Cuboidd tightBroadphase = CreateAabbFromOrientedBox(worldCenterD, childWorldOri, child.HalfExtents);
                    if (!tightBroadphase.IntersectsOrTouches(queryBox)) continue;
                    
                    Vector3 worldCenter = new Vector3(
                        (float)worldCenterD.X,
                        (float)worldCenterD.Y,
                        (float)worldCenterD.Z);

                    results.Add(new DynamicCollisionBox
                    {
                        Box = tightBroadphase,
                        Center = worldCenter,
                        CenterD = worldCenterD,
                        Orientation = childWorldOri,
                        HalfExtents = child.HalfExtents,
                        SourceEntity = entity,
                        CanSupport = true
                    });
                }
            }
        }

        public bool TryTransformWorldPointToLocal(Vec3d worldPoint, out Vector3 localPoint)
        {
            localPoint = Vector3.Zero;

            if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return false;

            Vector3 p = new Vector3(
                (float)(worldPoint.X - bodyPosD.X),
                (float)(worldPoint.Y - bodyPosD.Y),
                (float)(worldPoint.Z - bodyPosD.Z));

            localPoint = Vector3.Transform(p, Quaternion.Inverse(bodyOrientation));
            return true;
        }

        public bool TryGetPointVelocityDelta(Vector3 localPoint, out Vec3d delta)
        {
            delta = new Vec3d();
            if (!hasPreviousPose) return false;

            Vector3 prevOffset = Vector3.Transform(localPoint, previousBodyOrientation);
            Vector3 currOffset = Vector3.Transform(localPoint, currentBodyOrientation);

            delta.Set(
                currentBodyPositionD.X + currOffset.X - (previousBodyPositionD.X + prevOffset.X),
                currentBodyPositionD.Y + currOffset.Y - (previousBodyPositionD.Y + prevOffset.Y),
                currentBodyPositionD.Z + currOffset.Z - (previousBodyPositionD.Z + prevOffset.Z));
            return true;
        }

        public bool TryGetSupportTopYUnderBox(Cuboidd entityBox, double horizontalPadding, out double supportTopY)
        {
            supportTopY = 0.0;
            if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation))
                return false;

            bool found = false;
            Matrix4x4 bodyRotationMatrix = Matrix4x4.CreateFromQuaternion(bodyOrientation);

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                LocalBox child = manualChildBoxes[i];
                Vector3 localOffset = Vector3.Transform(child.LocalPosition, bodyOrientation);
                Vec3d wc = new Vec3d(bodyPosD.X + localOffset.X, bodyPosD.Y + localOffset.Y, bodyPosD.Z + localOffset.Z);
                Cuboidd b = CreateAabbFromOrientedBox(wc, bodyOrientation, child.HalfExtents);

                bool overlapsH =
                    entityBox.X2 > b.X1 - horizontalPadding && entityBox.X1 < b.X2 + horizontalPadding &&
                    entityBox.Z2 > b.Z1 - horizontalPadding && entityBox.Z1 < b.Z2 + horizontalPadding;

                if (!overlapsH) continue;
                if (!found || b.Y2 > supportTopY) { supportTopY = b.Y2; found = true; }
            }

            return found;
        }

        public void HandleEntityChunky(Entity entity, out BuiltCompound? cachedShape)
        {
            CompositeShape shape;
            entity.Api.Logger.Event("Default EntityChunky executing");
            shape = Block.DefaultCubeShape;//A basic solution that can be overwritten with a harmony patch
            
            AssetLocation shapeLoc = shape.Base.Clone();
            shapeLoc.Path = "shapes/" + shapeLoc.Path + ".json";
            cachedShape = physics.TryGetCompoundShape(shapeLoc.Path);
        }

        // ── shape building (runs once per entity type, result is shared) ──────────

        private BuiltCompound BuildCompoundFromShape(Shape shape)
        {
            var childMasses = new List<float>();
            var manualBoxes = new List<LocalBox>();

            if (shape.Elements == null || shape.Elements.Length == 0)
                throw new InvalidOperationException("Shape has no root elements.");

            for (int i = 0; i < shape.Elements.Length; i++)
            {
                AppendSelectedElementsRecursive(
                    shape.Elements[i], Matrix4x4.Identity,
                    shape.Elements[i].Name ?? string.Empty,
                    false, childMasses, manualBoxes);
            }

            if (manualBoxes.Count == 0)
                throw new InvalidOperationException("Shape produced no collider children.");

            Vector3 centerOfMass = ComputeCenterOfMass(manualBoxes, childMasses);
            for (int i = 0; i < manualBoxes.Count; i++)
            {
                LocalBox b = manualBoxes[i];
                b.LocalPosition -= centerOfMass;
                manualBoxes[i] = b;
            }

            return new BuiltCompound { LocalCenterOfMassOffset = centerOfMass, Boxes = manualBoxes };
        }

        private void AppendSelectedElementsRecursive(
            ShapeElement elem, Matrix4x4 parentWorld, string path,
            bool parentSelected, List<float> childMasses, List<LocalBox> manualBoxes)
        {
            bool selected = parentSelected || MatchesAnySelector(path);
            Matrix4x4 local = CreateVsElementLocalMatrix(elem, parentWorld.M11);
            Matrix4x4 world = local * parentWorld;

            if (selected && ElementHasRealBox(elem))
            {
                float fx = (float)elem.From![0] / 16f, fy = (float)elem.From![1] / 16f, fz = (float)elem.From![2] / 16f;
                float tx = (float)elem.To![0] / 16f, ty = (float)elem.To![1] / 16f, tz = (float)elem.To![2] / 16f;

                float w = MathF.Abs(tx - fx), h = MathF.Abs(ty - fy), l = MathF.Abs(tz - fz);
                if (w > 1e-5f && h > 1e-5f && l > 1e-5f)
                {
                    Vector3 localCenter = new Vector3(w * 0.5f, h * 0.5f, l * 0.5f);
                    Vector3 childPosition = Vector3.Transform(localCenter, world);
                    Quaternion childOri = ExtractPureRotation(world);
                    Vector3 axisScale = ExtractAxisScale(world);

                    Vector3 halfExtents = new Vector3(w * 0.5f * axisScale.X, h * 0.5f * axisScale.Y, l * 0.5f * axisScale.Z);

                    manualBoxes.Add(new LocalBox { LocalPosition = childPosition, LocalOrientation = childOri, HalfExtents = halfExtents });
                    childMasses.Add(halfExtents.X * 2f * halfExtents.Y * 2f * halfExtents.Z * 2f);
                }
            }

            if (elem.Children == null) return;
            for (int i = 0; i < elem.Children.Length; i++)
            {
                ShapeElement child = elem.Children[i];
                string childName = child.Name ?? string.Empty;
                string childPath = path.Length == 0 ? childName : path + "/" + childName;
                AppendSelectedElementsRecursive(child, world, childPath, selected, childMasses, manualBoxes);
            }
        }

        // ── pose ─────────────────────────────────────────────────────────────────

        private bool TryGetPose(out Vec3d bodyPos, out Quaternion bodyOri)
        {
            bodyPos = new Vec3d();
            bodyOri = Quaternion.Identity;

            if (entity?.Pos == null) return false;

            EntityPos pos = entity.Pos;

            bodyOri = ExtractPureRotation(
                Matrix4x4.CreateRotationY(MathF.PI / 2f) *
                Matrix4x4.CreateFromYawPitchRoll(pos.Yaw, pos.Pitch, pos.Roll));

            Vector3 localOffset = Vector3.Transform(
                new Vector3(-0.5f, 0f, -0.5f) + localCenterOfMassOffset, bodyOri);

            bodyPos.Set(pos.X + localOffset.X, pos.Y + localOffset.Y, pos.Z + localOffset.Z);
            return true;
        }
        
        public bool TryLocalToWorld(Vector3 local, out Vec3d world)
        {
            world = new Vec3d();
            if (!TryGetPose(out Vec3d bodyPos, out Quaternion bodyOri)) return false;
            Vector3 offset = Vector3.Transform(local, bodyOri);
            world.Set(bodyPos.X + offset.X, bodyPos.Y + offset.Y, bodyPos.Z + offset.Z);
            return true;
        }
        
        public bool TryWorldToLocal(Vec3d worldPoint, out Vector3 local)
        {
            local = Vector3.Zero;
            if (!TryGetPose(out Vec3d bodyPos, out Quaternion bodyOri)) return false;
            Vector3 rel = new(
                (float)(worldPoint.X - bodyPos.X),
                (float)(worldPoint.Y - bodyPos.Y),
                (float)(worldPoint.Z - bodyPos.Z));
            local = Vector3.Transform(rel, Quaternion.Inverse(bodyOri));
            return true;
        }

        // ── geometry utilities ────────────────────────────────────────────────────

        private static Cuboidd CreateAabbFromOrientedBox(Vec3d center, Quaternion orientation, Vector3 halfExtents)
        {
            Vector3 r = Vector3.Transform(Vector3.UnitX, orientation);
            Vector3 u = Vector3.Transform(Vector3.UnitY, orientation);
            Vector3 f = Vector3.Transform(Vector3.UnitZ, orientation);

            float ex = MathF.Abs(r.X) * halfExtents.X + MathF.Abs(u.X) * halfExtents.Y + MathF.Abs(f.X) * halfExtents.Z;
            float ey = MathF.Abs(r.Y) * halfExtents.X + MathF.Abs(u.Y) * halfExtents.Y + MathF.Abs(f.Y) * halfExtents.Z;
            float ez = MathF.Abs(r.Z) * halfExtents.X + MathF.Abs(u.Z) * halfExtents.Y + MathF.Abs(f.Z) * halfExtents.Z;

            return new Cuboidd(center.X - ex, center.Y - ey, center.Z - ez, center.X + ex, center.Y + ey, center.Z + ez);
        }

        private static Matrix4x4 CreateVsElementLocalMatrix(ShapeElement elem, float parentXSign)
        {
            float ox = elem.RotationOrigin != null ? (float)elem.RotationOrigin[0] / 16f : 0f;
            float oy = elem.RotationOrigin != null ? (float)elem.RotationOrigin[1] / 16f : 0f;
            float oz = elem.RotationOrigin != null ? (float)elem.RotationOrigin[2] / 16f : 0f;

            float fx, fy, fz;
            if (ElementHasRealBox(elem)) { fx = (float)elem.From![0] / 16f; fy = (float)elem.From![1] / 16f; fz = (float)elem.From![2] / 16f; }
            else { fx = ox; fy = oy; fz = oz; }

            float sx = elem.ScaleX == 0.0 ? 1f : (float)elem.ScaleX;
            float sy = elem.ScaleY == 0.0 ? 1f : (float)elem.ScaleY;
            float sz = elem.ScaleZ == 0.0 ? 1f : (float)elem.ScaleZ;

            Matrix4x4 rotX = elem.RotationX != 0.0 ? Matrix4x4.CreateRotationX((float)elem.RotationX * MathF.PI / 180f) : Matrix4x4.Identity;
            Matrix4x4 rotY = elem.RotationY != 0.0 ? Matrix4x4.CreateRotationY((float)elem.RotationY * MathF.PI / 180f) : Matrix4x4.Identity;
            Matrix4x4 rotZ = elem.RotationZ != 0.0 ? Matrix4x4.CreateRotationZ((float)elem.RotationZ * MathF.PI / 180f) : Matrix4x4.Identity;

            return Matrix4x4.CreateTranslation(fx - ox, fy - oy, fz - oz)
                 * Matrix4x4.CreateScale(sx, sy, sz)
                 * (rotZ * rotY * rotX)
                 * Matrix4x4.CreateTranslation(ox, oy, oz);
        }

        private static bool ElementHasRealBox(ShapeElement elem)
        {
            if (elem.From == null || elem.To == null) return false;
            return MathF.Abs((float)elem.To[0] - (float)elem.From[0]) > 1e-5f &&
                   MathF.Abs((float)elem.To[1] - (float)elem.From[1]) > 1e-5f &&
                   MathF.Abs((float)elem.To[2] - (float)elem.From[2]) > 1e-5f;
        }

        private static Quaternion ExtractPureRotation(Matrix4x4 m)
        {
            Vector3 x = Vector3.Normalize(Vector3.TransformNormal(Vector3.UnitX, m));
            Vector3 y = Vector3.Normalize(Vector3.TransformNormal(Vector3.UnitY, m));
            Vector3 z = Vector3.Normalize(Vector3.Cross(x, y));
            if (z.LengthSquared() <= 1e-10f) return Quaternion.Identity;
            y = Vector3.Normalize(Vector3.Cross(z, x));
            return Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(new Matrix4x4(
                x.X, x.Y, x.Z, 0f, y.X, y.Y, y.Z, 0f, z.X, z.Y, z.Z, 0f, 0f, 0f, 0f, 1f)));
        }

        private static Vector3 ExtractAxisScale(Matrix4x4 m) => new Vector3(
            Vector3.TransformNormal(Vector3.UnitX, m).Length(),
            Vector3.TransformNormal(Vector3.UnitY, m).Length(),
            Vector3.TransformNormal(Vector3.UnitZ, m).Length());

        private static Vector3 ComputeCenterOfMass(List<LocalBox> children, List<float> masses)
        {
            float total = 0f; Vector3 sum = Vector3.Zero;
            for (int i = 0; i < children.Count; i++) { total += masses[i]; sum += children[i].LocalPosition * masses[i]; }
            if (total <= 0f) throw new InvalidOperationException("Compound total mass must be > 0.");
            return sum / total;
        }

        private bool MatchesAnySelector(string path)
        {
            if (selectors == null || selectors.Length == 0) return true;
            for (int i = 0; i < selectors.Length; i++)
            {
                string sel = selectors[i]?.Trim() ?? "";
                if (sel.EndsWith("/**") && (path == sel[..^3] || path.StartsWith(sel[..^3] + "/"))) return true;
                if (sel.EndsWith("/*") && path.StartsWith(sel[..^2] + "/")) return true;
                if (WildcardMatch(path, sel)) return true;
            }
            return false;
        }

        private static bool WildcardMatch(string text, string pattern)
        {
            int t = 0, p = 0, star = -1, match = 0;
            while (t < text.Length)
            {
                if (p < pattern.Length && (pattern[p] == '?' || pattern[p] == text[t])) { t++; p++; continue; }
                if (p < pattern.Length && pattern[p] == '*') { star = p++; match = t; continue; }
                if (star != -1) { p = star + 1; t = ++match; continue; }
                return false;
            }
            while (p < pattern.Length && pattern[p] == '*') p++;
            return p == pattern.Length;
        }

        // ── debug rendering ───────────────────────────────────────────────────────

        public void DebugRender(ICoreClientAPI capi)
        {
            if (capi == null || entity == null || !entity.Alive) return;
            if (!TryGetPose(out Vec3d bodyPosD, out Quaternion bodyOrientation)) return;

            int color = ColorUtil.ToRgba(255, 0, 255, 0);
            Matrix4x4 bodyRotMat = Matrix4x4.CreateFromQuaternion(bodyOrientation);

            for (int i = 0; i < manualChildBoxes.Count; i++)
            {
                LocalBox child = manualChildBoxes[i];
                Vector3 localOffset = Vector3.Transform(child.LocalPosition, bodyOrientation);
                Vector3 worldCenter = new Vector3(
                    (float)(bodyPosD.X + localOffset.X),
                    (float)(bodyPosD.Y + localOffset.Y),
                    (float)(bodyPosD.Z + localOffset.Z));

                Matrix4x4 childLocalRot = Matrix4x4.CreateFromQuaternion(child.LocalOrientation);
                Quaternion childWorldOri = ExtractPureRotation(childLocalRot * bodyRotMat);

                DrawOrientedBoxOutline(capi, worldCenter, childWorldOri, child.HalfExtents, color);
            }
        }

        private static void DrawOrientedBoxOutline(ICoreClientAPI capi, Vector3 center, Quaternion ori, Vector3 he, int color)
        {
            Vector3[] c =
            {
                Corner(center,ori,he,-1,-1,-1), Corner(center,ori,he, 1,-1,-1),
                Corner(center,ori,he, 1,-1, 1), Corner(center,ori,he,-1,-1, 1),
                Corner(center,ori,he,-1, 1,-1), Corner(center,ori,he, 1, 1,-1),
                Corner(center,ori,he, 1, 1, 1), Corner(center,ori,he,-1, 1, 1),
            };
            Line(capi, c[0], c[1], color); Line(capi, c[1], c[2], color); Line(capi, c[2], c[3], color); Line(capi, c[3], c[0], color);
            Line(capi, c[4], c[5], color); Line(capi, c[5], c[6], color); Line(capi, c[6], c[7], color); Line(capi, c[7], c[4], color);
            Line(capi, c[0], c[4], color); Line(capi, c[1], c[5], color); Line(capi, c[2], c[6], color); Line(capi, c[3], c[7], color);
        }

        private static Vector3 Corner(Vector3 center, Quaternion ori, Vector3 he, float sx, float sy, float sz)
            => center + Vector3.Transform(new Vector3(he.X * sx, he.Y * sy, he.Z * sz), ori);

        private static void Line(ICoreClientAPI capi, Vector3 a, Vector3 b, int color)
        {
            BlockPos o = new BlockPos((int)Math.Floor(a.X), (int)Math.Floor(a.Y), (int)Math.Floor(a.Z));
            capi.Render.RenderLine(o, (float)(a.X - o.X), (float)(a.Y - o.Y), (float)(a.Z - o.Z),
                                      (float)(b.X - o.X), (float)(b.Y - o.Y), (float)(b.Z - o.Z), color);
        }
    }
}