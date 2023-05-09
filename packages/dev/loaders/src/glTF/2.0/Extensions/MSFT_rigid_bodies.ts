import { TransformNode } from "core/Meshes/transformNode";
import { GLTFLoader } from "../glTFLoader";
import { IGLTFLoaderExtension } from "../glTFLoaderExtension";
import { GLTF2 } from "loaders/glTF";
import { PhysicsShape, PhysicsShapeBox, PhysicsShapeCapsule, PhysicsShapeContainer, PhysicsShapeConvexHull, PhysicsShapeCylinder, PhysicsShapeMesh, PhysicsShapeSphere } from "core/Physics/v2/physicsShape";
import { AbstractMesh } from "core/Meshes/abstractMesh";
import { Nullable } from "core/types";
import { HavokPlugin } from "core/Physics/v2/Plugins/havokPlugin";
import { Matrix, Quaternion, Vector3 } from "core/Maths/math.vector";
import { PhysicsShapeType, PhysicsConstraintAxis, PhysicsMassProperties, PhysicsMotionType } from "core/Physics/v2/IPhysicsEnginePlugin";
import { PhysicsMaterialCombineMode } from "core/Physics/v2/physicsMaterial";
import { PhysicsBody } from "core/Physics/v2/physicsBody";
import { Physics6DoFConstraint, Physics6DoFLimit } from "core/Physics/v2/physicsConstraint";

import "core/Physics/physicsEngineComponent";

namespace MSFT_CollisionPrimitives
{
    export class Sphere
    {
        radius : number = 0.5;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Box
    {
        size : [number, number, number] = [1, 1, 1];

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Capsule
    {
        height: number = 0.5;
        radius: number = 0.25;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Cylinder
    {
        height: number = 0.5;
        radius: number = 0.25;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Convex
    {
        mesh: number;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class TriMesh
    {
        mesh: number;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Collider
    {
        collisionSystems : Array<string>;
        collideWithSystems : Array<string>;
        notCollideWithSystems : Array<string>;

        //<TODO.eoin Spec should probably be zero-or-one-of, and have extensions/extras
        sphere? : Sphere;
        box? : Box;
        capsule? : Capsule;
        cylinder? : Cylinder;
        convex? : Convex;
        trimesh? : TriMesh;
    }

    export class SceneExt
    {
        colliders: Array<Collider>;
    }
}

namespace MSFT_RigidBodies
{
    export class RigidBody
    {
        isKinematic : boolean = false;
        inverseMass : number = 1;
        centerOfMass : [number, number, number] = [0, 0, 0];
        inertiaOrientation: [number, number, number, number] = [0, 0, 0, 1];
        inverseInertiaTensor: [number, number, number] = [1, 1, 1];
        linearVelocity : [number, number, number] = [0, 0, 0];
        angularVelocity : [number, number, number] = [0, 0, 0];
        gravityFactor : number = 1;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class Constraint
    {
        min? : number;
        max? : number;
        springConstant? : number;
        springDamping? : number;

        linearAxes? : Array<number>;
        angularAxes? : Array<number>;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class JointLimitSet
    {
        limits: Array<Constraint>;
    }

    export class Joint
    {
        connectedNode : number;
        jointLimits : number;
        enableCollision : boolean;
    }

    export class NodeExt
    {
        rigidBody? : RigidBody;
        collider? : number;
        physicsMaterial? : number;
        joint? : Joint;

        extensions : {[key: string]: any} = {}
        extras : {[key: string]: any} = {}
    }

    export class PhysicsMaterial
    {
        staticFriction? : number;
        dynamicFriction? : number;
        restitution? : number;
        restitutionCombineMode? : string;
        frictionCombineMode? : string;
    }

    export class SceneExt
    {
        physicsMaterials? : Array<PhysicsMaterial>;
        physicsJointLimits? : Array<JointLimitSet>;
    }
}

class DeferredJoint
{
    jointInfo : MSFT_RigidBodies.Joint;
    pivotA : TransformNode;
    pivotB : TransformNode;
}

export class MSFT_RigidBodies_Plugin implements IGLTFLoaderExtension  {
    public name : string = "MSFT_rigid_bodies";
    public enabled : boolean = true;
    private loader : GLTF2.GLTFLoader;
    private _deferredJoints : Array<DeferredJoint> = []
    private _deferredBodies : Array<() => void> = [];
    private _deferredColliders : Array<() => Promise<void>> = [];
    private _physicsVersion : number;
    private _layerNames: string[] = [];

    public constructor(loader : GLTF2.GLTFLoader)
    {
        this.loader = loader;

        let physicsEngine = loader.babylonScene.getPhysicsEngine();

        if (!physicsEngine) {
            var hk = new HavokPlugin();
            const scene = loader.babylonScene;
            scene.enablePhysics(new Vector3(0, -9.8, 0), hk);
            physicsEngine = loader.babylonScene.getPhysicsEngine();
        }
        this._physicsVersion = physicsEngine ? physicsEngine.getPluginVersion() : -1;
    }

    public dispose() : void
    {
    }

    protected async _constructCollider(
        context: string, sceneNode: AbstractMesh, gltfNode: GLTF2.INode,
        colliderData: MSFT_CollisionPrimitives.Collider,
        rbExt : MSFT_RigidBodies.SceneExt, assign: ((babylonMesh: TransformNode) => void)) : Promise<Nullable<PhysicsShape>> {

        let scene = this.loader.babylonScene;
        let physicsShape: Nullable<PhysicsShape> = null;
        if (colliderData.sphere != undefined)
        {
            var sphere = new PhysicsShapeSphere(Vector3.Zero(), colliderData.sphere.radius, scene);
            physicsShape = sphere;
        }
        else if (colliderData.box != undefined)
        {
            const size = Vector3.FromArray(colliderData.box.size);;
            var box = new PhysicsShapeBox(Vector3.Zero(), Quaternion.Identity(), size, scene);
            physicsShape = box;
        }
        else if (colliderData.cylinder != undefined)
        {
            const pointA = new Vector3(0, 0.5 * colliderData.cylinder.height, 0);
            const pointB = new Vector3(0, 0.5 * -colliderData.cylinder.height, 0);
            var cylinder = new PhysicsShapeCylinder(pointA, pointB, colliderData.cylinder.radius, scene);
            physicsShape = cylinder;
        }
        else if (colliderData.capsule != undefined)
        {
            const capsuleData = colliderData.capsule;
            const pointA = new Vector3(0, 0.5 * (capsuleData.height - capsuleData.radius), 0);
            const pointB = new Vector3(0, 0.5 * (-capsuleData.height + capsuleData.radius), 0);
            var capsule = new PhysicsShapeCapsule(pointA, pointB, capsuleData.radius, scene);
            physicsShape = capsule;
        }
        else if (colliderData.convex != undefined)
        {
            var meshData = this.loader.gltf.meshes![colliderData.convex.mesh];
            //<todo.eoin I just want to access the mesh object here; not create one in the scene
            //@ts-ignore _loadMeshAsync is private:
            var convexMesh = await this.loader._loadMeshAsync(context.concat("/collider"), gltfNode, meshData, assign) as Mesh;
            convexMesh.isVisible = false;
            convexMesh.name = convexMesh.name + "_Collider";
            convexMesh.parent = sceneNode;

            // Looks like there's some very strange behavior of the nodes we get from _loadMeshAsync.
            // Transform nodes have an identity transform, but AbstractMesh nodes have _something_ else.
            convexMesh.position = Vector3.Zero();
            convexMesh.rotationQuaternion = Quaternion.Identity();
            convexMesh.scaling = Vector3.One();

            physicsShape = new PhysicsShapeConvexHull(convexMesh, scene);
        }
        else if (colliderData.trimesh != undefined)
        {
            var meshData = this.loader.gltf.meshes![colliderData.trimesh.mesh];
            //@ts-ignore _loadMeshAsync is private:
            var meshShape = await this.loader._loadMeshAsync(context.concat("/collider"), gltfNode, meshData, assign) as Mesh;
            meshShape.isVisible = false;
            meshShape.name = meshShape.name + "_Collider";
            meshShape.setParent(sceneNode);
            meshShape.position = Vector3.Zero();
            meshShape.rotationQuaternion = Quaternion.Identity();
            meshShape.scaling = Vector3.One();

            physicsShape = new PhysicsShapeMesh(meshShape, scene);
            physicsShape = new PhysicsShape({ type: PhysicsShapeType.MESH, parameters: { mesh: meshShape, includeChildMeshes: true }}, scene);
        }

        if (physicsShape == undefined) {
            return null;
        }

        // Add collision filter info
        {
            let filterMembership = 0;
            let filterCollideWith = 0;
            let hasFilterInfo = false;
            if (colliderData.collisionSystems) {
                filterMembership = this._layerNamesToMask(colliderData.collideWithSystems);
                hasFilterInfo = true;
            }
            if (colliderData.collideWithSystems) {
                filterCollideWith = this._layerNamesToMask(colliderData.collideWithSystems);
                hasFilterInfo = true;
            } else if (colliderData.notCollideWithSystems) {
                filterCollideWith = ~this._layerNamesToMask(colliderData.notCollideWithSystems);
                hasFilterInfo = true;
            }

            if (physicsShape && hasFilterInfo) {
                physicsShape.filterMembershipMask = filterMembership;
                physicsShape.filterCollideMask = filterCollideWith;
            }
        }

        if (!gltfNode.extensions) {
            return physicsShape;
        }

        var extData = gltfNode.extensions.MSFT_rigid_bodies as MSFT_RigidBodies.NodeExt;
        if (extData.physicsMaterial != null) {
            var mat : MSFT_RigidBodies.PhysicsMaterial = rbExt!.physicsMaterials![extData.physicsMaterial];

            //<todo.eoin Add rest of material props
            physicsShape.material = {
                friction: mat.dynamicFriction?? 0.5,
                staticFriction: mat.staticFriction?? 0.5,
                restitution: mat.restitution?? 0.0,
                frictionCombine: this._materialCombineModeToNative(mat.frictionCombineMode)?? PhysicsMaterialCombineMode.MAXIMUM,
                restitutionCombine: this._materialCombineModeToNative(mat.frictionCombineMode) ?? PhysicsMaterialCombineMode.MINIMUM
            };
        }

        //<todo.eoin Make no-materials triggers!
        return physicsShape;
    }

    private _materialCombineModeToNative(combine: string | undefined): PhysicsMaterialCombineMode | undefined {
        if (!combine) {
            return undefined;
        } else if (combine == "AVERAGE") {
            return PhysicsMaterialCombineMode.ARITHMETIC_MEAN;
        } else if (combine == "MINIMUM") {
            return PhysicsMaterialCombineMode.MINIMUM;
        } else if (combine == "MAXIMUM") {
            return PhysicsMaterialCombineMode.MAXIMUM;
        } else if (combine == "MULTIPLY") {
            return PhysicsMaterialCombineMode.MULTIPLY;
        }
        return undefined;
    }

    private _layerNamesToMask(names: Array<string>): number {
        let mask = 0;
        for (const name of names) {
            const idx = this._layerNames.indexOf(name);
            if (idx == -1) {
                mask |= 1 << this._layerNames.length;
                this._layerNames.push(name);
            } else {
                mask |= 1 << idx;
            }
        }
        return mask;
    }

    //<todo.eoin Split this up properly
    protected async _constructNodeObjects(
        context : string, sceneNode : AbstractMesh, gltfNode : GLTF2.INode,
        assign : ((babylonMesh: TransformNode) => void)) {
        var extData = gltfNode.extensions!.MSFT_rigid_bodies as MSFT_RigidBodies.NodeExt;

        //<todo.eoin Don't think Babylon supports triggers?
        if (extData.collider != null && extData.physicsMaterial != null)
        {
            let ext : MSFT_CollisionPrimitives.SceneExt = this.loader.gltf.extensions!.MSFT_collision_primitives;
            var rbExt : MSFT_RigidBodies.SceneExt = this.loader.gltf.extensions!.MSFT_rigid_bodies;
            let collider : MSFT_CollisionPrimitives.Collider = ext.colliders[extData.collider];
            this._deferredColliders.push(async () => {
                let physicsShape = await this._constructCollider(context, sceneNode, gltfNode, collider, rbExt, assign);

                if (physicsShape == null) {
                    return;
                }

                let rigidBodyNode = this._getParentRigidBody(sceneNode);
                if (rigidBodyNode == null) {
                    // This is a static collider. Just make a new body
                    //<todo.eoin Maybe could just put a body on the root?
                    // Remove the parent to get rid of the transform added by the glTF loader (same as dynamic bodies)
                    sceneNode.setParent(null);
                    sceneNode.physicsBody = new PhysicsBody(sceneNode, PhysicsMotionType.STATIC, false, sceneNode.getScene());
                    rigidBodyNode = sceneNode;
                }

                // Now, add the shape to the body:
                if (!rigidBodyNode.physicsBody!.shape) {
                    rigidBodyNode.physicsBody!.shape = new PhysicsShapeContainer(sceneNode.getScene());
                    //<todo.eoin Optimize the case of a single collider with identity transform
                }

                let containerShape = <PhysicsShapeContainer>rigidBodyNode.physicsBody!.shape;

                //<todo.eoin Would like to just use the addChildFromParent() interface here, but the root
                //< of a glTF file gets an additional (1,-1,1) scale, which the physics simulation doesn't
                //< account for (should probably bake it into the generated shapes) but for now, just manually
                //< calculate a childToParent transform which adds that scaling back in:
                const childToWorld = sceneNode.computeWorldMatrix(true);
                const parentToWorld = rigidBodyNode.computeWorldMatrix(true);
                let parentScaling = new Vector3();
                parentToWorld.decompose(parentScaling);
                const childToParent = childToWorld.multiply(Matrix.Invert(parentToWorld)).multiply(Matrix.Scaling(parentScaling.x, parentScaling.y, parentScaling.z));
                const translation = new Vector3();
                const rotation = new Quaternion();
                const scale = new Vector3();
                childToParent.decompose(scale, rotation, translation);
                containerShape.addChild(physicsShape, translation, rotation, scale);
            });
        }

        if (extData.rigidBody != null) {
            this._deferredBodies.push(() => {

                if (!extData.rigidBody)
                    return;

                sceneNode.setParent(null);
                const motionType = extData.rigidBody.isKinematic ? PhysicsMotionType.ANIMATED : PhysicsMotionType.DYNAMIC;
                sceneNode.physicsBody = new PhysicsBody(sceneNode, motionType, false, this.loader.babylonScene);

                if (extData.rigidBody.linearVelocity != null) {
                    sceneNode.physicsBody.setLinearVelocity(Vector3.FromArray(extData.rigidBody.linearVelocity));
                }

                if (extData.rigidBody.angularVelocity != null) {
                    sceneNode.physicsBody.setAngularVelocity(Vector3.FromArray(extData.rigidBody.angularVelocity));
                }

                var massProps : PhysicsMassProperties = {};

                if (extData.rigidBody!.centerOfMass != null) {
                    massProps.centerOfMass = Vector3.FromArray(extData.rigidBody!.centerOfMass);
                }

                if (extData.rigidBody.inertiaOrientation != null) {
                    massProps.inertiaOrientation = Quaternion.FromArray(extData.rigidBody.inertiaOrientation);
                }

                const safeInv = (x : number) => x == 0 ? 0 : 1 / x;
                if (extData.rigidBody.inverseInertiaTensor != null) {
                    const it = extData.rigidBody.inverseInertiaTensor;
                    massProps.inertia = new Vector3(safeInv(it[0]), safeInv(it[1]), safeInv(it[2]));
                }

                if (extData.rigidBody.inverseMass != null) {
                    massProps.mass = safeInv(extData.rigidBody.inverseMass);
                }

                sceneNode.physicsBody.setMassProperties(massProps);
            });
        }

        if (extData.joint != null)
        {
            // Can load some joint info, but this can cause circular awaits...
            var jointInfo = {jointInfo: extData.joint, pivotA: sceneNode,
                pivotB: this.loader.gltf.nodes![extData.joint.connectedNode]._babylonTransformNode!};
            this._deferredJoints.push(jointInfo);
        }
     }


    public async loadNodeAsync(context : string, node : GLTF2.INode, assign : ((babylonMesh: TransformNode) => void))
    {
        if (node.extensions != undefined
            && node.extensions.MSFT_rigid_bodies != undefined
           && this._physicsVersion == 2)
        {
            //<todo.eoin Can this really ever return a transform node? Will need to handle
            var loaded : AbstractMesh = <AbstractMesh>await this.loader.loadNodeAsync(context, node, assign);
            await this._constructNodeObjects(context, loaded, node, assign);

            return loaded;
        }

        return this.loader.loadNodeAsync(context, node, assign);
    }



    public loadSceneAsync(context : string, scene : GLTF2.IScene) {
        return this.loader.loadSceneAsync(context, scene);
    }

    public async onReady() {
        if (this._physicsVersion != 2) {
            return;
        }

        for (let f of this._deferredBodies)
            f();

        for (let f of this._deferredColliders)
            await f();

        for(let joint of this._deferredJoints) {
            this.make6DoFJoint(joint);
        }
    }

    protected make6DoFJoint(joint : DeferredJoint) {
        // Get transform from parent RB?
        var rbA = this._getParentRigidBody(joint.pivotA)!;
        var rbB = this._getParentRigidBody(joint.pivotB)!;

        var rbAToWorld = rbA.computeWorldMatrix(true);
        var rbAScale = Vector3.One();
        rbAToWorld.decompose(rbAScale);

        var pivotAToWorld = joint.pivotA.computeWorldMatrix(true);
        var pivotAToBodyA = pivotAToWorld.multiply(Matrix.Invert(rbAToWorld));
        var pivotOrientationInA = Quaternion.Identity();
        pivotAToBodyA.decompose(undefined, pivotOrientationInA);
        var pivotTranslationInA = Vector3.TransformCoordinates(Vector3.Zero(), pivotAToBodyA).multiply(rbAScale);


        var rbBToWorld = rbB.computeWorldMatrix(true);
        var rbBScale = Vector3.One();
        rbBToWorld.decompose(rbBScale);

        var pivotBToWorld = joint.pivotB.computeWorldMatrix(true);
        var pivotBToBodyB = pivotBToWorld.multiply(Matrix.Invert(rbBToWorld));
        var pivotOrientationInB = Quaternion.Identity();
        pivotBToBodyB.decompose(undefined, pivotOrientationInB);
        var pivotTranslationInB = Vector3.TransformCoordinates(Vector3.Zero(), pivotBToBodyB).multiply(rbBScale);

        var sceneExt : MSFT_RigidBodies.SceneExt = this.loader.gltf.extensions!.MSFT_rigid_bodies;

        var limitSet = sceneExt.physicsJointLimits![joint.jointInfo.jointLimits].limits;
        const nativeLimits: Physics6DoFLimit[] = []
        for (const l of limitSet) {
            if (l.linearAxes) {
                for (const axIdx of l.linearAxes) {
                    if (axIdx == 0) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.LINEAR_X, minLimit: l.min, maxLimit: l.max});
                    }
                    if (axIdx == 1) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.LINEAR_Y, minLimit: l.min, maxLimit: l.max});
                    }
                    if (axIdx == 2) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.LINEAR_Z, minLimit: l.min, maxLimit: l.max});
                    }
                  }
            } else if (l.angularAxes) {
                for (const axIdx of l.angularAxes) {
                    if (axIdx == 0) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.ANGULAR_X, minLimit: l.min, maxLimit: l.max});
                    }
                    if (axIdx == 1) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.ANGULAR_Y, minLimit: l.min, maxLimit: l.max});
                    }
                    if (axIdx == 2) {
                        nativeLimits.push({axis: PhysicsConstraintAxis.ANGULAR_Z, minLimit: l.min, maxLimit: l.max});
                    }
                }
            }
        }


        const axisA = (new Vector3(1,0,0)).applyRotationQuaternion(pivotOrientationInA);
        const axisB = (new Vector3(1,0,0)).applyRotationQuaternion(pivotOrientationInB);

        const perpAxisA = (new Vector3(0,-1,0)).applyRotationQuaternion(pivotOrientationInA);
        const perpAxisB = (new Vector3(0,-1,0)).applyRotationQuaternion(pivotOrientationInB);
        var constraintInstance = new Physics6DoFConstraint( {
            pivotA: pivotTranslationInA, pivotB: pivotTranslationInB,
            axisA: axisA, axisB: axisB, perpAxisA: perpAxisA, perpAxisB: perpAxisB}, nativeLimits, this.loader.babylonScene);
        rbA.physicsBody!.addConstraint(rbB.physicsBody!, constraintInstance);
        constraintInstance.isCollisionsEnabled = !!joint.jointInfo.enableCollision;
    }

    protected _getParentRigidBody(node : TransformNode) : Nullable<TransformNode>
    {
        var highestParentRb = null;
        while(node != null)
        {
            if('physicsBody' in node && node.physicsBody != null)
            {
                highestParentRb = node;
            }
            node = <TransformNode>node.parent;
        }

        return highestParentRb;
    }
}

GLTFLoader.RegisterExtension(
    //@ts-ignore
   "MSFT_rigid_bodies", function (loader) { return new MSFT_RigidBodies_Plugin(loader); } );