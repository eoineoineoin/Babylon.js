/* eslint-disable-next-line import/no-internal-modules */
import { canvas, engine } from "./index";
import * as BABYLON from "@dev/core";
import "@dev/loaders";

var createScene = function () {
    var scene = new BABYLON.Scene(engine);
var gravityVector = new BABYLON.Vector3(0, -9.81, 0);
var physicsPlugin = new BABYLON.HavokPlugin();
scene.enablePhysics(gravityVector, physicsPlugin);

    //Adding a light
    var light = new BABYLON.PointLight("Omni", new BABYLON.Vector3(20, 20, 100), scene);

    //Adding an Arc Rotate Camera
    var camera = new BABYLON.ArcRotateCamera("Camera", 0, 0.8, 100, BABYLON.Vector3.Zero(), scene);
    camera.attachControl(canvas, false);

console.log(BABYLON.GLTF2.GLTFLoader.RegisterExtension);

    // The first parameter can be used to specify which mesh to import. Here we import all meshes
    BABYLON.SceneLoader.ImportMesh("", "https://eoinrul.es/tmp/", "Lantern.glb", scene, function (newMeshes) {
        // Set the target of the camera to the first imported mesh
        //camera.target = newMeshes[0];
        console.log(newMeshes);
    });

    // Move the light with the camera
    scene.registerBeforeRender(function () {
        light.position = camera.position;
    });

    return scene;
}