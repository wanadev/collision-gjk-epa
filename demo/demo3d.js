'use strict';

// get the canvas DOM element
var canvas3D = document.getElementById('canvas3d');

// load the 3D engine
var engine = new BABYLON.Engine(canvas3D, true);

// Set the basics
var scene = new BABYLON.Scene(engine);
scene.clearColor = new BABYLON.Color3(1, 1, 1);
var camera = new BABYLON.ArcRotateCamera("ArcRotateCamera", 1, 0.8, 10, new BABYLON.Vector3(0, 0, 0), scene);
camera.setTarget(BABYLON.Vector3.Zero());
camera.attachControl(canvas3D, false);
var light = new BABYLON.HemisphericLight('light1', new BABYLON.Vector3(0, 1, 0), scene);
light.intensity = 0.5;

var meshMaterial = new BABYLON.StandardMaterial("mesh", scene);
meshMaterial.wireframe = true;

var ghostMaterial = new BABYLON.StandardMaterial("ghost", scene);
ghostMaterial.alpha = 0.5;

var meshes = [];
var box1 = BABYLON.Mesh.CreateBox("obj1", 2, scene);
box1.material = meshMaterial;
meshes.push(box1);
var box2 = BABYLON.Mesh.CreateBox("obj2", 2, scene);
box2.material = meshMaterial;
meshes.push(box2);

var ground = BABYLON.Mesh.CreateGround("ground", 1000, 1000, 1, scene, false);
ground.visibility = 0;

var response3D = new BABYLON.Vector3();
var ghost;

// run the render loop
engine.runRenderLoop(function() {
    scene.render();
});

// the canvas/window resize event handler
window.addEventListener('resize', function() {
    engine.resize();
});


var selected3D = false;
var callbacks3D = {
    onMouseDown: function(event) {
        var pickingInfo = scene.pick(scene.pointerX, scene.pointerY);
        if (pickingInfo.hit && pickingInfo.pickedMesh !== ground) {
            selected3D = pickingInfo.pickedMesh;
            camera.inputs.detachElement(canvas3D);
            ghost = pickingInfo.pickedMesh.clone();
            ghost.material = ghostMaterial;
        }
    },
    onMouseUp: function(event) {
        selected3D = false;
        camera.inputs.attachElement(canvas3D);
        if (ghost) ghost.dispose();
    },
    onMouseMove: function(event) {
        if (selected3D) {
            var startingPoint;
            startingPoint = getGroundPosition(event);

            selected3D.position.copyFrom(startingPoint);
            var vectors = selected3D.getBoundingInfo().boundingBox.vectorsWorld.slice();
            response3D.copyFromFloats(0, 0, 0);
            for (var i = 0; i < meshes.length; i++) {
                if (meshes[i] !== selected3D) {
                    var res = Collisiongjkepa.intersect(vectors, meshes[i].getBoundingInfo().boundingBox.vectorsWorld);
                    if (res) {
                        response3D.addInPlace(res);
                    }
                }
            }
            ghost.position.copyFrom(selected3D.position.subtract(response3D));
        }
    }
}

canvas3D.addEventListener('pointermove', callbacks3D.onMouseMove);
canvas3D.addEventListener('pointerdown', callbacks3D.onMouseDown);
canvas3D.addEventListener('pointerup', callbacks3D.onMouseUp);
///ui

var getGroundPosition = function () {
    // Use a predicate to get position on the ground
    var pickinfo = scene.pick(scene.pointerX, scene.pointerY, function (mesh) { return mesh == ground; });
    if (pickinfo.hit) {
        return pickinfo.pickedPoint;
    }

    return null;
};
