'use strict';

var expect = require('expect.js');
var BABYLON = require('babylonjs');

describe('wnp/lib/collision-gjk-epa', function() {
    var collisionner = require('../collision-gjk-epa');

    var points1 = [];

    for (var i = 0; i < Math.PI * 2; i += Math.PI / 4) {
        points1.push(new BABYLON.Vector2(Math.cos(i) * 2, Math.sin(i) * 2));
    }

    var points3d1 = [];
    for (i = 0; i < Math.PI * 2; i += Math.PI /4) {
        for (var j = Math.PI / 2; j < 3 * Math.PI / 2; j += Math.PI /4) {
            points3d1.push(new BABYLON.Vector3(Math.cos(j) * Math.cos(i) * 2, Math.cos(j) * Math.sin(i) * 2, Math.sin(j) * 2));
        }
    }

    it('checks if two cloud of point intersect in 2d', function() {
        for (var k = 0; k < 1; k += 0.3) {
            var points2 = [];
            for (var i = 0; i < Math.PI * 2; i += Math.PI / 4) {
                points2.push(new BABYLON.Vector2(Math.cos(i) + k, Math.sin(i) + k));
            }

            expect(collisionner.isIntersecting(points1, points2)).to.be(true);
            var result = collisionner.intersect(points1, points2);

            for (i = 0; i < points2.length; i++) {
                points2[i].addToRef(result, points2[i]);
            }

            expect(collisionner.isIntersecting(points1, points2)).to.be(false);
            expect(collisionner.intersect(points1, points2)).to.be.eql(false);
        }
    });

    it('checks if two cloud of point not intersect in 2d', function() {
        var points2 = [];
        for (var i = 0; i < Math.PI * 2; i += Math.PI / 4) {
            points2.push(new BABYLON.Vector2(Math.cos(i) + 3.1, Math.sin(i)));
        }
        expect(collisionner.isIntersecting(points1, points2)).to.be(false);
        expect(collisionner.intersect(points1, points2)).to.be.eql(false);
    });

    it('checks if two rectangles side by side do not intersect in 2d', function() {
        var rectangle1 = [
            new BABYLON.Vector2(-1, 0),
            new BABYLON.Vector2(-1, 1),
            new BABYLON.Vector2(1, 1),
            new BABYLON.Vector2(1, 0),
        ];
        var rectangle2 = [
            new BABYLON.Vector2(-10, 1),
            new BABYLON.Vector2(-10, 5),
            new BABYLON.Vector2(15, 5),
            new BABYLON.Vector2(15, 1),
        ];
        expect(collisionner.isIntersecting(rectangle1, rectangle2)).to.be(true);
        expect(collisionner.intersect(rectangle1, rectangle2)).to.be.eql({ x: 0, y: 0 });
    });

    it('checks if two rectangles side by side do not intersect in 2d', function() {
        var colliderBox = [
            new BABYLON.Vector2(10, 0),
            new BABYLON.Vector2(10, -3),
            new BABYLON.Vector2(-10, -3),
            new BABYLON.Vector2(-10, 0),
        ];
        
        var collidingBox = [
            new BABYLON.Vector2(1, 2),
            new BABYLON.Vector2(-5, 2),
            new BABYLON.Vector2(-5, 0),
            new BABYLON.Vector2(1, 0),
        ];
        
        expect(collisionner.isIntersecting(colliderBox, collidingBox)).to.be(true);
        expect(collisionner.intersect(colliderBox, collidingBox)).to.be.eql({ x: 0, y: 0 });
    });

    it('checks if two cloud of point intersect in 3d', function() {

        for (var k = 0; k < 1; k += 0.3) {
            var points3d2 = [];
            for (var i = 0; i < Math.PI * 2; i += Math.PI / 4) {
                for (var j = Math.PI / 2; j < 3 * Math.PI / 2; j += Math.PI / 4) {
                    points3d2.push(new BABYLON.Vector3(Math.cos(j) * Math.cos(i) + k, Math.cos(j) * Math.sin(i) + k, Math.sin(j) + k));
                }
            }
            expect(collisionner.isIntersecting(points3d1, points3d2)).to.be(true);
            var result = collisionner.intersect(points3d1, points3d2);
            result.addToRef(result.clone().normalize().scale(0.05), result);
            for (i = 0; i < points3d2.length; i++) {
                points3d2[i].addToRef(result, points3d2[i]);
            }

            expect(collisionner.isIntersecting(points3d1, points3d2)).to.be(false);
            expect(collisionner.intersect(points3d1, points3d2)).to.be.eql(false);
        }

    });

    it('checks if two cloud of point not intersect in 3d', function() {
        var points3d2 = [];
        for (var i = 0; i < Math.PI * 2; i += Math.PI / 4) {
            for (var j = Math.PI / 2; j < 3 * Math.PI / 2; j += Math.PI / 4) {
                points3d2.push(new BABYLON.Vector3(Math.cos(j) * Math.cos(i) + 3.1, Math.cos(j) * Math.sin(i), Math.sin(j)));
            }
        }
        expect(collisionner.isIntersecting(points3d1, points3d2)).to.be(false);
        expect(collisionner.intersect(points3d1, points3d2)).to.be.eql(false);
    });
});