(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.Collisiongjkepa = f()}})(function(){var define,module,exports;return (function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
'use strict';

/**
 * Class to help in the collision in 2D and 3D.
 * To works the algorithm needs two convexe point cloud
 * like bounding box or smthg like this.
 * The function functions intersect and isIntersecting
 * helps to have informations about the collision between two object.
 *
 * @class wnp.helpers.CollisionGjkEpa
 * @constructor
 */
var Triangle = function(a, b, c) {
      this.a = a;
      this.b = b;
      this.c = c;
      this.n = a.constructor.Cross(b.subtract(a), c.subtract(a)).normalize();
  };


var CollisionGjkEpa = {

    EPSILON: 0.000001,

    /**
     * Method to get a normal of 3 points in 2D and 3D.
     *
     * @method _getNormal
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3} a
     * @param {BABYLON.Vector2|BABYLON.Vector3} b
     * @param {BABYLON.Vector2|BABYLON.Vector3} c
     * @return {BABYLON.Vector2|BABYLON.Vector3} The normal.
     */
    _getNormal: function(a, b, c) {
        var Dot = a.constructor.Dot;
        var ac = Dot(a, c); // perform aDot(c)
        var bc = Dot(b, c); // perform bDot(c)

        // perform b * a.Dot(c) - a * b.Dot(c)
        var r = b.scale(ac).subtract(a.scale(bc)).normalize();
        return r;
    },

    /**
     * Gets the barycenter of a cloud points.
     *
     * @method _getBarycenter
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} vertices the cloud points
     * @return {BABYLON.Vector2|BABYLON.Vector3} The barycenter.
     */
    _getBarycenter: function(vertices) {
        var avg = vertices[0].constructor.Zero();
        for (var i = 0; i < vertices.length; i++) {
            avg.addToRef(vertices[i], avg);
        }
        avg.scaleInPlace(1 / vertices.length, avg);
        return avg;
    },

    /**
     * Gets the farthest point of a cloud points.
     *
     * @method _getFarthestPointInDirection
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} vertices the cloud points.
     * @param {BABYLON.Vector2|BABYLON.Vector3} d The direction to search.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The barycenter.
     */
    _getFarthestPointInDirection: function(vertices, d) {
        var Dot = vertices[0].constructor.Dot;
        var maxProduct = Dot(vertices[0], d);
        var index = 0;
        for (var i = 1; i < vertices.length; i++) {
            var product = Dot(vertices[i], d);
            if (product > maxProduct) {
                maxProduct = product;
                index = i;
            }
        }
        return vertices[index];
    },

    /**
     * Gets the nearest edge of the simplex.
     *
     * @method _getNearestEdge
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplex The simplex.
     * @return {Object} Informations about the nearest edge (distance, index and normal).
     */
    _getNearestEdge: function(simplex) {
        var distance = Infinity,
            index, normal;

        for (var i = 0; i < simplex.length; i++) {
            var j = (i + 1) % simplex.length;
            var v1 = simplex[i];
            var v2 = simplex[j];

            var edge = v2.subtract(v1);
            if (edge.lengthSquared() === 0) {
                continue;
            }

            var originTov1 = v1;

            var n = this._getNormal(edge, originTov1, edge);

            if (n.lengthSquared() === 0) {
                n.y = -edge.x;
                n.x = edge.y;
            }

            //n = n.scale(1 / n.length()); //normalize
            var dist = Math.abs(n.constructor.Dot(n, v1)); //distance from origin to edge

            if (dist < distance) {
                distance = dist;
                index = j;
                normal = n;
            }
        }

        return {
            distance: distance,
            index: index,
            normal: normal
        };

    },

    /**
     * Gets the nearest Triangle of the polytope.
     *
     * @method _getNearestTriangle
     * @private
     * @param {BABYLON.Triangle[]} polytope The polytope.
     * @return {Object} Informations about the nearest edge (distance and index).
     */
    _getNearestTriangle: function(polytope) {
        var distance = Infinity,
            index;

        for (var i = 0; i < polytope.length; i++) {

            var triangle = polytope[i];
            var dist = Math.abs(triangle.n.constructor.Dot(triangle.n, triangle.a));

            if (dist < distance) {
                distance = dist;
                index = i;
            }

        }

        return {
            distance: distance,
            index: index
        };
    },

    /**
     * Checks if the origin is in a line.
     *
     * @method _containsLine
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplex The simplex.
     * @param {BABYLON.Vector2|BABYLON.Vector3} dir The direction.
     * @return {Boolean} False in any case because the algorithm just begin.
     */
    _containsLine: function(simplex, dir) {
        var a = simplex[1];
        var b = simplex[0];
        var ab = b.subtract(a);
        var ao = a.scale(-1);
        if (ab.lengthSquared() !== 0) {
            dir.copyFrom(this._getNormal(ab, ao, ab));
        } else {
            dir.copyFrom(ao);
        }

        return false;
    },

    /**
     * Checks if the origin is in a triangle.
     *
     * @method _containsTriangle
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplex The simplex.
     * @param {BABYLON.Vector2|BABYLON.Vector3} dir The direction.
     * @return {Boolean} If in 2D case, may return true if the origin is in the triangle.
     */
    _containsTriangle: function(simplex, dir) {
        var a = simplex[2];
        var Dot = simplex[2].constructor.Dot;
        var b = simplex[1];
        var c = simplex[0];
        var ab = b.subtract(a);
        var ac = c.subtract(a);
        var ao = a.scale(-1);

        var abp = this._getNormal(ac, ab, ab);
        var acp = this._getNormal(ab, ac, ac);
        if (Dot(abp, ao) > 0) {
            simplex.splice(0, 1); // remove C
            dir.copyFrom(abp);
        } else if (Dot(acp, ao) > 0) {
            simplex.splice(1, 1); // remove B
            dir.copyFrom(acp);
        } else {
            if (dir.z === undefined) {
                return true;
            } else {
                var abc = simplex[2].constructor.Cross(ab, ac);
                dir.copyFrom(abc);
                if (Dot(abc, ao) <= 0) {
                    //upside down tetrahedron
                    simplex[0] = b;
                    simplex[1] = c;
                    simplex[2] = a;
                    dir.copyFrom(abc.scale(-1));
                }

                return false;
            }
        }
    },

    /**
     * Checks if the origin is in a tetrahedron.
     *
     * @method _containsTetrahedron
     * @private
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplex The simplex.
     * @param {BABYLON.Vector2|BABYLON.Vector3} dir The direction.
     * @return {Boolean} Return true if the origin is in the tetrahedron.
     */
    _containsTetrahedron: function(simplex, dir) {
        var a = simplex[3];
        var Dot = a.constructor.Dot;
        var Cross = a.constructor.Cross;
        var b = simplex[2];
        var c = simplex[1];
        var d = simplex[0];
        var ab = b.subtract(a);
        var ac = c.subtract(a);
        var ad = d.subtract(a);
        var ao = a.scale(-1);

        var abc = Cross(ab, ac);
        var acd = Cross(ac, ad);
        var adb = Cross(ad, ab);

        var abcTest = 0x1,
            acdTest = 0x2,
            adbTest = 0x4;

        var planeTests = (Dot(abc, ao) > 0 ? abcTest : 0) |
            (Dot(acd, ao) > 0 ? acdTest : 0) |
            (Dot(adb, ao) > 0 ? adbTest : 0);

        switch (planeTests) {
            case abcTest:
                return this._checkTetrahedron(ao, ab, ac, abc, dir, simplex);
            case acdTest:
                simplex[2] = c;
                simplex[1] = d;
                return this._checkTetrahedron(ao, ac, ad, acd, dir, simplex);
            case adbTest:
                //in front of triangle ADB
                simplex[1] = b;
                simplex[2] = d;
                return this._checkTetrahedron(ao, ad, ab, adb, dir, simplex);
            case abcTest | acdTest:
                return this._checkTwoTetrahedron(ao, ab, ac, abc, dir, simplex);
            case acdTest | adbTest:
                simplex[2] = c;
                simplex[1] = d;
                simplex[0] = b;
                return this._checkTwoTetrahedron(ao, ac, ad, acd, dir, simplex);
            case adbTest | abcTest:
                simplex[1] = b;
                simplex[2] = d;
                simplex[0] = c;
                return this._checkTwoTetrahedron(ao, ad, ab, adb, dir, simplex);
            default:
                break;
        }

        //origin in tetrahedron
        return true;
    },

    /**
     * @method _checkTwoTetrahedron
     * @private
     */
    _checkTwoTetrahedron: function(ao, ab, ac, abc, dir, simplex) {
        var abc_ac = abc.constructor.Cross(abc, ac);

        if (abc_ac.constructor.Dot(abc_ac, ao) > 0) {
            //the origin is beyond AC from ABC's
            //perspective, effectively excluding
            //ACD from consideration

            //we thus need test only ACD
            simplex[2] = simplex[1];
            simplex[1] = simplex[0];

            ab = simplex[2].subtract(simplex[3]);
            ac = simplex[1].subtract(simplex[3]);
            abc = ab.constructor.Cross(ab, ac);

            return this._checkTetrahedron(ao, ab, ac, abc, dir, simplex);
        }

        var ab_abc = abc.constructor.Cross(ab, abc);

        if (ab_abc.constructor.Dot(ab_abc, ao) > 0) {

            simplex.splice(0, 2);
            //dir is not ab_abc because it's not point towards the origin;
            //ABxA0xAB direction we are looking for
            dir.copyFrom(this._getNormal(ab, ao, ab));

            return false;
        }

    },

    /**
     * @method _checkTetrahedron
     * @private
     */
    _checkTetrahedron: function(ao, ab, ac, abc, dir, simplex) {

        var acp = abc.constructor.Cross(abc, ac);

        if (acp.constructor.Dot(acp, ao) > 0) {

            simplex[2] = simplex[3];
            simplex.splice(3, 1);
            simplex.splice(0, 1);
            //dir is not abc_ac because it's not point towards the origin;
            //ACxA0xAC direction we are looking for
            dir.copyFrom(this._getNormal(ac, ao, ac));

            return false;
        }

        //almost the same like triangle checks
        var ab_abc = ab.constructor.Cross(ab, abc);

        if (ab_abc.constructor.Dot(ab_abc, ao) > 0) {

            simplex.splice(0, 2);
            //dir is not ab_abc because it's not point towards the origin;
            //ABxA0xAB direction we are looking for
            dir.copyFrom(this._getNormal(ab, ao, ab));

            return false;
        }

        //build new tetrahedron with new base
        simplex.splice(0, 1);

        dir.copyFrom(abc);

        return false;
    },

    /**
     * Adds adge to the list and checks if the edge if not in.
     *
     * @method _addEdge
     * @private
     * @param {Object[]} edges The edges.
     * @param {Object} dir The edge to check.
     */
    _addEdge: function(edges, edge) {
        for (var j = 0; j < edges.length; j++) {
            if (edges[j].a === edge.b && edges[j].b === edge.a) {
                edges.splice(j, 1);
                return;
            }
        }
        edges.push(edge);
    },

    /**
     * The support function.
     *
     * @method support
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @param {BABYLON.Vector2|BABYLON.Vector3} direction The direction.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The support points.
     */
    support: function(colliderPoints, collidedPoints, direction) {
        // d is a vector direction (doesn't have to be normalized)
        // get points on the edge of the shapes in opposite directions
        direction.normalize();
        var p1 = this._getFarthestPointInDirection(colliderPoints, direction);
        var p2 = this._getFarthestPointInDirection(collidedPoints, direction.scale(-1));
        // perform the Minkowski Difference
        var p3 = p1.subtract(p2);
        // p3 is now a point in Minkowski space on the edge of the Minkowski Difference
        return p3;
    },

    /**
     * Checks if the simplex contains the origin.
     *
     * @method findResponseWithEdge
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplexThe simplex or false if no intersection.
     * @param {BABYLON.Vector2|BABYLON.Vector3} dir The direction to test.
     * @return {Boolean} Contains or not.
     */
    containsOrigin: function(simplex, dir) {
        if (simplex.length === 2) {
            return this._containsLine(simplex, dir);
        }
        if (simplex.length === 3) {
            return this._containsTriangle(simplex, dir);
        }
        if (simplex.length === 4) {
            return this._containsTetrahedron(simplex, dir);
        }
        return false;
    },

    /**
     * The GJK (Gilbert–Johnson–Keerthi) algorithm.
     * Computes support points to build the Minkowsky difference and
     * create a simplex. The points of the collider and the collided object
     * must be convexe.
     *
     * @method findResponseWithEdge
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @return {BABYLON.Vector2|BABYLON.Vector3[]} The simplex or false if no intersection.
     */
    check: function(colliderPoints, collidedPoints) {
        var it = 0;
        var a, simplex = [];

        var colliderCenter = this._getBarycenter(colliderPoints);
        var collidedCenter = this._getBarycenter(collidedPoints);

        // initial direction from the center of 1st body to the center of 2nd body
        var dir = colliderCenter.subtract(collidedCenter).normalize();

        // if initial direction is zero – set it to any arbitrary axis (we choose X)
        if (dir.lengthSquared() < this.EPSILON) {
            dir.x = 1;
        }

        // set the first support as initial point of the new simplex
        a = simplex[0] = this.support(colliderPoints, collidedPoints, dir);

        if (a.constructor.Dot(a, dir) <= 0) {
            return false;
        }

        dir.scaleInPlace(-1, dir); // we will be searching in the opposite direction next

        var max = collidedPoints.length * colliderPoints.length;
        while (it < max) {
            if (dir.lengthSquared() === 0 && simplex.length >= 2) {
                // Get perpendicular direction to last simplex
                dir = simplex[simplex.length - 1].subtract(simplex[simplex.length - 2]);
                var tmp = dir.y;
                dir.y = -dir.x;
                dir.x = tmp;
            }

            a = this.support(colliderPoints, collidedPoints, dir);

            // make sure that the last point we added actually passed the origin
            if (a.constructor.Dot(a, dir) <= 0) {
                // if the point added last was not past the origin in the direction of d
                // then the Minkowski Sum cannot possibly contain the origin since
                // the last point added is on the edge of the Minkowski Difference
                return false;
            }

            simplex.push(a);
            // otherwise we need to determine if the origin is in
            // the current simplex

            if (this.containsOrigin(simplex, dir)) {
                return simplex;
            }
            it++;
        }
    },

    /**
     * Finds the response with the simplex (edges) of the gjk algorithm.
     *
     * @method findResponseWithEdge
     * @param {BABYLON.Vector2[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2[]} collidedPoints The convexe collided object.
     * @param {BABYLON.Vector2[]} simplex The simplex.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    findResponseWithEdge: function(colliderPoints, collidedPoints, simplex) {
        var edge = this._getNearestEdge(simplex);
        var sup = this.support(colliderPoints, collidedPoints, edge.normal); //get support point in direction of edge's normal
        var d = Math.abs(sup.constructor.Dot(sup, edge.normal));

        if (d - edge.distance <= this.EPSILON) {
            return edge.normal.scale(edge.distance);
        } else {
            simplex.splice(edge.index, 0, sup);
        }
        return false;
    },

    /**
     * Finds the response with the polytope done with the simplex of the gjk algorithm.
     *
     * @method findResponseWithTriangle
     * @param {BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @param {Triangle[]} polytope The polytope done with the simplex.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    findResponseWithTriangle: function(colliderPoints, collidedPoints, polytope) {

        if (polytope.length === 0) {
            return false;
        }

        var nearest = this._getNearestTriangle(polytope);
        var triangle = polytope[nearest.index];

        var sup = this.support(colliderPoints, collidedPoints, triangle.n);

        var d = Math.abs(sup.constructor.Dot(sup, triangle.n));

        if ((d - nearest.distance <= this.EPSILON)) {
            return triangle.n.scale(nearest.distance);
        } else {
            var edges = [];
            for (var i = polytope.length - 1; i >= 0; i--) {
                triangle = polytope[i];
                // can this face be 'seen' by entry_cur_support?
                if (triangle.n.constructor.Dot(triangle.n, sup.subtract(polytope[i].a)) > 0) {
                    this._addEdge(edges, {
                        a: triangle.a,
                        b: triangle.b
                    });
                    this._addEdge(edges, {
                        a: triangle.b,
                        b: triangle.c
                    });
                    this._addEdge(edges, {
                        a: triangle.c,
                        b: triangle.a
                    });
                    polytope.splice(i, 1);
                }
            }

            // create new triangles from the edges in the edge list
            for (var i = 0; i < edges.length; i++) {
                triangle = new Triangle(sup, edges[i].a, edges[i].b);
                if (triangle.n.length() !== 0) {
                    polytope.push(triangle);
                }
            }
        }

        return false;
    },

    /**
     * Gets the response of the penetration vector with the simplex.
     *
     * @method getResponse
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} simplex The simplex of the Minkowsky difference.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    getResponse: function(colliderPoints, collidedPoints, simplex) {
        var it = 0,
            response;
        var polytope = simplex[0].z !== undefined ? [new Triangle(simplex[0], simplex[1], simplex[2]),
            new Triangle(simplex[0], simplex[2], simplex[3]),
            new Triangle(simplex[0], simplex[3], simplex[1]),
            new Triangle(simplex[1], simplex[3], simplex[2])
        ] : null;

        var max = collidedPoints.length * colliderPoints.length;
        while (it < max) {
            if (simplex[0].z === undefined) {
                response = this.findResponseWithEdge(colliderPoints, collidedPoints, simplex);
            } else {
                response = this.findResponseWithTriangle(colliderPoints, collidedPoints, polytope);
            }
            if (response) {
                var norm = response.clone().normalize().scaleInPlace(this.EPSILON);
                return response.addToRef(norm, response);
            }
            it++;
        }
        return false;
    },

    /**
     * Checks if the collider and the collided object are intersecting.
     *
     * @method isIntersecting
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @return {Boolean} Is intersecting or not.
     */
    isIntersecting: function(colliderPoints, collidedPoints) {
        return !!this.check(colliderPoints, collidedPoints);
    },

    /**
     * Checks if the collider and the collided object are intersecting
     * and give the response to be out of the object.
     *
     * @method intersect
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    intersect: function(colliderPoints, collidedPoints) {
        var simplex = this.check(colliderPoints, collidedPoints);

        //this.cube = this.cube || [];
        if (simplex) {
            return this.getResponse(colliderPoints, collidedPoints, simplex);
        }

        return false;
    }
};

module.exports = CollisionGjkEpa;

},{}]},{},[1])(1)
});
//# sourceMappingURL=data:application/json;charset:utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9ncnVudC1icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJjb2xsaXNpb24tZ2prLWVwYS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBIiwiZmlsZSI6ImdlbmVyYXRlZC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzQ29udGVudCI6WyIoZnVuY3Rpb24gZSh0LG4scil7ZnVuY3Rpb24gcyhvLHUpe2lmKCFuW29dKXtpZighdFtvXSl7dmFyIGE9dHlwZW9mIHJlcXVpcmU9PVwiZnVuY3Rpb25cIiYmcmVxdWlyZTtpZighdSYmYSlyZXR1cm4gYShvLCEwKTtpZihpKXJldHVybiBpKG8sITApO3ZhciBmPW5ldyBFcnJvcihcIkNhbm5vdCBmaW5kIG1vZHVsZSAnXCIrbytcIidcIik7dGhyb3cgZi5jb2RlPVwiTU9EVUxFX05PVF9GT1VORFwiLGZ9dmFyIGw9bltvXT17ZXhwb3J0czp7fX07dFtvXVswXS5jYWxsKGwuZXhwb3J0cyxmdW5jdGlvbihlKXt2YXIgbj10W29dWzFdW2VdO3JldHVybiBzKG4/bjplKX0sbCxsLmV4cG9ydHMsZSx0LG4scil9cmV0dXJuIG5bb10uZXhwb3J0c312YXIgaT10eXBlb2YgcmVxdWlyZT09XCJmdW5jdGlvblwiJiZyZXF1aXJlO2Zvcih2YXIgbz0wO288ci5sZW5ndGg7bysrKXMocltvXSk7cmV0dXJuIHN9KSIsIid1c2Ugc3RyaWN0JztcclxuXHJcbi8qKlxyXG4gKiBDbGFzcyB0byBoZWxwIGluIHRoZSBjb2xsaXNpb24gaW4gMkQgYW5kIDNELlxyXG4gKiBUbyB3b3JrcyB0aGUgYWxnb3JpdGhtIG5lZWRzIHR3byBjb252ZXhlIHBvaW50IGNsb3VkXHJcbiAqIGxpa2UgYm91bmRpbmcgYm94IG9yIHNtdGhnIGxpa2UgdGhpcy5cclxuICogVGhlIGZ1bmN0aW9uIGZ1bmN0aW9ucyBpbnRlcnNlY3QgYW5kIGlzSW50ZXJzZWN0aW5nXHJcbiAqIGhlbHBzIHRvIGhhdmUgaW5mb3JtYXRpb25zIGFib3V0IHRoZSBjb2xsaXNpb24gYmV0d2VlbiB0d28gb2JqZWN0LlxyXG4gKlxyXG4gKiBAY2xhc3Mgd25wLmhlbHBlcnMuQ29sbGlzaW9uR2prRXBhXHJcbiAqIEBjb25zdHJ1Y3RvclxyXG4gKi9cclxudmFyIFRyaWFuZ2xlID0gZnVuY3Rpb24oYSwgYiwgYykge1xyXG4gICAgICB0aGlzLmEgPSBhO1xyXG4gICAgICB0aGlzLmIgPSBiO1xyXG4gICAgICB0aGlzLmMgPSBjO1xyXG4gICAgICB0aGlzLm4gPSBhLmNvbnN0cnVjdG9yLkNyb3NzKGIuc3VidHJhY3QoYSksIGMuc3VidHJhY3QoYSkpLm5vcm1hbGl6ZSgpO1xyXG4gIH07XHJcblxyXG5cclxudmFyIENvbGxpc2lvbkdqa0VwYSA9IHtcclxuXHJcbiAgICBFUFNJTE9OOiAwLjAwMDAwMSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIE1ldGhvZCB0byBnZXQgYSBub3JtYWwgb2YgMyBwb2ludHMgaW4gMkQgYW5kIDNELlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgX2dldE5vcm1hbFxyXG4gICAgICogQHByaXZhdGVcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gYVxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBiXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGNcclxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBub3JtYWwuXHJcbiAgICAgKi9cclxuICAgIF9nZXROb3JtYWw6IGZ1bmN0aW9uKGEsIGIsIGMpIHtcclxuICAgICAgICB2YXIgRG90ID0gYS5jb25zdHJ1Y3Rvci5Eb3Q7XHJcbiAgICAgICAgdmFyIGFjID0gRG90KGEsIGMpOyAvLyBwZXJmb3JtIGFEb3QoYylcclxuICAgICAgICB2YXIgYmMgPSBEb3QoYiwgYyk7IC8vIHBlcmZvcm0gYkRvdChjKVxyXG5cclxuICAgICAgICAvLyBwZXJmb3JtIGIgKiBhLkRvdChjKSAtIGEgKiBiLkRvdChjKVxyXG4gICAgICAgIHZhciByID0gYi5zY2FsZShhYykuc3VidHJhY3QoYS5zY2FsZShiYykpLm5vcm1hbGl6ZSgpO1xyXG4gICAgICAgIHJldHVybiByO1xyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIEdldHMgdGhlIGJhcnljZW50ZXIgb2YgYSBjbG91ZCBwb2ludHMuXHJcbiAgICAgKlxyXG4gICAgICogQG1ldGhvZCBfZ2V0QmFyeWNlbnRlclxyXG4gICAgICogQHByaXZhdGVcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSB2ZXJ0aWNlcyB0aGUgY2xvdWQgcG9pbnRzXHJcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgYmFyeWNlbnRlci5cclxuICAgICAqL1xyXG4gICAgX2dldEJhcnljZW50ZXI6IGZ1bmN0aW9uKHZlcnRpY2VzKSB7XHJcbiAgICAgICAgdmFyIGF2ZyA9IHZlcnRpY2VzWzBdLmNvbnN0cnVjdG9yLlplcm8oKTtcclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHZlcnRpY2VzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgICAgIGF2Zy5hZGRUb1JlZih2ZXJ0aWNlc1tpXSwgYXZnKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgYXZnLnNjYWxlSW5QbGFjZSgxIC8gdmVydGljZXMubGVuZ3RoLCBhdmcpO1xyXG4gICAgICAgIHJldHVybiBhdmc7XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogR2V0cyB0aGUgZmFydGhlc3QgcG9pbnQgb2YgYSBjbG91ZCBwb2ludHMuXHJcbiAgICAgKlxyXG4gICAgICogQG1ldGhvZCBfZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uXHJcbiAgICAgKiBAcHJpdmF0ZVxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHZlcnRpY2VzIHRoZSBjbG91ZCBwb2ludHMuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGQgVGhlIGRpcmVjdGlvbiB0byBzZWFyY2guXHJcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgYmFyeWNlbnRlci5cclxuICAgICAqL1xyXG4gICAgX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbjogZnVuY3Rpb24odmVydGljZXMsIGQpIHtcclxuICAgICAgICB2YXIgRG90ID0gdmVydGljZXNbMF0uY29uc3RydWN0b3IuRG90O1xyXG4gICAgICAgIHZhciBtYXhQcm9kdWN0ID0gRG90KHZlcnRpY2VzWzBdLCBkKTtcclxuICAgICAgICB2YXIgaW5kZXggPSAwO1xyXG4gICAgICAgIGZvciAodmFyIGkgPSAxOyBpIDwgdmVydGljZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICAgICAgdmFyIHByb2R1Y3QgPSBEb3QodmVydGljZXNbaV0sIGQpO1xyXG4gICAgICAgICAgICBpZiAocHJvZHVjdCA+IG1heFByb2R1Y3QpIHtcclxuICAgICAgICAgICAgICAgIG1heFByb2R1Y3QgPSBwcm9kdWN0O1xyXG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHJldHVybiB2ZXJ0aWNlc1tpbmRleF07XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogR2V0cyB0aGUgbmVhcmVzdCBlZGdlIG9mIHRoZSBzaW1wbGV4LlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RFZGdlXHJcbiAgICAgKiBAcHJpdmF0ZVxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXHJcbiAgICAgKiBAcmV0dXJuIHtPYmplY3R9IEluZm9ybWF0aW9ucyBhYm91dCB0aGUgbmVhcmVzdCBlZGdlIChkaXN0YW5jZSwgaW5kZXggYW5kIG5vcm1hbCkuXHJcbiAgICAgKi9cclxuICAgIF9nZXROZWFyZXN0RWRnZTogZnVuY3Rpb24oc2ltcGxleCkge1xyXG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxyXG4gICAgICAgICAgICBpbmRleCwgbm9ybWFsO1xyXG5cclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHNpbXBsZXgubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICAgICAgdmFyIGogPSAoaSArIDEpICUgc2ltcGxleC5sZW5ndGg7XHJcbiAgICAgICAgICAgIHZhciB2MSA9IHNpbXBsZXhbaV07XHJcbiAgICAgICAgICAgIHZhciB2MiA9IHNpbXBsZXhbal07XHJcblxyXG4gICAgICAgICAgICB2YXIgZWRnZSA9IHYyLnN1YnRyYWN0KHYxKTtcclxuICAgICAgICAgICAgaWYgKGVkZ2UubGVuZ3RoU3F1YXJlZCgpID09PSAwKSB7XHJcbiAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgdmFyIG9yaWdpblRvdjEgPSB2MTtcclxuXHJcbiAgICAgICAgICAgIHZhciBuID0gdGhpcy5fZ2V0Tm9ybWFsKGVkZ2UsIG9yaWdpblRvdjEsIGVkZ2UpO1xyXG5cclxuICAgICAgICAgICAgaWYgKG4ubGVuZ3RoU3F1YXJlZCgpID09PSAwKSB7XHJcbiAgICAgICAgICAgICAgICBuLnkgPSAtZWRnZS54O1xyXG4gICAgICAgICAgICAgICAgbi54ID0gZWRnZS55O1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAvL24gPSBuLnNjYWxlKDEgLyBuLmxlbmd0aCgpKTsgLy9ub3JtYWxpemVcclxuICAgICAgICAgICAgdmFyIGRpc3QgPSBNYXRoLmFicyhuLmNvbnN0cnVjdG9yLkRvdChuLCB2MSkpOyAvL2Rpc3RhbmNlIGZyb20gb3JpZ2luIHRvIGVkZ2VcclxuXHJcbiAgICAgICAgICAgIGlmIChkaXN0IDwgZGlzdGFuY2UpIHtcclxuICAgICAgICAgICAgICAgIGRpc3RhbmNlID0gZGlzdDtcclxuICAgICAgICAgICAgICAgIGluZGV4ID0gajtcclxuICAgICAgICAgICAgICAgIG5vcm1hbCA9IG47XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHJldHVybiB7XHJcbiAgICAgICAgICAgIGRpc3RhbmNlOiBkaXN0YW5jZSxcclxuICAgICAgICAgICAgaW5kZXg6IGluZGV4LFxyXG4gICAgICAgICAgICBub3JtYWw6IG5vcm1hbFxyXG4gICAgICAgIH07XHJcblxyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIEdldHMgdGhlIG5lYXJlc3QgVHJpYW5nbGUgb2YgdGhlIHBvbHl0b3BlLlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RUcmlhbmdsZVxyXG4gICAgICogQHByaXZhdGVcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5UcmlhbmdsZVtdfSBwb2x5dG9wZSBUaGUgcG9seXRvcGUuXHJcbiAgICAgKiBAcmV0dXJuIHtPYmplY3R9IEluZm9ybWF0aW9ucyBhYm91dCB0aGUgbmVhcmVzdCBlZGdlIChkaXN0YW5jZSBhbmQgaW5kZXgpLlxyXG4gICAgICovXHJcbiAgICBfZ2V0TmVhcmVzdFRyaWFuZ2xlOiBmdW5jdGlvbihwb2x5dG9wZSkge1xyXG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxyXG4gICAgICAgICAgICBpbmRleDtcclxuXHJcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwb2x5dG9wZS5sZW5ndGg7IGkrKykge1xyXG5cclxuICAgICAgICAgICAgdmFyIHRyaWFuZ2xlID0gcG9seXRvcGVbaV07XHJcbiAgICAgICAgICAgIHZhciBkaXN0ID0gTWF0aC5hYnModHJpYW5nbGUubi5jb25zdHJ1Y3Rvci5Eb3QodHJpYW5nbGUubiwgdHJpYW5nbGUuYSkpO1xyXG5cclxuICAgICAgICAgICAgaWYgKGRpc3QgPCBkaXN0YW5jZSkge1xyXG4gICAgICAgICAgICAgICAgZGlzdGFuY2UgPSBkaXN0O1xyXG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcmV0dXJuIHtcclxuICAgICAgICAgICAgZGlzdGFuY2U6IGRpc3RhbmNlLFxyXG4gICAgICAgICAgICBpbmRleDogaW5kZXhcclxuICAgICAgICB9O1xyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgbGluZS5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc0xpbmVcclxuICAgICAqIEBwcml2YXRlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24uXHJcbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBGYWxzZSBpbiBhbnkgY2FzZSBiZWNhdXNlIHRoZSBhbGdvcml0aG0ganVzdCBiZWdpbi5cclxuICAgICAqL1xyXG4gICAgX2NvbnRhaW5zTGluZTogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XHJcbiAgICAgICAgdmFyIGEgPSBzaW1wbGV4WzFdO1xyXG4gICAgICAgIHZhciBiID0gc2ltcGxleFswXTtcclxuICAgICAgICB2YXIgYWIgPSBiLnN1YnRyYWN0KGEpO1xyXG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xyXG4gICAgICAgIGlmIChhYi5sZW5ndGhTcXVhcmVkKCkgIT09IDApIHtcclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYiwgYW8sIGFiKSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFvKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH0sXHJcblxyXG4gICAgLyoqXHJcbiAgICAgKiBDaGVja3MgaWYgdGhlIG9yaWdpbiBpcyBpbiBhIHRyaWFuZ2xlLlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgX2NvbnRhaW5zVHJpYW5nbGVcclxuICAgICAqIEBwcml2YXRlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24uXHJcbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBJZiBpbiAyRCBjYXNlLCBtYXkgcmV0dXJuIHRydWUgaWYgdGhlIG9yaWdpbiBpcyBpbiB0aGUgdHJpYW5nbGUuXHJcbiAgICAgKi9cclxuICAgIF9jb250YWluc1RyaWFuZ2xlOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcclxuICAgICAgICB2YXIgYSA9IHNpbXBsZXhbMl07XHJcbiAgICAgICAgdmFyIERvdCA9IHNpbXBsZXhbMl0uY29uc3RydWN0b3IuRG90O1xyXG4gICAgICAgIHZhciBiID0gc2ltcGxleFsxXTtcclxuICAgICAgICB2YXIgYyA9IHNpbXBsZXhbMF07XHJcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcclxuICAgICAgICB2YXIgYWMgPSBjLnN1YnRyYWN0KGEpO1xyXG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xyXG5cclxuICAgICAgICB2YXIgYWJwID0gdGhpcy5fZ2V0Tm9ybWFsKGFjLCBhYiwgYWIpO1xyXG4gICAgICAgIHZhciBhY3AgPSB0aGlzLl9nZXROb3JtYWwoYWIsIGFjLCBhYyk7XHJcbiAgICAgICAgaWYgKERvdChhYnAsIGFvKSA+IDApIHtcclxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7IC8vIHJlbW92ZSBDXHJcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYnApO1xyXG4gICAgICAgIH0gZWxzZSBpZiAoRG90KGFjcCwgYW8pID4gMCkge1xyXG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgxLCAxKTsgLy8gcmVtb3ZlIEJcclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFjcCk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgaWYgKGRpci56ID09PSB1bmRlZmluZWQpIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICAgICAgdmFyIGFiYyA9IHNpbXBsZXhbMl0uY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFjKTtcclxuICAgICAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYmMpO1xyXG4gICAgICAgICAgICAgICAgaWYgKERvdChhYmMsIGFvKSA8PSAwKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgLy91cHNpZGUgZG93biB0ZXRyYWhlZHJvblxyXG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBiO1xyXG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBjO1xyXG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBhO1xyXG4gICAgICAgICAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYmMuc2NhbGUoLTEpKTtcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogQ2hlY2tzIGlmIHRoZSBvcmlnaW4gaXMgaW4gYSB0ZXRyYWhlZHJvbi5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc1RldHJhaGVkcm9uXHJcbiAgICAgKiBAcHJpdmF0ZVxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uLlxyXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gUmV0dXJuIHRydWUgaWYgdGhlIG9yaWdpbiBpcyBpbiB0aGUgdGV0cmFoZWRyb24uXHJcbiAgICAgKi9cclxuICAgIF9jb250YWluc1RldHJhaGVkcm9uOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcclxuICAgICAgICB2YXIgYSA9IHNpbXBsZXhbM107XHJcbiAgICAgICAgdmFyIERvdCA9IGEuY29uc3RydWN0b3IuRG90O1xyXG4gICAgICAgIHZhciBDcm9zcyA9IGEuY29uc3RydWN0b3IuQ3Jvc3M7XHJcbiAgICAgICAgdmFyIGIgPSBzaW1wbGV4WzJdO1xyXG4gICAgICAgIHZhciBjID0gc2ltcGxleFsxXTtcclxuICAgICAgICB2YXIgZCA9IHNpbXBsZXhbMF07XHJcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcclxuICAgICAgICB2YXIgYWMgPSBjLnN1YnRyYWN0KGEpO1xyXG4gICAgICAgIHZhciBhZCA9IGQuc3VidHJhY3QoYSk7XHJcbiAgICAgICAgdmFyIGFvID0gYS5zY2FsZSgtMSk7XHJcblxyXG4gICAgICAgIHZhciBhYmMgPSBDcm9zcyhhYiwgYWMpO1xyXG4gICAgICAgIHZhciBhY2QgPSBDcm9zcyhhYywgYWQpO1xyXG4gICAgICAgIHZhciBhZGIgPSBDcm9zcyhhZCwgYWIpO1xyXG5cclxuICAgICAgICB2YXIgYWJjVGVzdCA9IDB4MSxcclxuICAgICAgICAgICAgYWNkVGVzdCA9IDB4MixcclxuICAgICAgICAgICAgYWRiVGVzdCA9IDB4NDtcclxuXHJcbiAgICAgICAgdmFyIHBsYW5lVGVzdHMgPSAoRG90KGFiYywgYW8pID4gMCA/IGFiY1Rlc3QgOiAwKSB8XHJcbiAgICAgICAgICAgIChEb3QoYWNkLCBhbykgPiAwID8gYWNkVGVzdCA6IDApIHxcclxuICAgICAgICAgICAgKERvdChhZGIsIGFvKSA+IDAgPyBhZGJUZXN0IDogMCk7XHJcblxyXG4gICAgICAgIHN3aXRjaCAocGxhbmVUZXN0cykge1xyXG4gICAgICAgICAgICBjYXNlIGFiY1Rlc3Q6XHJcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XHJcbiAgICAgICAgICAgIGNhc2UgYWNkVGVzdDpcclxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBjO1xyXG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGQ7XHJcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWMsIGFkLCBhY2QsIGRpciwgc2ltcGxleCk7XHJcbiAgICAgICAgICAgIGNhc2UgYWRiVGVzdDpcclxuICAgICAgICAgICAgICAgIC8vaW4gZnJvbnQgb2YgdHJpYW5nbGUgQURCXHJcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gYjtcclxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBkO1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xyXG4gICAgICAgICAgICBjYXNlIGFiY1Rlc3QgfCBhY2RUZXN0OlxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpO1xyXG4gICAgICAgICAgICBjYXNlIGFjZFRlc3QgfCBhZGJUZXN0OlxyXG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGM7XHJcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gZDtcclxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBiO1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFjLCBhZCwgYWNkLCBkaXIsIHNpbXBsZXgpO1xyXG4gICAgICAgICAgICBjYXNlIGFkYlRlc3QgfCBhYmNUZXN0OlxyXG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGI7XHJcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gZDtcclxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBjO1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xyXG4gICAgICAgICAgICBkZWZhdWx0OlxyXG4gICAgICAgICAgICAgICAgYnJlYWs7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvL29yaWdpbiBpbiB0ZXRyYWhlZHJvblxyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIEBtZXRob2QgX2NoZWNrVHdvVGV0cmFoZWRyb25cclxuICAgICAqIEBwcml2YXRlXHJcbiAgICAgKi9cclxuICAgIF9jaGVja1R3b1RldHJhaGVkcm9uOiBmdW5jdGlvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCkge1xyXG4gICAgICAgIHZhciBhYmNfYWMgPSBhYmMuY29uc3RydWN0b3IuQ3Jvc3MoYWJjLCBhYyk7XHJcblxyXG4gICAgICAgIGlmIChhYmNfYWMuY29uc3RydWN0b3IuRG90KGFiY19hYywgYW8pID4gMCkge1xyXG4gICAgICAgICAgICAvL3RoZSBvcmlnaW4gaXMgYmV5b25kIEFDIGZyb20gQUJDJ3NcclxuICAgICAgICAgICAgLy9wZXJzcGVjdGl2ZSwgZWZmZWN0aXZlbHkgZXhjbHVkaW5nXHJcbiAgICAgICAgICAgIC8vQUNEIGZyb20gY29uc2lkZXJhdGlvblxyXG5cclxuICAgICAgICAgICAgLy93ZSB0aHVzIG5lZWQgdGVzdCBvbmx5IEFDRFxyXG4gICAgICAgICAgICBzaW1wbGV4WzJdID0gc2ltcGxleFsxXTtcclxuICAgICAgICAgICAgc2ltcGxleFsxXSA9IHNpbXBsZXhbMF07XHJcblxyXG4gICAgICAgICAgICBhYiA9IHNpbXBsZXhbMl0uc3VidHJhY3Qoc2ltcGxleFszXSk7XHJcbiAgICAgICAgICAgIGFjID0gc2ltcGxleFsxXS5zdWJ0cmFjdChzaW1wbGV4WzNdKTtcclxuICAgICAgICAgICAgYWJjID0gYWIuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFjKTtcclxuXHJcbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1RldHJhaGVkcm9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBhYl9hYmMgPSBhYmMuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFiYyk7XHJcblxyXG4gICAgICAgIGlmIChhYl9hYmMuY29uc3RydWN0b3IuRG90KGFiX2FiYywgYW8pID4gMCkge1xyXG5cclxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMik7XHJcbiAgICAgICAgICAgIC8vZGlyIGlzIG5vdCBhYl9hYmMgYmVjYXVzZSBpdCdzIG5vdCBwb2ludCB0b3dhcmRzIHRoZSBvcmlnaW47XHJcbiAgICAgICAgICAgIC8vQUJ4QTB4QUIgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxyXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcclxuXHJcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIEBtZXRob2QgX2NoZWNrVGV0cmFoZWRyb25cclxuICAgICAqIEBwcml2YXRlXHJcbiAgICAgKi9cclxuICAgIF9jaGVja1RldHJhaGVkcm9uOiBmdW5jdGlvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCkge1xyXG5cclxuICAgICAgICB2YXIgYWNwID0gYWJjLmNvbnN0cnVjdG9yLkNyb3NzKGFiYywgYWMpO1xyXG5cclxuICAgICAgICBpZiAoYWNwLmNvbnN0cnVjdG9yLkRvdChhY3AsIGFvKSA+IDApIHtcclxuXHJcbiAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBzaW1wbGV4WzNdO1xyXG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgzLCAxKTtcclxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7XHJcbiAgICAgICAgICAgIC8vZGlyIGlzIG5vdCBhYmNfYWMgYmVjYXVzZSBpdCdzIG5vdCBwb2ludCB0b3dhcmRzIHRoZSBvcmlnaW47XHJcbiAgICAgICAgICAgIC8vQUN4QTB4QUMgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxyXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFjLCBhbywgYWMpKTtcclxuXHJcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vYWxtb3N0IHRoZSBzYW1lIGxpa2UgdHJpYW5nbGUgY2hlY2tzXHJcbiAgICAgICAgdmFyIGFiX2FiYyA9IGFiLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYmMpO1xyXG5cclxuICAgICAgICBpZiAoYWJfYWJjLmNvbnN0cnVjdG9yLkRvdChhYl9hYmMsIGFvKSA+IDApIHtcclxuXHJcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDIpO1xyXG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJfYWJjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xyXG4gICAgICAgICAgICAvL0FCeEEweEFCIGRpcmVjdGlvbiB3ZSBhcmUgbG9va2luZyBmb3JcclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYiwgYW8sIGFiKSk7XHJcblxyXG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvL2J1aWxkIG5ldyB0ZXRyYWhlZHJvbiB3aXRoIG5ldyBiYXNlXHJcbiAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7XHJcblxyXG4gICAgICAgIGRpci5jb3B5RnJvbShhYmMpO1xyXG5cclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogQWRkcyBhZGdlIHRvIHRoZSBsaXN0IGFuZCBjaGVja3MgaWYgdGhlIGVkZ2UgaWYgbm90IGluLlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgX2FkZEVkZ2VcclxuICAgICAqIEBwcml2YXRlXHJcbiAgICAgKiBAcGFyYW0ge09iamVjdFtdfSBlZGdlcyBUaGUgZWRnZXMuXHJcbiAgICAgKiBAcGFyYW0ge09iamVjdH0gZGlyIFRoZSBlZGdlIHRvIGNoZWNrLlxyXG4gICAgICovXHJcbiAgICBfYWRkRWRnZTogZnVuY3Rpb24oZWRnZXMsIGVkZ2UpIHtcclxuICAgICAgICBmb3IgKHZhciBqID0gMDsgaiA8IGVkZ2VzLmxlbmd0aDsgaisrKSB7XHJcbiAgICAgICAgICAgIGlmIChlZGdlc1tqXS5hID09PSBlZGdlLmIgJiYgZWRnZXNbal0uYiA9PT0gZWRnZS5hKSB7XHJcbiAgICAgICAgICAgICAgICBlZGdlcy5zcGxpY2UoaiwgMSk7XHJcbiAgICAgICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgZWRnZXMucHVzaChlZGdlKTtcclxuICAgIH0sXHJcblxyXG4gICAgLyoqXHJcbiAgICAgKiBUaGUgc3VwcG9ydCBmdW5jdGlvbi5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIHN1cHBvcnRcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXJlY3Rpb24gVGhlIGRpcmVjdGlvbi5cclxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBzdXBwb3J0IHBvaW50cy5cclxuICAgICAqL1xyXG4gICAgc3VwcG9ydDogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBkaXJlY3Rpb24pIHtcclxuICAgICAgICAvLyBkIGlzIGEgdmVjdG9yIGRpcmVjdGlvbiAoZG9lc24ndCBoYXZlIHRvIGJlIG5vcm1hbGl6ZWQpXHJcbiAgICAgICAgLy8gZ2V0IHBvaW50cyBvbiB0aGUgZWRnZSBvZiB0aGUgc2hhcGVzIGluIG9wcG9zaXRlIGRpcmVjdGlvbnNcclxuICAgICAgICBkaXJlY3Rpb24ubm9ybWFsaXplKCk7XHJcbiAgICAgICAgdmFyIHAxID0gdGhpcy5fZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uKGNvbGxpZGVyUG9pbnRzLCBkaXJlY3Rpb24pO1xyXG4gICAgICAgIHZhciBwMiA9IHRoaXMuX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbihjb2xsaWRlZFBvaW50cywgZGlyZWN0aW9uLnNjYWxlKC0xKSk7XHJcbiAgICAgICAgLy8gcGVyZm9ybSB0aGUgTWlua293c2tpIERpZmZlcmVuY2VcclxuICAgICAgICB2YXIgcDMgPSBwMS5zdWJ0cmFjdChwMik7XHJcbiAgICAgICAgLy8gcDMgaXMgbm93IGEgcG9pbnQgaW4gTWlua293c2tpIHNwYWNlIG9uIHRoZSBlZGdlIG9mIHRoZSBNaW5rb3dza2kgRGlmZmVyZW5jZVxyXG4gICAgICAgIHJldHVybiBwMztcclxuICAgIH0sXHJcblxyXG4gICAgLyoqXHJcbiAgICAgKiBDaGVja3MgaWYgdGhlIHNpbXBsZXggY29udGFpbnMgdGhlIG9yaWdpbi5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleFRoZSBzaW1wbGV4IG9yIGZhbHNlIGlmIG5vIGludGVyc2VjdGlvbi5cclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24gdG8gdGVzdC5cclxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IENvbnRhaW5zIG9yIG5vdC5cclxuICAgICAqL1xyXG4gICAgY29udGFpbnNPcmlnaW46IGZ1bmN0aW9uKHNpbXBsZXgsIGRpcikge1xyXG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMikge1xyXG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY29udGFpbnNMaW5lKHNpbXBsZXgsIGRpcik7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMykge1xyXG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY29udGFpbnNUcmlhbmdsZShzaW1wbGV4LCBkaXIpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBpZiAoc2ltcGxleC5sZW5ndGggPT09IDQpIHtcclxuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zVGV0cmFoZWRyb24oc2ltcGxleCwgZGlyKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIFRoZSBHSksgKEdpbGJlcnTigJNKb2huc29u4oCTS2VlcnRoaSkgYWxnb3JpdGhtLlxyXG4gICAgICogQ29tcHV0ZXMgc3VwcG9ydCBwb2ludHMgdG8gYnVpbGQgdGhlIE1pbmtvd3NreSBkaWZmZXJlbmNlIGFuZFxyXG4gICAgICogY3JlYXRlIGEgc2ltcGxleC4gVGhlIHBvaW50cyBvZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3RcclxuICAgICAqIG11c3QgYmUgY29udmV4ZS5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cclxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gVGhlIHNpbXBsZXggb3IgZmFsc2UgaWYgbm8gaW50ZXJzZWN0aW9uLlxyXG4gICAgICovXHJcbiAgICBjaGVjazogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKSB7XHJcbiAgICAgICAgdmFyIGl0ID0gMDtcclxuICAgICAgICB2YXIgYSwgc2ltcGxleCA9IFtdO1xyXG5cclxuICAgICAgICB2YXIgY29sbGlkZXJDZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKGNvbGxpZGVyUG9pbnRzKTtcclxuICAgICAgICB2YXIgY29sbGlkZWRDZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKGNvbGxpZGVkUG9pbnRzKTtcclxuXHJcbiAgICAgICAgLy8gaW5pdGlhbCBkaXJlY3Rpb24gZnJvbSB0aGUgY2VudGVyIG9mIDFzdCBib2R5IHRvIHRoZSBjZW50ZXIgb2YgMm5kIGJvZHlcclxuICAgICAgICB2YXIgZGlyID0gY29sbGlkZXJDZW50ZXIuc3VidHJhY3QoY29sbGlkZWRDZW50ZXIpLm5vcm1hbGl6ZSgpO1xyXG5cclxuICAgICAgICAvLyBpZiBpbml0aWFsIGRpcmVjdGlvbiBpcyB6ZXJvIOKAkyBzZXQgaXQgdG8gYW55IGFyYml0cmFyeSBheGlzICh3ZSBjaG9vc2UgWClcclxuICAgICAgICBpZiAoZGlyLmxlbmd0aFNxdWFyZWQoKSA8IHRoaXMuRVBTSUxPTikge1xyXG4gICAgICAgICAgICBkaXIueCA9IDE7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBzZXQgdGhlIGZpcnN0IHN1cHBvcnQgYXMgaW5pdGlhbCBwb2ludCBvZiB0aGUgbmV3IHNpbXBsZXhcclxuICAgICAgICBhID0gc2ltcGxleFswXSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XHJcblxyXG4gICAgICAgIGlmIChhLmNvbnN0cnVjdG9yLkRvdChhLCBkaXIpIDw9IDApIHtcclxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgZGlyLnNjYWxlSW5QbGFjZSgtMSwgZGlyKTsgLy8gd2Ugd2lsbCBiZSBzZWFyY2hpbmcgaW4gdGhlIG9wcG9zaXRlIGRpcmVjdGlvbiBuZXh0XHJcblxyXG4gICAgICAgIHZhciBtYXggPSBjb2xsaWRlZFBvaW50cy5sZW5ndGggKiBjb2xsaWRlclBvaW50cy5sZW5ndGg7XHJcbiAgICAgICAgd2hpbGUgKGl0IDwgbWF4KSB7XHJcbiAgICAgICAgICAgIGlmIChkaXIubGVuZ3RoU3F1YXJlZCgpID09PSAwICYmIHNpbXBsZXgubGVuZ3RoID49IDIpIHtcclxuICAgICAgICAgICAgICAgIC8vIEdldCBwZXJwZW5kaWN1bGFyIGRpcmVjdGlvbiB0byBsYXN0IHNpbXBsZXhcclxuICAgICAgICAgICAgICAgIGRpciA9IHNpbXBsZXhbc2ltcGxleC5sZW5ndGggLSAxXS5zdWJ0cmFjdChzaW1wbGV4W3NpbXBsZXgubGVuZ3RoIC0gMl0pO1xyXG4gICAgICAgICAgICAgICAgdmFyIHRtcCA9IGRpci55O1xyXG4gICAgICAgICAgICAgICAgZGlyLnkgPSAtZGlyLng7XHJcbiAgICAgICAgICAgICAgICBkaXIueCA9IHRtcDtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgYSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XHJcblxyXG4gICAgICAgICAgICAvLyBtYWtlIHN1cmUgdGhhdCB0aGUgbGFzdCBwb2ludCB3ZSBhZGRlZCBhY3R1YWxseSBwYXNzZWQgdGhlIG9yaWdpblxyXG4gICAgICAgICAgICBpZiAoYS5jb25zdHJ1Y3Rvci5Eb3QoYSwgZGlyKSA8PSAwKSB7XHJcbiAgICAgICAgICAgICAgICAvLyBpZiB0aGUgcG9pbnQgYWRkZWQgbGFzdCB3YXMgbm90IHBhc3QgdGhlIG9yaWdpbiBpbiB0aGUgZGlyZWN0aW9uIG9mIGRcclxuICAgICAgICAgICAgICAgIC8vIHRoZW4gdGhlIE1pbmtvd3NraSBTdW0gY2Fubm90IHBvc3NpYmx5IGNvbnRhaW4gdGhlIG9yaWdpbiBzaW5jZVxyXG4gICAgICAgICAgICAgICAgLy8gdGhlIGxhc3QgcG9pbnQgYWRkZWQgaXMgb24gdGhlIGVkZ2Ugb2YgdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIHNpbXBsZXgucHVzaChhKTtcclxuICAgICAgICAgICAgLy8gb3RoZXJ3aXNlIHdlIG5lZWQgdG8gZGV0ZXJtaW5lIGlmIHRoZSBvcmlnaW4gaXMgaW5cclxuICAgICAgICAgICAgLy8gdGhlIGN1cnJlbnQgc2ltcGxleFxyXG5cclxuICAgICAgICAgICAgaWYgKHRoaXMuY29udGFpbnNPcmlnaW4oc2ltcGxleCwgZGlyKSkge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHNpbXBsZXg7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaXQrKztcclxuICAgICAgICB9XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogRmluZHMgdGhlIHJlc3BvbnNlIHdpdGggdGhlIHNpbXBsZXggKGVkZ2VzKSBvZiB0aGUgZ2prIGFsZ29yaXRobS5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxyXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHBlbmV0cmF0aW9uIHZlY3Rvci5cclxuICAgICAqL1xyXG4gICAgZmluZFJlc3BvbnNlV2l0aEVkZ2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCkge1xyXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5fZ2V0TmVhcmVzdEVkZ2Uoc2ltcGxleCk7XHJcbiAgICAgICAgdmFyIHN1cCA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGVkZ2Uubm9ybWFsKTsgLy9nZXQgc3VwcG9ydCBwb2ludCBpbiBkaXJlY3Rpb24gb2YgZWRnZSdzIG5vcm1hbFxyXG4gICAgICAgIHZhciBkID0gTWF0aC5hYnMoc3VwLmNvbnN0cnVjdG9yLkRvdChzdXAsIGVkZ2Uubm9ybWFsKSk7XHJcblxyXG4gICAgICAgIGlmIChkIC0gZWRnZS5kaXN0YW5jZSA8PSB0aGlzLkVQU0lMT04pIHtcclxuICAgICAgICAgICAgcmV0dXJuIGVkZ2Uubm9ybWFsLnNjYWxlKGVkZ2UuZGlzdGFuY2UpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKGVkZ2UuaW5kZXgsIDAsIHN1cCk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH0sXHJcblxyXG4gICAgLyoqXHJcbiAgICAgKiBGaW5kcyB0aGUgcmVzcG9uc2Ugd2l0aCB0aGUgcG9seXRvcGUgZG9uZSB3aXRoIHRoZSBzaW1wbGV4IG9mIHRoZSBnamsgYWxnb3JpdGhtLlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgZmluZFJlc3BvbnNlV2l0aFRyaWFuZ2xlXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge1RyaWFuZ2xlW119IHBvbHl0b3BlIFRoZSBwb2x5dG9wZSBkb25lIHdpdGggdGhlIHNpbXBsZXguXHJcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxyXG4gICAgICovXHJcbiAgICBmaW5kUmVzcG9uc2VXaXRoVHJpYW5nbGU6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgcG9seXRvcGUpIHtcclxuXHJcbiAgICAgICAgaWYgKHBvbHl0b3BlLmxlbmd0aCA9PT0gMCkge1xyXG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICB2YXIgbmVhcmVzdCA9IHRoaXMuX2dldE5lYXJlc3RUcmlhbmdsZShwb2x5dG9wZSk7XHJcbiAgICAgICAgdmFyIHRyaWFuZ2xlID0gcG9seXRvcGVbbmVhcmVzdC5pbmRleF07XHJcblxyXG4gICAgICAgIHZhciBzdXAgPSB0aGlzLnN1cHBvcnQoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCB0cmlhbmdsZS5uKTtcclxuXHJcbiAgICAgICAgdmFyIGQgPSBNYXRoLmFicyhzdXAuY29uc3RydWN0b3IuRG90KHN1cCwgdHJpYW5nbGUubikpO1xyXG5cclxuICAgICAgICBpZiAoKGQgLSBuZWFyZXN0LmRpc3RhbmNlIDw9IHRoaXMuRVBTSUxPTikpIHtcclxuICAgICAgICAgICAgcmV0dXJuIHRyaWFuZ2xlLm4uc2NhbGUobmVhcmVzdC5kaXN0YW5jZSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgdmFyIGVkZ2VzID0gW107XHJcbiAgICAgICAgICAgIGZvciAodmFyIGkgPSBwb2x5dG9wZS5sZW5ndGggLSAxOyBpID49IDA7IGktLSkge1xyXG4gICAgICAgICAgICAgICAgdHJpYW5nbGUgPSBwb2x5dG9wZVtpXTtcclxuICAgICAgICAgICAgICAgIC8vIGNhbiB0aGlzIGZhY2UgYmUgJ3NlZW4nIGJ5IGVudHJ5X2N1cl9zdXBwb3J0P1xyXG4gICAgICAgICAgICAgICAgaWYgKHRyaWFuZ2xlLm4uY29uc3RydWN0b3IuRG90KHRyaWFuZ2xlLm4sIHN1cC5zdWJ0cmFjdChwb2x5dG9wZVtpXS5hKSkgPiAwKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5fYWRkRWRnZShlZGdlcywge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBhOiB0cmlhbmdsZS5hLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICBiOiB0cmlhbmdsZS5iXHJcbiAgICAgICAgICAgICAgICAgICAgfSk7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5fYWRkRWRnZShlZGdlcywge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBhOiB0cmlhbmdsZS5iLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICBiOiB0cmlhbmdsZS5jXHJcbiAgICAgICAgICAgICAgICAgICAgfSk7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5fYWRkRWRnZShlZGdlcywge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBhOiB0cmlhbmdsZS5jLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICBiOiB0cmlhbmdsZS5hXHJcbiAgICAgICAgICAgICAgICAgICAgfSk7XHJcbiAgICAgICAgICAgICAgICAgICAgcG9seXRvcGUuc3BsaWNlKGksIDEpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgbmV3IHRyaWFuZ2xlcyBmcm9tIHRoZSBlZGdlcyBpbiB0aGUgZWRnZSBsaXN0XHJcbiAgICAgICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZWRnZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICAgICAgICAgIHRyaWFuZ2xlID0gbmV3IFRyaWFuZ2xlKHN1cCwgZWRnZXNbaV0uYSwgZWRnZXNbaV0uYik7XHJcbiAgICAgICAgICAgICAgICBpZiAodHJpYW5nbGUubi5sZW5ndGgoKSAhPT0gMCkge1xyXG4gICAgICAgICAgICAgICAgICAgIHBvbHl0b3BlLnB1c2godHJpYW5nbGUpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogR2V0cyB0aGUgcmVzcG9uc2Ugb2YgdGhlIHBlbmV0cmF0aW9uIHZlY3RvciB3aXRoIHRoZSBzaW1wbGV4LlxyXG4gICAgICpcclxuICAgICAqIEBtZXRob2QgZ2V0UmVzcG9uc2VcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXggb2YgdGhlIE1pbmtvd3NreSBkaWZmZXJlbmNlLlxyXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHBlbmV0cmF0aW9uIHZlY3Rvci5cclxuICAgICAqL1xyXG4gICAgZ2V0UmVzcG9uc2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCkge1xyXG4gICAgICAgIHZhciBpdCA9IDAsXHJcbiAgICAgICAgICAgIHJlc3BvbnNlO1xyXG4gICAgICAgIHZhciBwb2x5dG9wZSA9IHNpbXBsZXhbMF0ueiAhPT0gdW5kZWZpbmVkID8gW25ldyBUcmlhbmdsZShzaW1wbGV4WzBdLCBzaW1wbGV4WzFdLCBzaW1wbGV4WzJdKSxcclxuICAgICAgICAgICAgbmV3IFRyaWFuZ2xlKHNpbXBsZXhbMF0sIHNpbXBsZXhbMl0sIHNpbXBsZXhbM10pLFxyXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFszXSwgc2ltcGxleFsxXSksXHJcbiAgICAgICAgICAgIG5ldyBUcmlhbmdsZShzaW1wbGV4WzFdLCBzaW1wbGV4WzNdLCBzaW1wbGV4WzJdKVxyXG4gICAgICAgIF0gOiBudWxsO1xyXG5cclxuICAgICAgICB2YXIgbWF4ID0gY29sbGlkZWRQb2ludHMubGVuZ3RoICogY29sbGlkZXJQb2ludHMubGVuZ3RoO1xyXG4gICAgICAgIHdoaWxlIChpdCA8IG1heCkge1xyXG4gICAgICAgICAgICBpZiAoc2ltcGxleFswXS56ID09PSB1bmRlZmluZWQpIHtcclxuICAgICAgICAgICAgICAgIHJlc3BvbnNlID0gdGhpcy5maW5kUmVzcG9uc2VXaXRoRWRnZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgpO1xyXG4gICAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHBvbHl0b3BlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBpZiAocmVzcG9uc2UpIHtcclxuICAgICAgICAgICAgICAgIHZhciBub3JtID0gcmVzcG9uc2UuY2xvbmUoKS5ub3JtYWxpemUoKS5zY2FsZUluUGxhY2UodGhpcy5FUFNJTE9OKTtcclxuICAgICAgICAgICAgICAgIHJldHVybiByZXNwb25zZS5hZGRUb1JlZihub3JtLCByZXNwb25zZSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaXQrKztcclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfSxcclxuXHJcbiAgICAvKipcclxuICAgICAqIENoZWNrcyBpZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3QgYXJlIGludGVyc2VjdGluZy5cclxuICAgICAqXHJcbiAgICAgKiBAbWV0aG9kIGlzSW50ZXJzZWN0aW5nXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxyXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cclxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IElzIGludGVyc2VjdGluZyBvciBub3QuXHJcbiAgICAgKi9cclxuICAgIGlzSW50ZXJzZWN0aW5nOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcclxuICAgICAgICByZXR1cm4gISF0aGlzLmNoZWNrKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cyk7XHJcbiAgICB9LFxyXG5cclxuICAgIC8qKlxyXG4gICAgICogQ2hlY2tzIGlmIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdCBhcmUgaW50ZXJzZWN0aW5nXHJcbiAgICAgKiBhbmQgZ2l2ZSB0aGUgcmVzcG9uc2UgdG8gYmUgb3V0IG9mIHRoZSBvYmplY3QuXHJcbiAgICAgKlxyXG4gICAgICogQG1ldGhvZCBpbnRlcnNlY3RcclxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXHJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxyXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHBlbmV0cmF0aW9uIHZlY3Rvci5cclxuICAgICAqL1xyXG4gICAgaW50ZXJzZWN0OiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcclxuICAgICAgICB2YXIgc2ltcGxleCA9IHRoaXMuY2hlY2soY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKTtcclxuXHJcbiAgICAgICAgLy90aGlzLmN1YmUgPSB0aGlzLmN1YmUgfHwgW107XHJcbiAgICAgICAgaWYgKHNpbXBsZXgpIHtcclxuICAgICAgICAgICAgcmV0dXJuIHRoaXMuZ2V0UmVzcG9uc2UoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4KTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gQ29sbGlzaW9uR2prRXBhO1xyXG4iXX0=
