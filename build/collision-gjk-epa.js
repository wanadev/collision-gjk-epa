(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.Collisiongjkepa = f()}})(function(){var define,module,exports;return (function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(require,module,exports){
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
                // Origin is on the edge
                n.y = -edge.x;
                n.x = edge.y;
                // normal should go outside the simplex
                var center = this._getBarycenter(simplex)
                var centerTov1 = v1.subtract(center);
                if (n.constructor.Dot(n, centerTov1) < 0) {
                    n.y = -n.y;
                    n.x = -n.x;
                }
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

//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJjb2xsaXNpb24tZ2prLWVwYS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsImZpbGUiOiJnZW5lcmF0ZWQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uKCl7ZnVuY3Rpb24gcihlLG4sdCl7ZnVuY3Rpb24gbyhpLGYpe2lmKCFuW2ldKXtpZighZVtpXSl7dmFyIGM9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZTtpZighZiYmYylyZXR1cm4gYyhpLCEwKTtpZih1KXJldHVybiB1KGksITApO3ZhciBhPW5ldyBFcnJvcihcIkNhbm5vdCBmaW5kIG1vZHVsZSAnXCIraStcIidcIik7dGhyb3cgYS5jb2RlPVwiTU9EVUxFX05PVF9GT1VORFwiLGF9dmFyIHA9bltpXT17ZXhwb3J0czp7fX07ZVtpXVswXS5jYWxsKHAuZXhwb3J0cyxmdW5jdGlvbihyKXt2YXIgbj1lW2ldWzFdW3JdO3JldHVybiBvKG58fHIpfSxwLHAuZXhwb3J0cyxyLGUsbix0KX1yZXR1cm4gbltpXS5leHBvcnRzfWZvcih2YXIgdT1cImZ1bmN0aW9uXCI9PXR5cGVvZiByZXF1aXJlJiZyZXF1aXJlLGk9MDtpPHQubGVuZ3RoO2krKylvKHRbaV0pO3JldHVybiBvfXJldHVybiByfSkoKSIsIid1c2Ugc3RyaWN0JztcblxuLyoqXG4gKiBDbGFzcyB0byBoZWxwIGluIHRoZSBjb2xsaXNpb24gaW4gMkQgYW5kIDNELlxuICogVG8gd29ya3MgdGhlIGFsZ29yaXRobSBuZWVkcyB0d28gY29udmV4ZSBwb2ludCBjbG91ZFxuICogbGlrZSBib3VuZGluZyBib3ggb3Igc210aGcgbGlrZSB0aGlzLlxuICogVGhlIGZ1bmN0aW9uIGZ1bmN0aW9ucyBpbnRlcnNlY3QgYW5kIGlzSW50ZXJzZWN0aW5nXG4gKiBoZWxwcyB0byBoYXZlIGluZm9ybWF0aW9ucyBhYm91dCB0aGUgY29sbGlzaW9uIGJldHdlZW4gdHdvIG9iamVjdC5cbiAqXG4gKiBAY2xhc3Mgd25wLmhlbHBlcnMuQ29sbGlzaW9uR2prRXBhXG4gKiBAY29uc3RydWN0b3JcbiAqL1xudmFyIFRyaWFuZ2xlID0gZnVuY3Rpb24oYSwgYiwgYykge1xuICAgICAgdGhpcy5hID0gYTtcbiAgICAgIHRoaXMuYiA9IGI7XG4gICAgICB0aGlzLmMgPSBjO1xuICAgICAgdGhpcy5uID0gYS5jb25zdHJ1Y3Rvci5Dcm9zcyhiLnN1YnRyYWN0KGEpLCBjLnN1YnRyYWN0KGEpKS5ub3JtYWxpemUoKTtcbiAgfTtcblxuXG52YXIgQ29sbGlzaW9uR2prRXBhID0ge1xuXG4gICAgRVBTSUxPTjogMC4wMDAwMDEsXG5cbiAgICAvKipcbiAgICAgKiBNZXRob2QgdG8gZ2V0IGEgbm9ybWFsIG9mIDMgcG9pbnRzIGluIDJEIGFuZCAzRC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5vcm1hbFxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBhXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBiXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBjXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIG5vcm1hbC5cbiAgICAgKi9cbiAgICBfZ2V0Tm9ybWFsOiBmdW5jdGlvbihhLCBiLCBjKSB7XG4gICAgICAgIHZhciBEb3QgPSBhLmNvbnN0cnVjdG9yLkRvdDtcbiAgICAgICAgdmFyIGFjID0gRG90KGEsIGMpOyAvLyBwZXJmb3JtIGFEb3QoYylcbiAgICAgICAgdmFyIGJjID0gRG90KGIsIGMpOyAvLyBwZXJmb3JtIGJEb3QoYylcblxuICAgICAgICAvLyBwZXJmb3JtIGIgKiBhLkRvdChjKSAtIGEgKiBiLkRvdChjKVxuICAgICAgICB2YXIgciA9IGIuc2NhbGUoYWMpLnN1YnRyYWN0KGEuc2NhbGUoYmMpKS5ub3JtYWxpemUoKTtcbiAgICAgICAgcmV0dXJuIHI7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIGJhcnljZW50ZXIgb2YgYSBjbG91ZCBwb2ludHMuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9nZXRCYXJ5Y2VudGVyXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gdmVydGljZXMgdGhlIGNsb3VkIHBvaW50c1xuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBiYXJ5Y2VudGVyLlxuICAgICAqL1xuICAgIF9nZXRCYXJ5Y2VudGVyOiBmdW5jdGlvbih2ZXJ0aWNlcykge1xuICAgICAgICB2YXIgYXZnID0gdmVydGljZXNbMF0uY29uc3RydWN0b3IuWmVybygpO1xuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHZlcnRpY2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICBhdmcuYWRkVG9SZWYodmVydGljZXNbaV0sIGF2Zyk7XG4gICAgICAgIH1cbiAgICAgICAgYXZnLnNjYWxlSW5QbGFjZSgxIC8gdmVydGljZXMubGVuZ3RoLCBhdmcpO1xuICAgICAgICByZXR1cm4gYXZnO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBmYXJ0aGVzdCBwb2ludCBvZiBhIGNsb3VkIHBvaW50cy5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvblxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHZlcnRpY2VzIHRoZSBjbG91ZCBwb2ludHMuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkIFRoZSBkaXJlY3Rpb24gdG8gc2VhcmNoLlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBiYXJ5Y2VudGVyLlxuICAgICAqL1xuICAgIF9nZXRGYXJ0aGVzdFBvaW50SW5EaXJlY3Rpb246IGZ1bmN0aW9uKHZlcnRpY2VzLCBkKSB7XG4gICAgICAgIHZhciBEb3QgPSB2ZXJ0aWNlc1swXS5jb25zdHJ1Y3Rvci5Eb3Q7XG4gICAgICAgIHZhciBtYXhQcm9kdWN0ID0gRG90KHZlcnRpY2VzWzBdLCBkKTtcbiAgICAgICAgdmFyIGluZGV4ID0gMDtcbiAgICAgICAgZm9yICh2YXIgaSA9IDE7IGkgPCB2ZXJ0aWNlcy5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgdmFyIHByb2R1Y3QgPSBEb3QodmVydGljZXNbaV0sIGQpO1xuICAgICAgICAgICAgaWYgKHByb2R1Y3QgPiBtYXhQcm9kdWN0KSB7XG4gICAgICAgICAgICAgICAgbWF4UHJvZHVjdCA9IHByb2R1Y3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIHJldHVybiB2ZXJ0aWNlc1tpbmRleF07XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIG5lYXJlc3QgZWRnZSBvZiB0aGUgc2ltcGxleC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RFZGdlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcmV0dXJuIHtPYmplY3R9IEluZm9ybWF0aW9ucyBhYm91dCB0aGUgbmVhcmVzdCBlZGdlIChkaXN0YW5jZSwgaW5kZXggYW5kIG5vcm1hbCkuXG4gICAgICovXG4gICAgX2dldE5lYXJlc3RFZGdlOiBmdW5jdGlvbihzaW1wbGV4KSB7XG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxuICAgICAgICAgICAgaW5kZXgsIG5vcm1hbDtcblxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHNpbXBsZXgubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIHZhciBqID0gKGkgKyAxKSAlIHNpbXBsZXgubGVuZ3RoO1xuICAgICAgICAgICAgdmFyIHYxID0gc2ltcGxleFtpXTtcbiAgICAgICAgICAgIHZhciB2MiA9IHNpbXBsZXhbal07XG5cbiAgICAgICAgICAgIHZhciBlZGdlID0gdjIuc3VidHJhY3QodjEpO1xuICAgICAgICAgICAgaWYgKGVkZ2UubGVuZ3RoU3F1YXJlZCgpID09PSAwKSB7XG4gICAgICAgICAgICAgICAgY29udGludWU7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIHZhciBvcmlnaW5Ub3YxID0gdjE7XG5cbiAgICAgICAgICAgIHZhciBuID0gdGhpcy5fZ2V0Tm9ybWFsKGVkZ2UsIG9yaWdpblRvdjEsIGVkZ2UpO1xuXG4gICAgICAgICAgICBpZiAobi5sZW5ndGhTcXVhcmVkKCkgPT09IDApIHtcbiAgICAgICAgICAgICAgICAvLyBPcmlnaW4gaXMgb24gdGhlIGVkZ2VcbiAgICAgICAgICAgICAgICBuLnkgPSAtZWRnZS54O1xuICAgICAgICAgICAgICAgIG4ueCA9IGVkZ2UueTtcbiAgICAgICAgICAgICAgICAvLyBub3JtYWwgc2hvdWxkIGdvIG91dHNpZGUgdGhlIHNpbXBsZXhcbiAgICAgICAgICAgICAgICB2YXIgY2VudGVyID0gdGhpcy5fZ2V0QmFyeWNlbnRlcihzaW1wbGV4KVxuICAgICAgICAgICAgICAgIHZhciBjZW50ZXJUb3YxID0gdjEuc3VidHJhY3QoY2VudGVyKTtcbiAgICAgICAgICAgICAgICBpZiAobi5jb25zdHJ1Y3Rvci5Eb3QobiwgY2VudGVyVG92MSkgPCAwKSB7XG4gICAgICAgICAgICAgICAgICAgIG4ueSA9IC1uLnk7XG4gICAgICAgICAgICAgICAgICAgIG4ueCA9IC1uLng7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICAvL24gPSBuLnNjYWxlKDEgLyBuLmxlbmd0aCgpKTsgLy9ub3JtYWxpemVcbiAgICAgICAgICAgIHZhciBkaXN0ID0gTWF0aC5hYnMobi5jb25zdHJ1Y3Rvci5Eb3QobiwgdjEpKTsgLy9kaXN0YW5jZSBmcm9tIG9yaWdpbiB0byBlZGdlXG5cbiAgICAgICAgICAgIGlmIChkaXN0IDwgZGlzdGFuY2UpIHtcbiAgICAgICAgICAgICAgICBkaXN0YW5jZSA9IGRpc3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBqO1xuICAgICAgICAgICAgICAgIG5vcm1hbCA9IG47XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4ge1xuICAgICAgICAgICAgZGlzdGFuY2U6IGRpc3RhbmNlLFxuICAgICAgICAgICAgaW5kZXg6IGluZGV4LFxuICAgICAgICAgICAgbm9ybWFsOiBub3JtYWxcbiAgICAgICAgfTtcblxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBuZWFyZXN0IFRyaWFuZ2xlIG9mIHRoZSBwb2x5dG9wZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RUcmlhbmdsZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlRyaWFuZ2xlW119IHBvbHl0b3BlIFRoZSBwb2x5dG9wZS5cbiAgICAgKiBAcmV0dXJuIHtPYmplY3R9IEluZm9ybWF0aW9ucyBhYm91dCB0aGUgbmVhcmVzdCBlZGdlIChkaXN0YW5jZSBhbmQgaW5kZXgpLlxuICAgICAqL1xuICAgIF9nZXROZWFyZXN0VHJpYW5nbGU6IGZ1bmN0aW9uKHBvbHl0b3BlKSB7XG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxuICAgICAgICAgICAgaW5kZXg7XG5cbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwb2x5dG9wZS5sZW5ndGg7IGkrKykge1xuXG4gICAgICAgICAgICB2YXIgdHJpYW5nbGUgPSBwb2x5dG9wZVtpXTtcbiAgICAgICAgICAgIHZhciBkaXN0ID0gTWF0aC5hYnModHJpYW5nbGUubi5jb25zdHJ1Y3Rvci5Eb3QodHJpYW5nbGUubiwgdHJpYW5nbGUuYSkpO1xuXG4gICAgICAgICAgICBpZiAoZGlzdCA8IGRpc3RhbmNlKSB7XG4gICAgICAgICAgICAgICAgZGlzdGFuY2UgPSBkaXN0O1xuICAgICAgICAgICAgICAgIGluZGV4ID0gaTtcbiAgICAgICAgICAgIH1cblxuICAgICAgICB9XG5cbiAgICAgICAgcmV0dXJuIHtcbiAgICAgICAgICAgIGRpc3RhbmNlOiBkaXN0YW5jZSxcbiAgICAgICAgICAgIGluZGV4OiBpbmRleFxuICAgICAgICB9O1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIG9yaWdpbiBpcyBpbiBhIGxpbmUuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc0xpbmVcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24uXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gRmFsc2UgaW4gYW55IGNhc2UgYmVjYXVzZSB0aGUgYWxnb3JpdGhtIGp1c3QgYmVnaW4uXG4gICAgICovXG4gICAgX2NvbnRhaW5zTGluZTogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIHZhciBhID0gc2ltcGxleFsxXTtcbiAgICAgICAgdmFyIGIgPSBzaW1wbGV4WzBdO1xuICAgICAgICB2YXIgYWIgPSBiLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYW8gPSBhLnNjYWxlKC0xKTtcbiAgICAgICAgaWYgKGFiLmxlbmd0aFNxdWFyZWQoKSAhPT0gMCkge1xuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYiwgYW8sIGFiKSk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICBkaXIuY29weUZyb20oYW8pO1xuICAgICAgICB9XG5cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIG9yaWdpbiBpcyBpbiBhIHRyaWFuZ2xlLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfY29udGFpbnNUcmlhbmdsZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBJZiBpbiAyRCBjYXNlLCBtYXkgcmV0dXJuIHRydWUgaWYgdGhlIG9yaWdpbiBpcyBpbiB0aGUgdHJpYW5nbGUuXG4gICAgICovXG4gICAgX2NvbnRhaW5zVHJpYW5nbGU6IGZ1bmN0aW9uKHNpbXBsZXgsIGRpcikge1xuICAgICAgICB2YXIgYSA9IHNpbXBsZXhbMl07XG4gICAgICAgIHZhciBEb3QgPSBzaW1wbGV4WzJdLmNvbnN0cnVjdG9yLkRvdDtcbiAgICAgICAgdmFyIGIgPSBzaW1wbGV4WzFdO1xuICAgICAgICB2YXIgYyA9IHNpbXBsZXhbMF07XG4gICAgICAgIHZhciBhYiA9IGIuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhYyA9IGMuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xuXG4gICAgICAgIHZhciBhYnAgPSB0aGlzLl9nZXROb3JtYWwoYWMsIGFiLCBhYik7XG4gICAgICAgIHZhciBhY3AgPSB0aGlzLl9nZXROb3JtYWwoYWIsIGFjLCBhYyk7XG4gICAgICAgIGlmIChEb3QoYWJwLCBhbykgPiAwKSB7XG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAxKTsgLy8gcmVtb3ZlIENcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYnApO1xuICAgICAgICB9IGVsc2UgaWYgKERvdChhY3AsIGFvKSA+IDApIHtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDEsIDEpOyAvLyByZW1vdmUgQlxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFjcCk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICBpZiAoZGlyLnogPT09IHVuZGVmaW5lZCkge1xuICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgICAgICB2YXIgYWJjID0gc2ltcGxleFsyXS5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWMpO1xuICAgICAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYmMpO1xuICAgICAgICAgICAgICAgIGlmIChEb3QoYWJjLCBhbykgPD0gMCkge1xuICAgICAgICAgICAgICAgICAgICAvL3Vwc2lkZSBkb3duIHRldHJhaGVkcm9uXG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBiO1xuICAgICAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gYztcbiAgICAgICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGE7XG4gICAgICAgICAgICAgICAgICAgIGRpci5jb3B5RnJvbShhYmMuc2NhbGUoLTEpKTtcbiAgICAgICAgICAgICAgICB9XG5cbiAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBvcmlnaW4gaXMgaW4gYSB0ZXRyYWhlZHJvbi5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2NvbnRhaW5zVGV0cmFoZWRyb25cbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24uXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gUmV0dXJuIHRydWUgaWYgdGhlIG9yaWdpbiBpcyBpbiB0aGUgdGV0cmFoZWRyb24uXG4gICAgICovXG4gICAgX2NvbnRhaW5zVGV0cmFoZWRyb246IGZ1bmN0aW9uKHNpbXBsZXgsIGRpcikge1xuICAgICAgICB2YXIgYSA9IHNpbXBsZXhbM107XG4gICAgICAgIHZhciBEb3QgPSBhLmNvbnN0cnVjdG9yLkRvdDtcbiAgICAgICAgdmFyIENyb3NzID0gYS5jb25zdHJ1Y3Rvci5Dcm9zcztcbiAgICAgICAgdmFyIGIgPSBzaW1wbGV4WzJdO1xuICAgICAgICB2YXIgYyA9IHNpbXBsZXhbMV07XG4gICAgICAgIHZhciBkID0gc2ltcGxleFswXTtcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFjID0gYy5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFkID0gZC5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFvID0gYS5zY2FsZSgtMSk7XG5cbiAgICAgICAgdmFyIGFiYyA9IENyb3NzKGFiLCBhYyk7XG4gICAgICAgIHZhciBhY2QgPSBDcm9zcyhhYywgYWQpO1xuICAgICAgICB2YXIgYWRiID0gQ3Jvc3MoYWQsIGFiKTtcblxuICAgICAgICB2YXIgYWJjVGVzdCA9IDB4MSxcbiAgICAgICAgICAgIGFjZFRlc3QgPSAweDIsXG4gICAgICAgICAgICBhZGJUZXN0ID0gMHg0O1xuXG4gICAgICAgIHZhciBwbGFuZVRlc3RzID0gKERvdChhYmMsIGFvKSA+IDAgPyBhYmNUZXN0IDogMCkgfFxuICAgICAgICAgICAgKERvdChhY2QsIGFvKSA+IDAgPyBhY2RUZXN0IDogMCkgfFxuICAgICAgICAgICAgKERvdChhZGIsIGFvKSA+IDAgPyBhZGJUZXN0IDogMCk7XG5cbiAgICAgICAgc3dpdGNoIChwbGFuZVRlc3RzKSB7XG4gICAgICAgICAgICBjYXNlIGFiY1Rlc3Q6XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhY2RUZXN0OlxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBjO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBkO1xuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1RldHJhaGVkcm9uKGFvLCBhYywgYWQsIGFjZCwgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWRiVGVzdDpcbiAgICAgICAgICAgICAgICAvL2luIGZyb250IG9mIHRyaWFuZ2xlIEFEQlxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBiO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBkO1xuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1RldHJhaGVkcm9uKGFvLCBhZCwgYWIsIGFkYiwgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWJjVGVzdCB8IGFjZFRlc3Q6XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhY2RUZXN0IHwgYWRiVGVzdDpcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gYztcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gZDtcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzBdID0gYjtcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUd29UZXRyYWhlZHJvbihhbywgYWMsIGFkLCBhY2QsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFkYlRlc3QgfCBhYmNUZXN0OlxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBiO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBkO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBjO1xuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1R3b1RldHJhaGVkcm9uKGFvLCBhZCwgYWIsIGFkYiwgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGRlZmF1bHQ6XG4gICAgICAgICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cblxuICAgICAgICAvL29yaWdpbiBpbiB0ZXRyYWhlZHJvblxuICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQG1ldGhvZCBfY2hlY2tUd29UZXRyYWhlZHJvblxuICAgICAqIEBwcml2YXRlXG4gICAgICovXG4gICAgX2NoZWNrVHdvVGV0cmFoZWRyb246IGZ1bmN0aW9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KSB7XG4gICAgICAgIHZhciBhYmNfYWMgPSBhYmMuY29uc3RydWN0b3IuQ3Jvc3MoYWJjLCBhYyk7XG5cbiAgICAgICAgaWYgKGFiY19hYy5jb25zdHJ1Y3Rvci5Eb3QoYWJjX2FjLCBhbykgPiAwKSB7XG4gICAgICAgICAgICAvL3RoZSBvcmlnaW4gaXMgYmV5b25kIEFDIGZyb20gQUJDJ3NcbiAgICAgICAgICAgIC8vcGVyc3BlY3RpdmUsIGVmZmVjdGl2ZWx5IGV4Y2x1ZGluZ1xuICAgICAgICAgICAgLy9BQ0QgZnJvbSBjb25zaWRlcmF0aW9uXG5cbiAgICAgICAgICAgIC8vd2UgdGh1cyBuZWVkIHRlc3Qgb25seSBBQ0RcbiAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBzaW1wbGV4WzFdO1xuICAgICAgICAgICAgc2ltcGxleFsxXSA9IHNpbXBsZXhbMF07XG5cbiAgICAgICAgICAgIGFiID0gc2ltcGxleFsyXS5zdWJ0cmFjdChzaW1wbGV4WzNdKTtcbiAgICAgICAgICAgIGFjID0gc2ltcGxleFsxXS5zdWJ0cmFjdChzaW1wbGV4WzNdKTtcbiAgICAgICAgICAgIGFiYyA9IGFiLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYyk7XG5cbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1RldHJhaGVkcm9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgfVxuXG4gICAgICAgIHZhciBhYl9hYmMgPSBhYmMuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFiYyk7XG5cbiAgICAgICAgaWYgKGFiX2FiYy5jb25zdHJ1Y3Rvci5Eb3QoYWJfYWJjLCBhbykgPiAwKSB7XG5cbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDIpO1xuICAgICAgICAgICAgLy9kaXIgaXMgbm90IGFiX2FiYyBiZWNhdXNlIGl0J3Mgbm90IHBvaW50IHRvd2FyZHMgdGhlIG9yaWdpbjtcbiAgICAgICAgICAgIC8vQUJ4QTB4QUIgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYiwgYW8sIGFiKSk7XG5cbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEBtZXRob2QgX2NoZWNrVGV0cmFoZWRyb25cbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqL1xuICAgIF9jaGVja1RldHJhaGVkcm9uOiBmdW5jdGlvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCkge1xuXG4gICAgICAgIHZhciBhY3AgPSBhYmMuY29uc3RydWN0b3IuQ3Jvc3MoYWJjLCBhYyk7XG5cbiAgICAgICAgaWYgKGFjcC5jb25zdHJ1Y3Rvci5Eb3QoYWNwLCBhbykgPiAwKSB7XG5cbiAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBzaW1wbGV4WzNdO1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMywgMSk7XG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAxKTtcbiAgICAgICAgICAgIC8vZGlyIGlzIG5vdCBhYmNfYWMgYmVjYXVzZSBpdCdzIG5vdCBwb2ludCB0b3dhcmRzIHRoZSBvcmlnaW47XG4gICAgICAgICAgICAvL0FDeEEweEFDIGRpcmVjdGlvbiB3ZSBhcmUgbG9va2luZyBmb3JcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbSh0aGlzLl9nZXROb3JtYWwoYWMsIGFvLCBhYykpO1xuXG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgIH1cblxuICAgICAgICAvL2FsbW9zdCB0aGUgc2FtZSBsaWtlIHRyaWFuZ2xlIGNoZWNrc1xuICAgICAgICB2YXIgYWJfYWJjID0gYWIuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFiYyk7XG5cbiAgICAgICAgaWYgKGFiX2FiYy5jb25zdHJ1Y3Rvci5Eb3QoYWJfYWJjLCBhbykgPiAwKSB7XG5cbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDIpO1xuICAgICAgICAgICAgLy9kaXIgaXMgbm90IGFiX2FiYyBiZWNhdXNlIGl0J3Mgbm90IHBvaW50IHRvd2FyZHMgdGhlIG9yaWdpbjtcbiAgICAgICAgICAgIC8vQUJ4QTB4QUIgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYiwgYW8sIGFiKSk7XG5cbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgICAgIC8vYnVpbGQgbmV3IHRldHJhaGVkcm9uIHdpdGggbmV3IGJhc2VcbiAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7XG5cbiAgICAgICAgZGlyLmNvcHlGcm9tKGFiYyk7XG5cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBBZGRzIGFkZ2UgdG8gdGhlIGxpc3QgYW5kIGNoZWNrcyBpZiB0aGUgZWRnZSBpZiBub3QgaW4uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9hZGRFZGdlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge09iamVjdFtdfSBlZGdlcyBUaGUgZWRnZXMuXG4gICAgICogQHBhcmFtIHtPYmplY3R9IGRpciBUaGUgZWRnZSB0byBjaGVjay5cbiAgICAgKi9cbiAgICBfYWRkRWRnZTogZnVuY3Rpb24oZWRnZXMsIGVkZ2UpIHtcbiAgICAgICAgZm9yICh2YXIgaiA9IDA7IGogPCBlZGdlcy5sZW5ndGg7IGorKykge1xuICAgICAgICAgICAgaWYgKGVkZ2VzW2pdLmEgPT09IGVkZ2UuYiAmJiBlZGdlc1tqXS5iID09PSBlZGdlLmEpIHtcbiAgICAgICAgICAgICAgICBlZGdlcy5zcGxpY2UoaiwgMSk7XG4gICAgICAgICAgICAgICAgcmV0dXJuO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIGVkZ2VzLnB1c2goZWRnZSk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIFRoZSBzdXBwb3J0IGZ1bmN0aW9uLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBzdXBwb3J0XG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyZWN0aW9uIFRoZSBkaXJlY3Rpb24uXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHN1cHBvcnQgcG9pbnRzLlxuICAgICAqL1xuICAgIHN1cHBvcnQ6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgZGlyZWN0aW9uKSB7XG4gICAgICAgIC8vIGQgaXMgYSB2ZWN0b3IgZGlyZWN0aW9uIChkb2Vzbid0IGhhdmUgdG8gYmUgbm9ybWFsaXplZClcbiAgICAgICAgLy8gZ2V0IHBvaW50cyBvbiB0aGUgZWRnZSBvZiB0aGUgc2hhcGVzIGluIG9wcG9zaXRlIGRpcmVjdGlvbnNcbiAgICAgICAgZGlyZWN0aW9uLm5vcm1hbGl6ZSgpO1xuICAgICAgICB2YXIgcDEgPSB0aGlzLl9nZXRGYXJ0aGVzdFBvaW50SW5EaXJlY3Rpb24oY29sbGlkZXJQb2ludHMsIGRpcmVjdGlvbik7XG4gICAgICAgIHZhciBwMiA9IHRoaXMuX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbihjb2xsaWRlZFBvaW50cywgZGlyZWN0aW9uLnNjYWxlKC0xKSk7XG4gICAgICAgIC8vIHBlcmZvcm0gdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXG4gICAgICAgIHZhciBwMyA9IHAxLnN1YnRyYWN0KHAyKTtcbiAgICAgICAgLy8gcDMgaXMgbm93IGEgcG9pbnQgaW4gTWlua293c2tpIHNwYWNlIG9uIHRoZSBlZGdlIG9mIHRoZSBNaW5rb3dza2kgRGlmZmVyZW5jZVxuICAgICAgICByZXR1cm4gcDM7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgc2ltcGxleCBjb250YWlucyB0aGUgb3JpZ2luLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBmaW5kUmVzcG9uc2VXaXRoRWRnZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4VGhlIHNpbXBsZXggb3IgZmFsc2UgaWYgbm8gaW50ZXJzZWN0aW9uLlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24gdG8gdGVzdC5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBDb250YWlucyBvciBub3QuXG4gICAgICovXG4gICAgY29udGFpbnNPcmlnaW46IGZ1bmN0aW9uKHNpbXBsZXgsIGRpcikge1xuICAgICAgICBpZiAoc2ltcGxleC5sZW5ndGggPT09IDIpIHtcbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jb250YWluc0xpbmUoc2ltcGxleCwgZGlyKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAoc2ltcGxleC5sZW5ndGggPT09IDMpIHtcbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jb250YWluc1RyaWFuZ2xlKHNpbXBsZXgsIGRpcik7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHNpbXBsZXgubGVuZ3RoID09PSA0KSB7XG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY29udGFpbnNUZXRyYWhlZHJvbihzaW1wbGV4LCBkaXIpO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogVGhlIEdKSyAoR2lsYmVydOKAk0pvaG5zb27igJNLZWVydGhpKSBhbGdvcml0aG0uXG4gICAgICogQ29tcHV0ZXMgc3VwcG9ydCBwb2ludHMgdG8gYnVpbGQgdGhlIE1pbmtvd3NreSBkaWZmZXJlbmNlIGFuZFxuICAgICAqIGNyZWF0ZSBhIHNpbXBsZXguIFRoZSBwb2ludHMgb2YgdGhlIGNvbGxpZGVyIGFuZCB0aGUgY29sbGlkZWQgb2JqZWN0XG4gICAgICogbXVzdCBiZSBjb252ZXhlLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBmaW5kUmVzcG9uc2VXaXRoRWRnZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IFRoZSBzaW1wbGV4IG9yIGZhbHNlIGlmIG5vIGludGVyc2VjdGlvbi5cbiAgICAgKi9cbiAgICBjaGVjazogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKSB7XG4gICAgICAgIHZhciBpdCA9IDA7XG4gICAgICAgIHZhciBhLCBzaW1wbGV4ID0gW107XG5cbiAgICAgICAgdmFyIGNvbGxpZGVyQ2VudGVyID0gdGhpcy5fZ2V0QmFyeWNlbnRlcihjb2xsaWRlclBvaW50cyk7XG4gICAgICAgIHZhciBjb2xsaWRlZENlbnRlciA9IHRoaXMuX2dldEJhcnljZW50ZXIoY29sbGlkZWRQb2ludHMpO1xuXG4gICAgICAgIC8vIGluaXRpYWwgZGlyZWN0aW9uIGZyb20gdGhlIGNlbnRlciBvZiAxc3QgYm9keSB0byB0aGUgY2VudGVyIG9mIDJuZCBib2R5XG4gICAgICAgIHZhciBkaXIgPSBjb2xsaWRlckNlbnRlci5zdWJ0cmFjdChjb2xsaWRlZENlbnRlcikubm9ybWFsaXplKCk7XG5cbiAgICAgICAgLy8gaWYgaW5pdGlhbCBkaXJlY3Rpb24gaXMgemVybyDigJMgc2V0IGl0IHRvIGFueSBhcmJpdHJhcnkgYXhpcyAod2UgY2hvb3NlIFgpXG4gICAgICAgIGlmIChkaXIubGVuZ3RoU3F1YXJlZCgpIDwgdGhpcy5FUFNJTE9OKSB7XG4gICAgICAgICAgICBkaXIueCA9IDE7XG4gICAgICAgIH1cblxuICAgICAgICAvLyBzZXQgdGhlIGZpcnN0IHN1cHBvcnQgYXMgaW5pdGlhbCBwb2ludCBvZiB0aGUgbmV3IHNpbXBsZXhcbiAgICAgICAgYSA9IHNpbXBsZXhbMF0gPSB0aGlzLnN1cHBvcnQoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBkaXIpO1xuXG4gICAgICAgIGlmIChhLmNvbnN0cnVjdG9yLkRvdChhLCBkaXIpIDw9IDApIHtcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgICAgIGRpci5zY2FsZUluUGxhY2UoLTEsIGRpcik7IC8vIHdlIHdpbGwgYmUgc2VhcmNoaW5nIGluIHRoZSBvcHBvc2l0ZSBkaXJlY3Rpb24gbmV4dFxuXG4gICAgICAgIHZhciBtYXggPSBjb2xsaWRlZFBvaW50cy5sZW5ndGggKiBjb2xsaWRlclBvaW50cy5sZW5ndGg7XG4gICAgICAgIHdoaWxlIChpdCA8IG1heCkge1xuICAgICAgICAgICAgaWYgKGRpci5sZW5ndGhTcXVhcmVkKCkgPT09IDAgJiYgc2ltcGxleC5sZW5ndGggPj0gMikge1xuICAgICAgICAgICAgICAgIC8vIEdldCBwZXJwZW5kaWN1bGFyIGRpcmVjdGlvbiB0byBsYXN0IHNpbXBsZXhcbiAgICAgICAgICAgICAgICBkaXIgPSBzaW1wbGV4W3NpbXBsZXgubGVuZ3RoIC0gMV0uc3VidHJhY3Qoc2ltcGxleFtzaW1wbGV4Lmxlbmd0aCAtIDJdKTtcbiAgICAgICAgICAgICAgICB2YXIgdG1wID0gZGlyLnk7XG4gICAgICAgICAgICAgICAgZGlyLnkgPSAtZGlyLng7XG4gICAgICAgICAgICAgICAgZGlyLnggPSB0bXA7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIGEgPSB0aGlzLnN1cHBvcnQoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBkaXIpO1xuXG4gICAgICAgICAgICAvLyBtYWtlIHN1cmUgdGhhdCB0aGUgbGFzdCBwb2ludCB3ZSBhZGRlZCBhY3R1YWxseSBwYXNzZWQgdGhlIG9yaWdpblxuICAgICAgICAgICAgaWYgKGEuY29uc3RydWN0b3IuRG90KGEsIGRpcikgPD0gMCkge1xuICAgICAgICAgICAgICAgIC8vIGlmIHRoZSBwb2ludCBhZGRlZCBsYXN0IHdhcyBub3QgcGFzdCB0aGUgb3JpZ2luIGluIHRoZSBkaXJlY3Rpb24gb2YgZFxuICAgICAgICAgICAgICAgIC8vIHRoZW4gdGhlIE1pbmtvd3NraSBTdW0gY2Fubm90IHBvc3NpYmx5IGNvbnRhaW4gdGhlIG9yaWdpbiBzaW5jZVxuICAgICAgICAgICAgICAgIC8vIHRoZSBsYXN0IHBvaW50IGFkZGVkIGlzIG9uIHRoZSBlZGdlIG9mIHRoZSBNaW5rb3dza2kgRGlmZmVyZW5jZVxuICAgICAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgc2ltcGxleC5wdXNoKGEpO1xuICAgICAgICAgICAgLy8gb3RoZXJ3aXNlIHdlIG5lZWQgdG8gZGV0ZXJtaW5lIGlmIHRoZSBvcmlnaW4gaXMgaW5cbiAgICAgICAgICAgIC8vIHRoZSBjdXJyZW50IHNpbXBsZXhcblxuICAgICAgICAgICAgaWYgKHRoaXMuY29udGFpbnNPcmlnaW4oc2ltcGxleCwgZGlyKSkge1xuICAgICAgICAgICAgICAgIHJldHVybiBzaW1wbGV4O1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgaXQrKztcbiAgICAgICAgfVxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBGaW5kcyB0aGUgcmVzcG9uc2Ugd2l0aCB0aGUgc2ltcGxleCAoZWRnZXMpIG9mIHRoZSBnamsgYWxnb3JpdGhtLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBmaW5kUmVzcG9uc2VXaXRoRWRnZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGZpbmRSZXNwb25zZVdpdGhFZGdlOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgpIHtcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLl9nZXROZWFyZXN0RWRnZShzaW1wbGV4KTtcbiAgICAgICAgdmFyIHN1cCA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGVkZ2Uubm9ybWFsKTsgLy9nZXQgc3VwcG9ydCBwb2ludCBpbiBkaXJlY3Rpb24gb2YgZWRnZSdzIG5vcm1hbFxuICAgICAgICB2YXIgZCA9IE1hdGguYWJzKHN1cC5jb25zdHJ1Y3Rvci5Eb3Qoc3VwLCBlZGdlLm5vcm1hbCkpO1xuXG4gICAgICAgIGlmIChkIC0gZWRnZS5kaXN0YW5jZSA8PSB0aGlzLkVQU0lMT04pIHtcbiAgICAgICAgICAgIHJldHVybiBlZGdlLm5vcm1hbC5zY2FsZShlZGdlLmRpc3RhbmNlKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKGVkZ2UuaW5kZXgsIDAsIHN1cCk7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBGaW5kcyB0aGUgcmVzcG9uc2Ugd2l0aCB0aGUgcG9seXRvcGUgZG9uZSB3aXRoIHRoZSBzaW1wbGV4IG9mIHRoZSBnamsgYWxnb3JpdGhtLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBmaW5kUmVzcG9uc2VXaXRoVHJpYW5nbGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7VHJpYW5nbGVbXX0gcG9seXRvcGUgVGhlIHBvbHl0b3BlIGRvbmUgd2l0aCB0aGUgc2ltcGxleC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZTogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBwb2x5dG9wZSkge1xuXG4gICAgICAgIGlmIChwb2x5dG9wZS5sZW5ndGggPT09IDApIHtcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgICAgIHZhciBuZWFyZXN0ID0gdGhpcy5fZ2V0TmVhcmVzdFRyaWFuZ2xlKHBvbHl0b3BlKTtcbiAgICAgICAgdmFyIHRyaWFuZ2xlID0gcG9seXRvcGVbbmVhcmVzdC5pbmRleF07XG5cbiAgICAgICAgdmFyIHN1cCA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHRyaWFuZ2xlLm4pO1xuXG4gICAgICAgIHZhciBkID0gTWF0aC5hYnMoc3VwLmNvbnN0cnVjdG9yLkRvdChzdXAsIHRyaWFuZ2xlLm4pKTtcblxuICAgICAgICBpZiAoKGQgLSBuZWFyZXN0LmRpc3RhbmNlIDw9IHRoaXMuRVBTSUxPTikpIHtcbiAgICAgICAgICAgIHJldHVybiB0cmlhbmdsZS5uLnNjYWxlKG5lYXJlc3QuZGlzdGFuY2UpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgdmFyIGVkZ2VzID0gW107XG4gICAgICAgICAgICBmb3IgKHZhciBpID0gcG9seXRvcGUubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pIHtcbiAgICAgICAgICAgICAgICB0cmlhbmdsZSA9IHBvbHl0b3BlW2ldO1xuICAgICAgICAgICAgICAgIC8vIGNhbiB0aGlzIGZhY2UgYmUgJ3NlZW4nIGJ5IGVudHJ5X2N1cl9zdXBwb3J0P1xuICAgICAgICAgICAgICAgIGlmICh0cmlhbmdsZS5uLmNvbnN0cnVjdG9yLkRvdCh0cmlhbmdsZS5uLCBzdXAuc3VidHJhY3QocG9seXRvcGVbaV0uYSkpID4gMCkge1xuICAgICAgICAgICAgICAgICAgICB0aGlzLl9hZGRFZGdlKGVkZ2VzLCB7XG4gICAgICAgICAgICAgICAgICAgICAgICBhOiB0cmlhbmdsZS5hLFxuICAgICAgICAgICAgICAgICAgICAgICAgYjogdHJpYW5nbGUuYlxuICAgICAgICAgICAgICAgICAgICB9KTtcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5fYWRkRWRnZShlZGdlcywge1xuICAgICAgICAgICAgICAgICAgICAgICAgYTogdHJpYW5nbGUuYixcbiAgICAgICAgICAgICAgICAgICAgICAgIGI6IHRyaWFuZ2xlLmNcbiAgICAgICAgICAgICAgICAgICAgfSk7XG4gICAgICAgICAgICAgICAgICAgIHRoaXMuX2FkZEVkZ2UoZWRnZXMsIHtcbiAgICAgICAgICAgICAgICAgICAgICAgIGE6IHRyaWFuZ2xlLmMsXG4gICAgICAgICAgICAgICAgICAgICAgICBiOiB0cmlhbmdsZS5hXG4gICAgICAgICAgICAgICAgICAgIH0pO1xuICAgICAgICAgICAgICAgICAgICBwb2x5dG9wZS5zcGxpY2UoaSwgMSk7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICAvLyBjcmVhdGUgbmV3IHRyaWFuZ2xlcyBmcm9tIHRoZSBlZGdlcyBpbiB0aGUgZWRnZSBsaXN0XG4gICAgICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGVkZ2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICAgICAgdHJpYW5nbGUgPSBuZXcgVHJpYW5nbGUoc3VwLCBlZGdlc1tpXS5hLCBlZGdlc1tpXS5iKTtcbiAgICAgICAgICAgICAgICBpZiAodHJpYW5nbGUubi5sZW5ndGgoKSAhPT0gMCkge1xuICAgICAgICAgICAgICAgICAgICBwb2x5dG9wZS5wdXNoKHRyaWFuZ2xlKTtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIHJlc3BvbnNlIG9mIHRoZSBwZW5ldHJhdGlvbiB2ZWN0b3Igd2l0aCB0aGUgc2ltcGxleC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgZ2V0UmVzcG9uc2VcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXggb2YgdGhlIE1pbmtvd3NreSBkaWZmZXJlbmNlLlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgZ2V0UmVzcG9uc2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCkge1xuICAgICAgICB2YXIgaXQgPSAwLFxuICAgICAgICAgICAgcmVzcG9uc2U7XG4gICAgICAgIHZhciBwb2x5dG9wZSA9IHNpbXBsZXhbMF0ueiAhPT0gdW5kZWZpbmVkID8gW25ldyBUcmlhbmdsZShzaW1wbGV4WzBdLCBzaW1wbGV4WzFdLCBzaW1wbGV4WzJdKSxcbiAgICAgICAgICAgIG5ldyBUcmlhbmdsZShzaW1wbGV4WzBdLCBzaW1wbGV4WzJdLCBzaW1wbGV4WzNdKSxcbiAgICAgICAgICAgIG5ldyBUcmlhbmdsZShzaW1wbGV4WzBdLCBzaW1wbGV4WzNdLCBzaW1wbGV4WzFdKSxcbiAgICAgICAgICAgIG5ldyBUcmlhbmdsZShzaW1wbGV4WzFdLCBzaW1wbGV4WzNdLCBzaW1wbGV4WzJdKVxuICAgICAgICBdIDogbnVsbDtcblxuICAgICAgICB2YXIgbWF4ID0gY29sbGlkZWRQb2ludHMubGVuZ3RoICogY29sbGlkZXJQb2ludHMubGVuZ3RoO1xuICAgICAgICB3aGlsZSAoaXQgPCBtYXgpIHtcbiAgICAgICAgICAgIGlmIChzaW1wbGV4WzBdLnogPT09IHVuZGVmaW5lZCkge1xuICAgICAgICAgICAgICAgIHJlc3BvbnNlID0gdGhpcy5maW5kUmVzcG9uc2VXaXRoRWRnZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgpO1xuICAgICAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgICAgICByZXNwb25zZSA9IHRoaXMuZmluZFJlc3BvbnNlV2l0aFRyaWFuZ2xlKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgcG9seXRvcGUpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgaWYgKHJlc3BvbnNlKSB7XG4gICAgICAgICAgICAgICAgdmFyIG5vcm0gPSByZXNwb25zZS5jbG9uZSgpLm5vcm1hbGl6ZSgpLnNjYWxlSW5QbGFjZSh0aGlzLkVQU0lMT04pO1xuICAgICAgICAgICAgICAgIHJldHVybiByZXNwb25zZS5hZGRUb1JlZihub3JtLCByZXNwb25zZSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpdCsrO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdCBhcmUgaW50ZXJzZWN0aW5nLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBpc0ludGVyc2VjdGluZ1xuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBJcyBpbnRlcnNlY3Rpbmcgb3Igbm90LlxuICAgICAqL1xuICAgIGlzSW50ZXJzZWN0aW5nOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcbiAgICAgICAgcmV0dXJuICEhdGhpcy5jaGVjayhjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIGNvbGxpZGVyIGFuZCB0aGUgY29sbGlkZWQgb2JqZWN0IGFyZSBpbnRlcnNlY3RpbmdcbiAgICAgKiBhbmQgZ2l2ZSB0aGUgcmVzcG9uc2UgdG8gYmUgb3V0IG9mIHRoZSBvYmplY3QuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGludGVyc2VjdFxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGludGVyc2VjdDogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKSB7XG4gICAgICAgIHZhciBzaW1wbGV4ID0gdGhpcy5jaGVjayhjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpO1xuXG4gICAgICAgIC8vdGhpcy5jdWJlID0gdGhpcy5jdWJlIHx8IFtdO1xuICAgICAgICBpZiAoc2ltcGxleCkge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuZ2V0UmVzcG9uc2UoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4KTtcbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IENvbGxpc2lvbkdqa0VwYTtcbiJdfQ==
