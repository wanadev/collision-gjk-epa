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
        var r = b.scale(ac).subtract(a.scale(bc));
        if (r.lengthSquared() < this.EPSILON * this.EPSILON) {
            return r.scale(0);
        }
        return r.normalize();
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

//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJjb2xsaXNpb24tZ2prLWVwYS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsImZpbGUiOiJnZW5lcmF0ZWQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uKCl7ZnVuY3Rpb24gcihlLG4sdCl7ZnVuY3Rpb24gbyhpLGYpe2lmKCFuW2ldKXtpZighZVtpXSl7dmFyIGM9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZTtpZighZiYmYylyZXR1cm4gYyhpLCEwKTtpZih1KXJldHVybiB1KGksITApO3ZhciBhPW5ldyBFcnJvcihcIkNhbm5vdCBmaW5kIG1vZHVsZSAnXCIraStcIidcIik7dGhyb3cgYS5jb2RlPVwiTU9EVUxFX05PVF9GT1VORFwiLGF9dmFyIHA9bltpXT17ZXhwb3J0czp7fX07ZVtpXVswXS5jYWxsKHAuZXhwb3J0cyxmdW5jdGlvbihyKXt2YXIgbj1lW2ldWzFdW3JdO3JldHVybiBvKG58fHIpfSxwLHAuZXhwb3J0cyxyLGUsbix0KX1yZXR1cm4gbltpXS5leHBvcnRzfWZvcih2YXIgdT1cImZ1bmN0aW9uXCI9PXR5cGVvZiByZXF1aXJlJiZyZXF1aXJlLGk9MDtpPHQubGVuZ3RoO2krKylvKHRbaV0pO3JldHVybiBvfXJldHVybiByfSkoKSIsIid1c2Ugc3RyaWN0JztcblxuLyoqXG4gKiBDbGFzcyB0byBoZWxwIGluIHRoZSBjb2xsaXNpb24gaW4gMkQgYW5kIDNELlxuICogVG8gd29ya3MgdGhlIGFsZ29yaXRobSBuZWVkcyB0d28gY29udmV4ZSBwb2ludCBjbG91ZFxuICogbGlrZSBib3VuZGluZyBib3ggb3Igc210aGcgbGlrZSB0aGlzLlxuICogVGhlIGZ1bmN0aW9uIGZ1bmN0aW9ucyBpbnRlcnNlY3QgYW5kIGlzSW50ZXJzZWN0aW5nXG4gKiBoZWxwcyB0byBoYXZlIGluZm9ybWF0aW9ucyBhYm91dCB0aGUgY29sbGlzaW9uIGJldHdlZW4gdHdvIG9iamVjdC5cbiAqXG4gKiBAY2xhc3Mgd25wLmhlbHBlcnMuQ29sbGlzaW9uR2prRXBhXG4gKiBAY29uc3RydWN0b3JcbiAqL1xudmFyIFRyaWFuZ2xlID0gZnVuY3Rpb24oYSwgYiwgYykge1xuICAgICAgdGhpcy5hID0gYTtcbiAgICAgIHRoaXMuYiA9IGI7XG4gICAgICB0aGlzLmMgPSBjO1xuICAgICAgdGhpcy5uID0gYS5jb25zdHJ1Y3Rvci5Dcm9zcyhiLnN1YnRyYWN0KGEpLCBjLnN1YnRyYWN0KGEpKS5ub3JtYWxpemUoKTtcbiAgfTtcblxuXG52YXIgQ29sbGlzaW9uR2prRXBhID0ge1xuXG4gICAgRVBTSUxPTjogMC4wMDAwMDEsXG5cbiAgICAvKipcbiAgICAgKiBNZXRob2QgdG8gZ2V0IGEgbm9ybWFsIG9mIDMgcG9pbnRzIGluIDJEIGFuZCAzRC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5vcm1hbFxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBhXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBiXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBjXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIG5vcm1hbC5cbiAgICAgKi9cbiAgICBfZ2V0Tm9ybWFsOiBmdW5jdGlvbihhLCBiLCBjKSB7XG4gICAgICAgIHZhciBEb3QgPSBhLmNvbnN0cnVjdG9yLkRvdDtcbiAgICAgICAgdmFyIGFjID0gRG90KGEsIGMpOyAvLyBwZXJmb3JtIGFEb3QoYylcbiAgICAgICAgdmFyIGJjID0gRG90KGIsIGMpOyAvLyBwZXJmb3JtIGJEb3QoYylcblxuICAgICAgICAvLyBwZXJmb3JtIGIgKiBhLkRvdChjKSAtIGEgKiBiLkRvdChjKVxuICAgICAgICB2YXIgciA9IGIuc2NhbGUoYWMpLnN1YnRyYWN0KGEuc2NhbGUoYmMpKTtcbiAgICAgICAgaWYgKHIubGVuZ3RoU3F1YXJlZCgpIDwgdGhpcy5FUFNJTE9OICogdGhpcy5FUFNJTE9OKSB7XG4gICAgICAgICAgICByZXR1cm4gci5zY2FsZSgwKTtcbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gci5ub3JtYWxpemUoKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyB0aGUgYmFyeWNlbnRlciBvZiBhIGNsb3VkIHBvaW50cy5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldEJhcnljZW50ZXJcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSB2ZXJ0aWNlcyB0aGUgY2xvdWQgcG9pbnRzXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIGJhcnljZW50ZXIuXG4gICAgICovXG4gICAgX2dldEJhcnljZW50ZXI6IGZ1bmN0aW9uKHZlcnRpY2VzKSB7XG4gICAgICAgIHZhciBhdmcgPSB2ZXJ0aWNlc1swXS5jb25zdHJ1Y3Rvci5aZXJvKCk7XG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdmVydGljZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIGF2Zy5hZGRUb1JlZih2ZXJ0aWNlc1tpXSwgYXZnKTtcbiAgICAgICAgfVxuICAgICAgICBhdmcuc2NhbGVJblBsYWNlKDEgLyB2ZXJ0aWNlcy5sZW5ndGgsIGF2Zyk7XG4gICAgICAgIHJldHVybiBhdmc7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIGZhcnRoZXN0IHBvaW50IG9mIGEgY2xvdWQgcG9pbnRzLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gdmVydGljZXMgdGhlIGNsb3VkIHBvaW50cy5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGQgVGhlIGRpcmVjdGlvbiB0byBzZWFyY2guXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIGJhcnljZW50ZXIuXG4gICAgICovXG4gICAgX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbjogZnVuY3Rpb24odmVydGljZXMsIGQpIHtcbiAgICAgICAgdmFyIERvdCA9IHZlcnRpY2VzWzBdLmNvbnN0cnVjdG9yLkRvdDtcbiAgICAgICAgdmFyIG1heFByb2R1Y3QgPSBEb3QodmVydGljZXNbMF0sIGQpO1xuICAgICAgICB2YXIgaW5kZXggPSAwO1xuICAgICAgICBmb3IgKHZhciBpID0gMTsgaSA8IHZlcnRpY2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICB2YXIgcHJvZHVjdCA9IERvdCh2ZXJ0aWNlc1tpXSwgZCk7XG4gICAgICAgICAgICBpZiAocHJvZHVjdCA+IG1heFByb2R1Y3QpIHtcbiAgICAgICAgICAgICAgICBtYXhQcm9kdWN0ID0gcHJvZHVjdDtcbiAgICAgICAgICAgICAgICBpbmRleCA9IGk7XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIHZlcnRpY2VzW2luZGV4XTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyB0aGUgbmVhcmVzdCBlZGdlIG9mIHRoZSBzaW1wbGV4LlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0TmVhcmVzdEVkZ2VcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEByZXR1cm4ge09iamVjdH0gSW5mb3JtYXRpb25zIGFib3V0IHRoZSBuZWFyZXN0IGVkZ2UgKGRpc3RhbmNlLCBpbmRleCBhbmQgbm9ybWFsKS5cbiAgICAgKi9cbiAgICBfZ2V0TmVhcmVzdEVkZ2U6IGZ1bmN0aW9uKHNpbXBsZXgpIHtcbiAgICAgICAgdmFyIGRpc3RhbmNlID0gSW5maW5pdHksXG4gICAgICAgICAgICBpbmRleCwgbm9ybWFsO1xuXG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgc2ltcGxleC5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgdmFyIGogPSAoaSArIDEpICUgc2ltcGxleC5sZW5ndGg7XG4gICAgICAgICAgICB2YXIgdjEgPSBzaW1wbGV4W2ldO1xuICAgICAgICAgICAgdmFyIHYyID0gc2ltcGxleFtqXTtcblxuICAgICAgICAgICAgdmFyIGVkZ2UgPSB2Mi5zdWJ0cmFjdCh2MSk7XG4gICAgICAgICAgICBpZiAoZWRnZS5sZW5ndGhTcXVhcmVkKCkgPT09IDApIHtcbiAgICAgICAgICAgICAgICBjb250aW51ZTtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgdmFyIG9yaWdpblRvdjEgPSB2MTtcblxuICAgICAgICAgICAgdmFyIG4gPSB0aGlzLl9nZXROb3JtYWwoZWRnZSwgb3JpZ2luVG92MSwgZWRnZSk7XG5cbiAgICAgICAgICAgIGlmIChuLmxlbmd0aFNxdWFyZWQoKSA9PT0gMCkge1xuICAgICAgICAgICAgICAgIC8vIE9yaWdpbiBpcyBvbiB0aGUgZWRnZVxuICAgICAgICAgICAgICAgIG4ueSA9IC1lZGdlLng7XG4gICAgICAgICAgICAgICAgbi54ID0gZWRnZS55O1xuICAgICAgICAgICAgICAgIC8vIG5vcm1hbCBzaG91bGQgZ28gb3V0c2lkZSB0aGUgc2ltcGxleFxuICAgICAgICAgICAgICAgIHZhciBjZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKHNpbXBsZXgpXG4gICAgICAgICAgICAgICAgdmFyIGNlbnRlclRvdjEgPSB2MS5zdWJ0cmFjdChjZW50ZXIpO1xuICAgICAgICAgICAgICAgIGlmIChuLmNvbnN0cnVjdG9yLkRvdChuLCBjZW50ZXJUb3YxKSA8IDApIHtcbiAgICAgICAgICAgICAgICAgICAgbi55ID0gLW4ueTtcbiAgICAgICAgICAgICAgICAgICAgbi54ID0gLW4ueDtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIC8vbiA9IG4uc2NhbGUoMSAvIG4ubGVuZ3RoKCkpOyAvL25vcm1hbGl6ZVxuICAgICAgICAgICAgdmFyIGRpc3QgPSBNYXRoLmFicyhuLmNvbnN0cnVjdG9yLkRvdChuLCB2MSkpOyAvL2Rpc3RhbmNlIGZyb20gb3JpZ2luIHRvIGVkZ2VcblxuICAgICAgICAgICAgaWYgKGRpc3QgPCBkaXN0YW5jZSkge1xuICAgICAgICAgICAgICAgIGRpc3RhbmNlID0gZGlzdDtcbiAgICAgICAgICAgICAgICBpbmRleCA9IGo7XG4gICAgICAgICAgICAgICAgbm9ybWFsID0gbjtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiB7XG4gICAgICAgICAgICBkaXN0YW5jZTogZGlzdGFuY2UsXG4gICAgICAgICAgICBpbmRleDogaW5kZXgsXG4gICAgICAgICAgICBub3JtYWw6IG5vcm1hbFxuICAgICAgICB9O1xuXG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIG5lYXJlc3QgVHJpYW5nbGUgb2YgdGhlIHBvbHl0b3BlLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0TmVhcmVzdFRyaWFuZ2xlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVHJpYW5nbGVbXX0gcG9seXRvcGUgVGhlIHBvbHl0b3BlLlxuICAgICAqIEByZXR1cm4ge09iamVjdH0gSW5mb3JtYXRpb25zIGFib3V0IHRoZSBuZWFyZXN0IGVkZ2UgKGRpc3RhbmNlIGFuZCBpbmRleCkuXG4gICAgICovXG4gICAgX2dldE5lYXJlc3RUcmlhbmdsZTogZnVuY3Rpb24ocG9seXRvcGUpIHtcbiAgICAgICAgdmFyIGRpc3RhbmNlID0gSW5maW5pdHksXG4gICAgICAgICAgICBpbmRleDtcblxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHBvbHl0b3BlLmxlbmd0aDsgaSsrKSB7XG5cbiAgICAgICAgICAgIHZhciB0cmlhbmdsZSA9IHBvbHl0b3BlW2ldO1xuICAgICAgICAgICAgdmFyIGRpc3QgPSBNYXRoLmFicyh0cmlhbmdsZS5uLmNvbnN0cnVjdG9yLkRvdCh0cmlhbmdsZS5uLCB0cmlhbmdsZS5hKSk7XG5cbiAgICAgICAgICAgIGlmIChkaXN0IDwgZGlzdGFuY2UpIHtcbiAgICAgICAgICAgICAgICBkaXN0YW5jZSA9IGRpc3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4ge1xuICAgICAgICAgICAgZGlzdGFuY2U6IGRpc3RhbmNlLFxuICAgICAgICAgICAgaW5kZXg6IGluZGV4XG4gICAgICAgIH07XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgbGluZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2NvbnRhaW5zTGluZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBGYWxzZSBpbiBhbnkgY2FzZSBiZWNhdXNlIHRoZSBhbGdvcml0aG0ganVzdCBiZWdpbi5cbiAgICAgKi9cbiAgICBfY29udGFpbnNMaW5lOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcbiAgICAgICAgdmFyIGEgPSBzaW1wbGV4WzFdO1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMF07XG4gICAgICAgIHZhciBhYiA9IGIuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xuICAgICAgICBpZiAoYWIubGVuZ3RoU3F1YXJlZCgpICE9PSAwKSB7XG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbShhbyk7XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgdHJpYW5nbGUuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc1RyaWFuZ2xlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uLlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IElmIGluIDJEIGNhc2UsIG1heSByZXR1cm4gdHJ1ZSBpZiB0aGUgb3JpZ2luIGlzIGluIHRoZSB0cmlhbmdsZS5cbiAgICAgKi9cbiAgICBfY29udGFpbnNUcmlhbmdsZTogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIHZhciBhID0gc2ltcGxleFsyXTtcbiAgICAgICAgdmFyIERvdCA9IHNpbXBsZXhbMl0uY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMV07XG4gICAgICAgIHZhciBjID0gc2ltcGxleFswXTtcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFjID0gYy5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFvID0gYS5zY2FsZSgtMSk7XG5cbiAgICAgICAgdmFyIGFicCA9IHRoaXMuX2dldE5vcm1hbChhYywgYWIsIGFiKTtcbiAgICAgICAgdmFyIGFjcCA9IHRoaXMuX2dldE5vcm1hbChhYiwgYWMsIGFjKTtcbiAgICAgICAgaWYgKERvdChhYnAsIGFvKSA+IDApIHtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDEpOyAvLyByZW1vdmUgQ1xuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFicCk7XG4gICAgICAgIH0gZWxzZSBpZiAoRG90KGFjcCwgYW8pID4gMCkge1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMSwgMSk7IC8vIHJlbW92ZSBCXG4gICAgICAgICAgICBkaXIuY29weUZyb20oYWNwKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIGlmIChkaXIueiA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgICAgIHZhciBhYmMgPSBzaW1wbGV4WzJdLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYyk7XG4gICAgICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFiYyk7XG4gICAgICAgICAgICAgICAgaWYgKERvdChhYmMsIGFvKSA8PSAwKSB7XG4gICAgICAgICAgICAgICAgICAgIC8vdXBzaWRlIGRvd24gdGV0cmFoZWRyb25cbiAgICAgICAgICAgICAgICAgICAgc2ltcGxleFswXSA9IGI7XG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBjO1xuICAgICAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gYTtcbiAgICAgICAgICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFiYy5zY2FsZSgtMSkpO1xuICAgICAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIG9yaWdpbiBpcyBpbiBhIHRldHJhaGVkcm9uLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfY29udGFpbnNUZXRyYWhlZHJvblxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBSZXR1cm4gdHJ1ZSBpZiB0aGUgb3JpZ2luIGlzIGluIHRoZSB0ZXRyYWhlZHJvbi5cbiAgICAgKi9cbiAgICBfY29udGFpbnNUZXRyYWhlZHJvbjogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIHZhciBhID0gc2ltcGxleFszXTtcbiAgICAgICAgdmFyIERvdCA9IGEuY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgQ3Jvc3MgPSBhLmNvbnN0cnVjdG9yLkNyb3NzO1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMl07XG4gICAgICAgIHZhciBjID0gc2ltcGxleFsxXTtcbiAgICAgICAgdmFyIGQgPSBzaW1wbGV4WzBdO1xuICAgICAgICB2YXIgYWIgPSBiLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYWMgPSBjLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYWQgPSBkLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYW8gPSBhLnNjYWxlKC0xKTtcblxuICAgICAgICB2YXIgYWJjID0gQ3Jvc3MoYWIsIGFjKTtcbiAgICAgICAgdmFyIGFjZCA9IENyb3NzKGFjLCBhZCk7XG4gICAgICAgIHZhciBhZGIgPSBDcm9zcyhhZCwgYWIpO1xuXG4gICAgICAgIHZhciBhYmNUZXN0ID0gMHgxLFxuICAgICAgICAgICAgYWNkVGVzdCA9IDB4MixcbiAgICAgICAgICAgIGFkYlRlc3QgPSAweDQ7XG5cbiAgICAgICAgdmFyIHBsYW5lVGVzdHMgPSAoRG90KGFiYywgYW8pID4gMCA/IGFiY1Rlc3QgOiAwKSB8XG4gICAgICAgICAgICAoRG90KGFjZCwgYW8pID4gMCA/IGFjZFRlc3QgOiAwKSB8XG4gICAgICAgICAgICAoRG90KGFkYiwgYW8pID4gMCA/IGFkYlRlc3QgOiAwKTtcblxuICAgICAgICBzd2l0Y2ggKHBsYW5lVGVzdHMpIHtcbiAgICAgICAgICAgIGNhc2UgYWJjVGVzdDpcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFjZFRlc3Q6XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGM7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGQ7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFjLCBhZCwgYWNkLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhZGJUZXN0OlxuICAgICAgICAgICAgICAgIC8vaW4gZnJvbnQgb2YgdHJpYW5nbGUgQURCXG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGI7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGQ7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhYmNUZXN0IHwgYWNkVGVzdDpcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUd29UZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFjZFRlc3QgfCBhZGJUZXN0OlxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBjO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBkO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBiO1xuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1R3b1RldHJhaGVkcm9uKGFvLCBhYywgYWQsIGFjZCwgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWRiVGVzdCB8IGFiY1Rlc3Q6XG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGI7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGQ7XG4gICAgICAgICAgICAgICAgc2ltcGxleFswXSA9IGM7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgZGVmYXVsdDpcbiAgICAgICAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuXG4gICAgICAgIC8vb3JpZ2luIGluIHRldHJhaGVkcm9uXG4gICAgICAgIHJldHVybiB0cnVlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBAbWV0aG9kIF9jaGVja1R3b1RldHJhaGVkcm9uXG4gICAgICogQHByaXZhdGVcbiAgICAgKi9cbiAgICBfY2hlY2tUd29UZXRyYWhlZHJvbjogZnVuY3Rpb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpIHtcbiAgICAgICAgdmFyIGFiY19hYyA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYmMsIGFjKTtcblxuICAgICAgICBpZiAoYWJjX2FjLmNvbnN0cnVjdG9yLkRvdChhYmNfYWMsIGFvKSA+IDApIHtcbiAgICAgICAgICAgIC8vdGhlIG9yaWdpbiBpcyBiZXlvbmQgQUMgZnJvbSBBQkMnc1xuICAgICAgICAgICAgLy9wZXJzcGVjdGl2ZSwgZWZmZWN0aXZlbHkgZXhjbHVkaW5nXG4gICAgICAgICAgICAvL0FDRCBmcm9tIGNvbnNpZGVyYXRpb25cblxuICAgICAgICAgICAgLy93ZSB0aHVzIG5lZWQgdGVzdCBvbmx5IEFDRFxuICAgICAgICAgICAgc2ltcGxleFsyXSA9IHNpbXBsZXhbMV07XG4gICAgICAgICAgICBzaW1wbGV4WzFdID0gc2ltcGxleFswXTtcblxuICAgICAgICAgICAgYWIgPSBzaW1wbGV4WzJdLnN1YnRyYWN0KHNpbXBsZXhbM10pO1xuICAgICAgICAgICAgYWMgPSBzaW1wbGV4WzFdLnN1YnRyYWN0KHNpbXBsZXhbM10pO1xuICAgICAgICAgICAgYWJjID0gYWIuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFjKTtcblxuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIGFiX2FiYyA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWJjKTtcblxuICAgICAgICBpZiAoYWJfYWJjLmNvbnN0cnVjdG9yLkRvdChhYl9hYmMsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMik7XG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJfYWJjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xuICAgICAgICAgICAgLy9BQnhBMHhBQiBkaXJlY3Rpb24gd2UgYXJlIGxvb2tpbmcgZm9yXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcblxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQG1ldGhvZCBfY2hlY2tUZXRyYWhlZHJvblxuICAgICAqIEBwcml2YXRlXG4gICAgICovXG4gICAgX2NoZWNrVGV0cmFoZWRyb246IGZ1bmN0aW9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KSB7XG5cbiAgICAgICAgdmFyIGFjcCA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYmMsIGFjKTtcblxuICAgICAgICBpZiAoYWNwLmNvbnN0cnVjdG9yLkRvdChhY3AsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleFsyXSA9IHNpbXBsZXhbM107XG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgzLCAxKTtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDEpO1xuICAgICAgICAgICAgLy9kaXIgaXMgbm90IGFiY19hYyBiZWNhdXNlIGl0J3Mgbm90IHBvaW50IHRvd2FyZHMgdGhlIG9yaWdpbjtcbiAgICAgICAgICAgIC8vQUN4QTB4QUMgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYywgYW8sIGFjKSk7XG5cbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgICAgIC8vYWxtb3N0IHRoZSBzYW1lIGxpa2UgdHJpYW5nbGUgY2hlY2tzXG4gICAgICAgIHZhciBhYl9hYmMgPSBhYi5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWJjKTtcblxuICAgICAgICBpZiAoYWJfYWJjLmNvbnN0cnVjdG9yLkRvdChhYl9hYmMsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMik7XG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJfYWJjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xuICAgICAgICAgICAgLy9BQnhBMHhBQiBkaXJlY3Rpb24gd2UgYXJlIGxvb2tpbmcgZm9yXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcblxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgLy9idWlsZCBuZXcgdGV0cmFoZWRyb24gd2l0aCBuZXcgYmFzZVxuICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAxKTtcblxuICAgICAgICBkaXIuY29weUZyb20oYWJjKTtcblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYWRnZSB0byB0aGUgbGlzdCBhbmQgY2hlY2tzIGlmIHRoZSBlZGdlIGlmIG5vdCBpbi5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2FkZEVkZ2VcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7T2JqZWN0W119IGVkZ2VzIFRoZSBlZGdlcy5cbiAgICAgKiBAcGFyYW0ge09iamVjdH0gZGlyIFRoZSBlZGdlIHRvIGNoZWNrLlxuICAgICAqL1xuICAgIF9hZGRFZGdlOiBmdW5jdGlvbihlZGdlcywgZWRnZSkge1xuICAgICAgICBmb3IgKHZhciBqID0gMDsgaiA8IGVkZ2VzLmxlbmd0aDsgaisrKSB7XG4gICAgICAgICAgICBpZiAoZWRnZXNbal0uYSA9PT0gZWRnZS5iICYmIGVkZ2VzW2pdLmIgPT09IGVkZ2UuYSkge1xuICAgICAgICAgICAgICAgIGVkZ2VzLnNwbGljZShqLCAxKTtcbiAgICAgICAgICAgICAgICByZXR1cm47XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgZWRnZXMucHVzaChlZGdlKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogVGhlIHN1cHBvcnQgZnVuY3Rpb24uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIHN1cHBvcnRcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXJlY3Rpb24gVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgc3VwcG9ydCBwb2ludHMuXG4gICAgICovXG4gICAgc3VwcG9ydDogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBkaXJlY3Rpb24pIHtcbiAgICAgICAgLy8gZCBpcyBhIHZlY3RvciBkaXJlY3Rpb24gKGRvZXNuJ3QgaGF2ZSB0byBiZSBub3JtYWxpemVkKVxuICAgICAgICAvLyBnZXQgcG9pbnRzIG9uIHRoZSBlZGdlIG9mIHRoZSBzaGFwZXMgaW4gb3Bwb3NpdGUgZGlyZWN0aW9uc1xuICAgICAgICBkaXJlY3Rpb24ubm9ybWFsaXplKCk7XG4gICAgICAgIHZhciBwMSA9IHRoaXMuX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbihjb2xsaWRlclBvaW50cywgZGlyZWN0aW9uKTtcbiAgICAgICAgdmFyIHAyID0gdGhpcy5fZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uKGNvbGxpZGVkUG9pbnRzLCBkaXJlY3Rpb24uc2NhbGUoLTEpKTtcbiAgICAgICAgLy8gcGVyZm9ybSB0aGUgTWlua293c2tpIERpZmZlcmVuY2VcbiAgICAgICAgdmFyIHAzID0gcDEuc3VidHJhY3QocDIpO1xuICAgICAgICAvLyBwMyBpcyBub3cgYSBwb2ludCBpbiBNaW5rb3dza2kgc3BhY2Ugb24gdGhlIGVkZ2Ugb2YgdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXG4gICAgICAgIHJldHVybiBwMztcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBzaW1wbGV4IGNvbnRhaW5zIHRoZSBvcmlnaW4uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXhUaGUgc2ltcGxleCBvciBmYWxzZSBpZiBubyBpbnRlcnNlY3Rpb24uXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbiB0byB0ZXN0LlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IENvbnRhaW5zIG9yIG5vdC5cbiAgICAgKi9cbiAgICBjb250YWluc09yaWdpbjogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMikge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zTGluZShzaW1wbGV4LCBkaXIpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMykge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zVHJpYW5nbGUoc2ltcGxleCwgZGlyKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAoc2ltcGxleC5sZW5ndGggPT09IDQpIHtcbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jb250YWluc1RldHJhaGVkcm9uKHNpbXBsZXgsIGRpcik7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBUaGUgR0pLIChHaWxiZXJ04oCTSm9obnNvbuKAk0tlZXJ0aGkpIGFsZ29yaXRobS5cbiAgICAgKiBDb21wdXRlcyBzdXBwb3J0IHBvaW50cyB0byBidWlsZCB0aGUgTWlua293c2t5IGRpZmZlcmVuY2UgYW5kXG4gICAgICogY3JlYXRlIGEgc2ltcGxleC4gVGhlIHBvaW50cyBvZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3RcbiAgICAgKiBtdXN0IGJlIGNvbnZleGUuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gVGhlIHNpbXBsZXggb3IgZmFsc2UgaWYgbm8gaW50ZXJzZWN0aW9uLlxuICAgICAqL1xuICAgIGNoZWNrOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcbiAgICAgICAgdmFyIGl0ID0gMDtcbiAgICAgICAgdmFyIGEsIHNpbXBsZXggPSBbXTtcblxuICAgICAgICB2YXIgY29sbGlkZXJDZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKGNvbGxpZGVyUG9pbnRzKTtcbiAgICAgICAgdmFyIGNvbGxpZGVkQ2VudGVyID0gdGhpcy5fZ2V0QmFyeWNlbnRlcihjb2xsaWRlZFBvaW50cyk7XG5cbiAgICAgICAgLy8gaW5pdGlhbCBkaXJlY3Rpb24gZnJvbSB0aGUgY2VudGVyIG9mIDFzdCBib2R5IHRvIHRoZSBjZW50ZXIgb2YgMm5kIGJvZHlcbiAgICAgICAgdmFyIGRpciA9IGNvbGxpZGVyQ2VudGVyLnN1YnRyYWN0KGNvbGxpZGVkQ2VudGVyKS5ub3JtYWxpemUoKTtcblxuICAgICAgICAvLyBpZiBpbml0aWFsIGRpcmVjdGlvbiBpcyB6ZXJvIOKAkyBzZXQgaXQgdG8gYW55IGFyYml0cmFyeSBheGlzICh3ZSBjaG9vc2UgWClcbiAgICAgICAgaWYgKGRpci5sZW5ndGhTcXVhcmVkKCkgPCB0aGlzLkVQU0lMT04pIHtcbiAgICAgICAgICAgIGRpci54ID0gMTtcbiAgICAgICAgfVxuXG4gICAgICAgIC8vIHNldCB0aGUgZmlyc3Qgc3VwcG9ydCBhcyBpbml0aWFsIHBvaW50IG9mIHRoZSBuZXcgc2ltcGxleFxuICAgICAgICBhID0gc2ltcGxleFswXSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XG5cbiAgICAgICAgaWYgKGEuY29uc3RydWN0b3IuRG90KGEsIGRpcikgPD0gMCkge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgZGlyLnNjYWxlSW5QbGFjZSgtMSwgZGlyKTsgLy8gd2Ugd2lsbCBiZSBzZWFyY2hpbmcgaW4gdGhlIG9wcG9zaXRlIGRpcmVjdGlvbiBuZXh0XG5cbiAgICAgICAgdmFyIG1heCA9IGNvbGxpZGVkUG9pbnRzLmxlbmd0aCAqIGNvbGxpZGVyUG9pbnRzLmxlbmd0aDtcbiAgICAgICAgd2hpbGUgKGl0IDwgbWF4KSB7XG4gICAgICAgICAgICBpZiAoZGlyLmxlbmd0aFNxdWFyZWQoKSA9PT0gMCAmJiBzaW1wbGV4Lmxlbmd0aCA+PSAyKSB7XG4gICAgICAgICAgICAgICAgLy8gR2V0IHBlcnBlbmRpY3VsYXIgZGlyZWN0aW9uIHRvIGxhc3Qgc2ltcGxleFxuICAgICAgICAgICAgICAgIGRpciA9IHNpbXBsZXhbc2ltcGxleC5sZW5ndGggLSAxXS5zdWJ0cmFjdChzaW1wbGV4W3NpbXBsZXgubGVuZ3RoIC0gMl0pO1xuICAgICAgICAgICAgICAgIHZhciB0bXAgPSBkaXIueTtcbiAgICAgICAgICAgICAgICBkaXIueSA9IC1kaXIueDtcbiAgICAgICAgICAgICAgICBkaXIueCA9IHRtcDtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgYSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XG5cbiAgICAgICAgICAgIC8vIG1ha2Ugc3VyZSB0aGF0IHRoZSBsYXN0IHBvaW50IHdlIGFkZGVkIGFjdHVhbGx5IHBhc3NlZCB0aGUgb3JpZ2luXG4gICAgICAgICAgICBpZiAoYS5jb25zdHJ1Y3Rvci5Eb3QoYSwgZGlyKSA8PSAwKSB7XG4gICAgICAgICAgICAgICAgLy8gaWYgdGhlIHBvaW50IGFkZGVkIGxhc3Qgd2FzIG5vdCBwYXN0IHRoZSBvcmlnaW4gaW4gdGhlIGRpcmVjdGlvbiBvZiBkXG4gICAgICAgICAgICAgICAgLy8gdGhlbiB0aGUgTWlua293c2tpIFN1bSBjYW5ub3QgcG9zc2libHkgY29udGFpbiB0aGUgb3JpZ2luIHNpbmNlXG4gICAgICAgICAgICAgICAgLy8gdGhlIGxhc3QgcG9pbnQgYWRkZWQgaXMgb24gdGhlIGVkZ2Ugb2YgdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXG4gICAgICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICBzaW1wbGV4LnB1c2goYSk7XG4gICAgICAgICAgICAvLyBvdGhlcndpc2Ugd2UgbmVlZCB0byBkZXRlcm1pbmUgaWYgdGhlIG9yaWdpbiBpcyBpblxuICAgICAgICAgICAgLy8gdGhlIGN1cnJlbnQgc2ltcGxleFxuXG4gICAgICAgICAgICBpZiAodGhpcy5jb250YWluc09yaWdpbihzaW1wbGV4LCBkaXIpKSB7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHNpbXBsZXg7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpdCsrO1xuICAgICAgICB9XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEZpbmRzIHRoZSByZXNwb25zZSB3aXRoIHRoZSBzaW1wbGV4IChlZGdlcykgb2YgdGhlIGdqayBhbGdvcml0aG0uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgZmluZFJlc3BvbnNlV2l0aEVkZ2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCkge1xuICAgICAgICB2YXIgZWRnZSA9IHRoaXMuX2dldE5lYXJlc3RFZGdlKHNpbXBsZXgpO1xuICAgICAgICB2YXIgc3VwID0gdGhpcy5zdXBwb3J0KGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgZWRnZS5ub3JtYWwpOyAvL2dldCBzdXBwb3J0IHBvaW50IGluIGRpcmVjdGlvbiBvZiBlZGdlJ3Mgbm9ybWFsXG4gICAgICAgIHZhciBkID0gTWF0aC5hYnMoc3VwLmNvbnN0cnVjdG9yLkRvdChzdXAsIGVkZ2Uubm9ybWFsKSk7XG5cbiAgICAgICAgaWYgKGQgLSBlZGdlLmRpc3RhbmNlIDw9IHRoaXMuRVBTSUxPTikge1xuICAgICAgICAgICAgcmV0dXJuIGVkZ2Uubm9ybWFsLnNjYWxlKGVkZ2UuZGlzdGFuY2UpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoZWRnZS5pbmRleCwgMCwgc3VwKTtcbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEZpbmRzIHRoZSByZXNwb25zZSB3aXRoIHRoZSBwb2x5dG9wZSBkb25lIHdpdGggdGhlIHNpbXBsZXggb2YgdGhlIGdqayBhbGdvcml0aG0uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtUcmlhbmdsZVtdfSBwb2x5dG9wZSBUaGUgcG9seXRvcGUgZG9uZSB3aXRoIHRoZSBzaW1wbGV4LlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgZmluZFJlc3BvbnNlV2l0aFRyaWFuZ2xlOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHBvbHl0b3BlKSB7XG5cbiAgICAgICAgaWYgKHBvbHl0b3BlLmxlbmd0aCA9PT0gMCkge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIG5lYXJlc3QgPSB0aGlzLl9nZXROZWFyZXN0VHJpYW5nbGUocG9seXRvcGUpO1xuICAgICAgICB2YXIgdHJpYW5nbGUgPSBwb2x5dG9wZVtuZWFyZXN0LmluZGV4XTtcblxuICAgICAgICB2YXIgc3VwID0gdGhpcy5zdXBwb3J0KGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgdHJpYW5nbGUubik7XG5cbiAgICAgICAgdmFyIGQgPSBNYXRoLmFicyhzdXAuY29uc3RydWN0b3IuRG90KHN1cCwgdHJpYW5nbGUubikpO1xuXG4gICAgICAgIGlmICgoZCAtIG5lYXJlc3QuZGlzdGFuY2UgPD0gdGhpcy5FUFNJTE9OKSkge1xuICAgICAgICAgICAgcmV0dXJuIHRyaWFuZ2xlLm4uc2NhbGUobmVhcmVzdC5kaXN0YW5jZSk7XG4gICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICB2YXIgZWRnZXMgPSBbXTtcbiAgICAgICAgICAgIGZvciAodmFyIGkgPSBwb2x5dG9wZS5sZW5ndGggLSAxOyBpID49IDA7IGktLSkge1xuICAgICAgICAgICAgICAgIHRyaWFuZ2xlID0gcG9seXRvcGVbaV07XG4gICAgICAgICAgICAgICAgLy8gY2FuIHRoaXMgZmFjZSBiZSAnc2VlbicgYnkgZW50cnlfY3VyX3N1cHBvcnQ/XG4gICAgICAgICAgICAgICAgaWYgKHRyaWFuZ2xlLm4uY29uc3RydWN0b3IuRG90KHRyaWFuZ2xlLm4sIHN1cC5zdWJ0cmFjdChwb2x5dG9wZVtpXS5hKSkgPiAwKSB7XG4gICAgICAgICAgICAgICAgICAgIHRoaXMuX2FkZEVkZ2UoZWRnZXMsIHtcbiAgICAgICAgICAgICAgICAgICAgICAgIGE6IHRyaWFuZ2xlLmEsXG4gICAgICAgICAgICAgICAgICAgICAgICBiOiB0cmlhbmdsZS5iXG4gICAgICAgICAgICAgICAgICAgIH0pO1xuICAgICAgICAgICAgICAgICAgICB0aGlzLl9hZGRFZGdlKGVkZ2VzLCB7XG4gICAgICAgICAgICAgICAgICAgICAgICBhOiB0cmlhbmdsZS5iLFxuICAgICAgICAgICAgICAgICAgICAgICAgYjogdHJpYW5nbGUuY1xuICAgICAgICAgICAgICAgICAgICB9KTtcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5fYWRkRWRnZShlZGdlcywge1xuICAgICAgICAgICAgICAgICAgICAgICAgYTogdHJpYW5nbGUuYyxcbiAgICAgICAgICAgICAgICAgICAgICAgIGI6IHRyaWFuZ2xlLmFcbiAgICAgICAgICAgICAgICAgICAgfSk7XG4gICAgICAgICAgICAgICAgICAgIHBvbHl0b3BlLnNwbGljZShpLCAxKTtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIC8vIGNyZWF0ZSBuZXcgdHJpYW5nbGVzIGZyb20gdGhlIGVkZ2VzIGluIHRoZSBlZGdlIGxpc3RcbiAgICAgICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZWRnZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgICAgICB0cmlhbmdsZSA9IG5ldyBUcmlhbmdsZShzdXAsIGVkZ2VzW2ldLmEsIGVkZ2VzW2ldLmIpO1xuICAgICAgICAgICAgICAgIGlmICh0cmlhbmdsZS5uLmxlbmd0aCgpICE9PSAwKSB7XG4gICAgICAgICAgICAgICAgICAgIHBvbHl0b3BlLnB1c2godHJpYW5nbGUpO1xuICAgICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyB0aGUgcmVzcG9uc2Ugb2YgdGhlIHBlbmV0cmF0aW9uIHZlY3RvciB3aXRoIHRoZSBzaW1wbGV4LlxuICAgICAqXG4gICAgICogQG1ldGhvZCBnZXRSZXNwb25zZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleCBvZiB0aGUgTWlua293c2t5IGRpZmZlcmVuY2UuXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHBlbmV0cmF0aW9uIHZlY3Rvci5cbiAgICAgKi9cbiAgICBnZXRSZXNwb25zZTogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4KSB7XG4gICAgICAgIHZhciBpdCA9IDAsXG4gICAgICAgICAgICByZXNwb25zZTtcbiAgICAgICAgdmFyIHBvbHl0b3BlID0gc2ltcGxleFswXS56ICE9PSB1bmRlZmluZWQgPyBbbmV3IFRyaWFuZ2xlKHNpbXBsZXhbMF0sIHNpbXBsZXhbMV0sIHNpbXBsZXhbMl0pLFxuICAgICAgICAgICAgbmV3IFRyaWFuZ2xlKHNpbXBsZXhbMF0sIHNpbXBsZXhbMl0sIHNpbXBsZXhbM10pLFxuICAgICAgICAgICAgbmV3IFRyaWFuZ2xlKHNpbXBsZXhbMF0sIHNpbXBsZXhbM10sIHNpbXBsZXhbMV0pLFxuICAgICAgICAgICAgbmV3IFRyaWFuZ2xlKHNpbXBsZXhbMV0sIHNpbXBsZXhbM10sIHNpbXBsZXhbMl0pXG4gICAgICAgIF0gOiBudWxsO1xuXG4gICAgICAgIHZhciBtYXggPSBjb2xsaWRlZFBvaW50cy5sZW5ndGggKiBjb2xsaWRlclBvaW50cy5sZW5ndGg7XG4gICAgICAgIHdoaWxlIChpdCA8IG1heCkge1xuICAgICAgICAgICAgaWYgKHNpbXBsZXhbMF0ueiA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhFZGdlKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCk7XG4gICAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgICAgIHJlc3BvbnNlID0gdGhpcy5maW5kUmVzcG9uc2VXaXRoVHJpYW5nbGUoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBwb2x5dG9wZSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAocmVzcG9uc2UpIHtcbiAgICAgICAgICAgICAgICB2YXIgbm9ybSA9IHJlc3BvbnNlLmNsb25lKCkubm9ybWFsaXplKCkuc2NhbGVJblBsYWNlKHRoaXMuRVBTSUxPTik7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHJlc3BvbnNlLmFkZFRvUmVmKG5vcm0sIHJlc3BvbnNlKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGl0Kys7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIGNvbGxpZGVyIGFuZCB0aGUgY29sbGlkZWQgb2JqZWN0IGFyZSBpbnRlcnNlY3RpbmcuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGlzSW50ZXJzZWN0aW5nXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IElzIGludGVyc2VjdGluZyBvciBub3QuXG4gICAgICovXG4gICAgaXNJbnRlcnNlY3Rpbmc6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cykge1xuICAgICAgICByZXR1cm4gISF0aGlzLmNoZWNrKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cyk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3QgYXJlIGludGVyc2VjdGluZ1xuICAgICAqIGFuZCBnaXZlIHRoZSByZXNwb25zZSB0byBiZSBvdXQgb2YgdGhlIG9iamVjdC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgaW50ZXJzZWN0XG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgaW50ZXJzZWN0OiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcbiAgICAgICAgdmFyIHNpbXBsZXggPSB0aGlzLmNoZWNrKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cyk7XG5cbiAgICAgICAgLy90aGlzLmN1YmUgPSB0aGlzLmN1YmUgfHwgW107XG4gICAgICAgIGlmIChzaW1wbGV4KSB7XG4gICAgICAgICAgICByZXR1cm4gdGhpcy5nZXRSZXNwb25zZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgpO1xuICAgICAgICB9XG5cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH1cbn07XG5cbm1vZHVsZS5leHBvcnRzID0gQ29sbGlzaW9uR2prRXBhO1xuIl19
