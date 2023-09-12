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
     * @param {number[]} [except] Exceptions: edge indices to ignore
     * @return {Object} Informations about the nearest edge (distance, index and normal).
     */
    _getNearestEdge: function(simplex, except) {
        if (except === undefined) except = [];

        var distance = Infinity,
            index, normal;

        for (var i = 0; i < simplex.length; i++) {
            var j = (i + 1) % simplex.length;
            if (except.includes(j)) continue; // ignore this edge

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
     * @param {number[]} [except] Exceptions: triangle indices to ignore
     * @return {Object} Informations about the nearest edge (distance and index).
     */
    _getNearestTriangle: function(polytope, except) {
        if (except === undefined) except = [];

        var distance = Infinity,
            index;

        for (var i = 0; i < polytope.length; i++) {
            if (except.includes(i)) continue;

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
     * @method containsOrigin
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
     * @method check
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
     * @param {number[]} [except] Exceptions: edge indices to ignore
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    findResponseWithEdge: function(colliderPoints, collidedPoints, simplex, except) {
        if (except === undefined) except = [];

        var edge = this._getNearestEdge(simplex, except);
        if (edge.index === undefined) return false;

        var sup = this.support(colliderPoints, collidedPoints, edge.normal); //get support point in direction of edge's normal
        var d = Math.abs(sup.constructor.Dot(sup, edge.normal));

        if (d - edge.distance <= this.EPSILON) {
            except.push(edge.index);
            return edge.normal.scale(edge.distance);
        }
        // add new support point in simplex
        simplex.splice(edge.index, 0, sup);
        // update indices of edges exceptions
        this._updateExceptionsByInsert(except, edge.index, simplex);

        return false;
    },

    /**
     * Finds the response with the polytope done with the simplex of the gjk algorithm.
     *
     * @method findResponseWithTriangle
     * @param {BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @param {Triangle[]} polytope The polytope done with the simplex.
     * @param {number[]} [except] Exceptions: triangle indices to ignore
     * @return {BABYLON.Vector2|BABYLON.Vector3} The penetration vector.
     */
    findResponseWithTriangle: function(colliderPoints, collidedPoints, polytope, except) {
        if (except === undefined) except = [];

        if (polytope.length === 0) {
            return false;
        }

        var nearest = this._getNearestTriangle(polytope, except);
        if (nearest.index === undefined) return false;

        var triangle = polytope[nearest.index];

        var sup = this.support(colliderPoints, collidedPoints, triangle.n);

        var d = Math.abs(sup.constructor.Dot(sup, triangle.n));

        if ((d - nearest.distance <= this.EPSILON)) {
            except.push(nearest.index);
            return triangle.n.scale(nearest.distance);
        }

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
                // remove a triangle
                polytope.splice(i, 1);
                // update exceptions with removed triangle index
                this._removeException(except, i);
            }
        }

        // create new triangles from the edges in the edge list
        for (var i = 0; i < edges.length; i++) {
            triangle = new Triangle(sup, edges[i].a, edges[i].b);
            if (triangle.n.length() !== 0) {
                polytope.push(triangle);
            }
        }

        return false;
    },

    /**
     * Update exception from insertion of new edge
     * @param {number[]} except 
     * @param {number} edgeIndex inserted edge index
     * @param {BABYLON.Vector2[]} simplex 
     */
    _updateExceptionsByInsert: function(except, edgeIndex, simplex) {
        for (var i = 0; i < except.length; i++) {
            if (except[i] >= edgeIndex) {
                except[i] = (except[i] + 1) % simplex.length;
            }
        }
    },

    /**
     * Update exceptions when an index has been removed
     * @param {number[]} except 
     * @param {number} indexToRemove 
     */
    _removeException: function(except, indexToRemove) {
        for (var i = except.length - 1; i >= 0; i--) {
            if(except[i] === indexToRemove) {
                except.splice(i, 1);
            } else if (except[i] > indexToRemove) {
                except[i]--;
            }
        }
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
        var responses = this.getResponses(colliderPoints, collidedPoints, simplex, { maxResponses: 1 });
        if (!responses) return responses;
        if (responses[0]) return responses[0];
        return false;
    },

    getResponses: function(colliderPoints, collidedPoints, simplex, options) {
        if (options === undefined) options = {};
        options = Object.assign({ maxResponses: -1 }, options);

        var it = 0,
            response;
        var polytope = simplex[0].z !== undefined ? [new Triangle(simplex[0], simplex[1], simplex[2]),
            new Triangle(simplex[0], simplex[2], simplex[3]),
            new Triangle(simplex[0], simplex[3], simplex[1]),
            new Triangle(simplex[1], simplex[3], simplex[2])
        ] : null;

        var max = collidedPoints.length * colliderPoints.length;
        var except = [];
        var responses = [];
        while (it < max && responses.length !== options.maxResponses) {
            if (simplex[0].z === undefined) {
                response = this.findResponseWithEdge(colliderPoints, collidedPoints, simplex, except);
            } else {
                response = this.findResponseWithTriangle(colliderPoints, collidedPoints, polytope, except);
            }
            if (response) {
                var norm = response.clone().normalize().scaleInPlace(this.EPSILON);
                var reponseWithOffset = response.addToRef(norm, response);
                this._addResponse(responses, reponseWithOffset);
            }
            it++;
        }
        return responses;
    },

    /**
     * Add response to responses if not already present
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} responses
     * @param {BABYLON.Vector2|BABYLON.Vector3} response
     */
    _addResponse: function(responses, response) {
        for (var i = 0; i < responses.length; i++) {
            if (responses[i].subtract(response).lengthSquared() < this.EPSILON) {
                return;
            }
        }

        responses.push(response);
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
    },

    /**
     * Checks if the collider and the collided object are intersecting
     * and give the multiple responses to be out of the object.
     *
     * @method intersectMultiple
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} colliderPoints The convexe collider object.
     * @param {BABYLON.Vector2|BABYLON.Vector3[]} collidedPoints The convexe collided object.
     * @param {Object} [options]
     * @return {BABYLON.Vector2|BABYLON.Vector3[]} The penetration vectors.
     */
    intersectMultiple: function(colliderPoints, collidedPoints, options) {
        var simplex = this.check(colliderPoints, collidedPoints);

        if (simplex) {
            return this.getResponses(colliderPoints, collidedPoints, simplex, options);
        }

        return false;
    }
};

module.exports = CollisionGjkEpa;

},{}]},{},[1])(1)
});

//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJjb2xsaXNpb24tZ2prLWVwYS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBIiwiZmlsZSI6ImdlbmVyYXRlZC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzQ29udGVudCI6WyIoZnVuY3Rpb24oKXtmdW5jdGlvbiByKGUsbix0KXtmdW5jdGlvbiBvKGksZil7aWYoIW5baV0pe2lmKCFlW2ldKXt2YXIgYz1cImZ1bmN0aW9uXCI9PXR5cGVvZiByZXF1aXJlJiZyZXF1aXJlO2lmKCFmJiZjKXJldHVybiBjKGksITApO2lmKHUpcmV0dXJuIHUoaSwhMCk7dmFyIGE9bmV3IEVycm9yKFwiQ2Fubm90IGZpbmQgbW9kdWxlICdcIitpK1wiJ1wiKTt0aHJvdyBhLmNvZGU9XCJNT0RVTEVfTk9UX0ZPVU5EXCIsYX12YXIgcD1uW2ldPXtleHBvcnRzOnt9fTtlW2ldWzBdLmNhbGwocC5leHBvcnRzLGZ1bmN0aW9uKHIpe3ZhciBuPWVbaV1bMV1bcl07cmV0dXJuIG8obnx8cil9LHAscC5leHBvcnRzLHIsZSxuLHQpfXJldHVybiBuW2ldLmV4cG9ydHN9Zm9yKHZhciB1PVwiZnVuY3Rpb25cIj09dHlwZW9mIHJlcXVpcmUmJnJlcXVpcmUsaT0wO2k8dC5sZW5ndGg7aSsrKW8odFtpXSk7cmV0dXJuIG99cmV0dXJuIHJ9KSgpIiwiJ3VzZSBzdHJpY3QnO1xuXG4vKipcbiAqIENsYXNzIHRvIGhlbHAgaW4gdGhlIGNvbGxpc2lvbiBpbiAyRCBhbmQgM0QuXG4gKiBUbyB3b3JrcyB0aGUgYWxnb3JpdGhtIG5lZWRzIHR3byBjb252ZXhlIHBvaW50IGNsb3VkXG4gKiBsaWtlIGJvdW5kaW5nIGJveCBvciBzbXRoZyBsaWtlIHRoaXMuXG4gKiBUaGUgZnVuY3Rpb24gZnVuY3Rpb25zIGludGVyc2VjdCBhbmQgaXNJbnRlcnNlY3RpbmdcbiAqIGhlbHBzIHRvIGhhdmUgaW5mb3JtYXRpb25zIGFib3V0IHRoZSBjb2xsaXNpb24gYmV0d2VlbiB0d28gb2JqZWN0LlxuICpcbiAqIEBjbGFzcyB3bnAuaGVscGVycy5Db2xsaXNpb25HamtFcGFcbiAqIEBjb25zdHJ1Y3RvclxuICovXG52YXIgVHJpYW5nbGUgPSBmdW5jdGlvbihhLCBiLCBjKSB7XG4gICAgICB0aGlzLmEgPSBhO1xuICAgICAgdGhpcy5iID0gYjtcbiAgICAgIHRoaXMuYyA9IGM7XG4gICAgICB0aGlzLm4gPSBhLmNvbnN0cnVjdG9yLkNyb3NzKGIuc3VidHJhY3QoYSksIGMuc3VidHJhY3QoYSkpLm5vcm1hbGl6ZSgpO1xuICB9O1xuXG5cbnZhciBDb2xsaXNpb25HamtFcGEgPSB7XG5cbiAgICBFUFNJTE9OOiAwLjAwMDAwMSxcblxuICAgIC8qKlxuICAgICAqIE1ldGhvZCB0byBnZXQgYSBub3JtYWwgb2YgMyBwb2ludHMgaW4gMkQgYW5kIDNELlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0Tm9ybWFsXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGFcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGJcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGNcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgbm9ybWFsLlxuICAgICAqL1xuICAgIF9nZXROb3JtYWw6IGZ1bmN0aW9uKGEsIGIsIGMpIHtcbiAgICAgICAgdmFyIERvdCA9IGEuY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgYWMgPSBEb3QoYSwgYyk7IC8vIHBlcmZvcm0gYURvdChjKVxuICAgICAgICB2YXIgYmMgPSBEb3QoYiwgYyk7IC8vIHBlcmZvcm0gYkRvdChjKVxuXG4gICAgICAgIC8vIHBlcmZvcm0gYiAqIGEuRG90KGMpIC0gYSAqIGIuRG90KGMpXG4gICAgICAgIHZhciByID0gYi5zY2FsZShhYykuc3VidHJhY3QoYS5zY2FsZShiYykpO1xuICAgICAgICBpZiAoci5sZW5ndGhTcXVhcmVkKCkgPCB0aGlzLkVQU0lMT04gKiB0aGlzLkVQU0lMT04pIHtcbiAgICAgICAgICAgIHJldHVybiByLnNjYWxlKDApO1xuICAgICAgICB9XG4gICAgICAgIHJldHVybiByLm5vcm1hbGl6ZSgpO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBiYXJ5Y2VudGVyIG9mIGEgY2xvdWQgcG9pbnRzLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0QmFyeWNlbnRlclxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHZlcnRpY2VzIHRoZSBjbG91ZCBwb2ludHNcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgYmFyeWNlbnRlci5cbiAgICAgKi9cbiAgICBfZ2V0QmFyeWNlbnRlcjogZnVuY3Rpb24odmVydGljZXMpIHtcbiAgICAgICAgdmFyIGF2ZyA9IHZlcnRpY2VzWzBdLmNvbnN0cnVjdG9yLlplcm8oKTtcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCB2ZXJ0aWNlcy5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgYXZnLmFkZFRvUmVmKHZlcnRpY2VzW2ldLCBhdmcpO1xuICAgICAgICB9XG4gICAgICAgIGF2Zy5zY2FsZUluUGxhY2UoMSAvIHZlcnRpY2VzLmxlbmd0aCwgYXZnKTtcbiAgICAgICAgcmV0dXJuIGF2ZztcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogR2V0cyB0aGUgZmFydGhlc3QgcG9pbnQgb2YgYSBjbG91ZCBwb2ludHMuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9nZXRGYXJ0aGVzdFBvaW50SW5EaXJlY3Rpb25cbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSB2ZXJ0aWNlcyB0aGUgY2xvdWQgcG9pbnRzLlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZCBUaGUgZGlyZWN0aW9uIHRvIHNlYXJjaC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgYmFyeWNlbnRlci5cbiAgICAgKi9cbiAgICBfZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uOiBmdW5jdGlvbih2ZXJ0aWNlcywgZCkge1xuICAgICAgICB2YXIgRG90ID0gdmVydGljZXNbMF0uY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgbWF4UHJvZHVjdCA9IERvdCh2ZXJ0aWNlc1swXSwgZCk7XG4gICAgICAgIHZhciBpbmRleCA9IDA7XG4gICAgICAgIGZvciAodmFyIGkgPSAxOyBpIDwgdmVydGljZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIHZhciBwcm9kdWN0ID0gRG90KHZlcnRpY2VzW2ldLCBkKTtcbiAgICAgICAgICAgIGlmIChwcm9kdWN0ID4gbWF4UHJvZHVjdCkge1xuICAgICAgICAgICAgICAgIG1heFByb2R1Y3QgPSBwcm9kdWN0O1xuICAgICAgICAgICAgICAgIGluZGV4ID0gaTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gdmVydGljZXNbaW5kZXhdO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBuZWFyZXN0IGVkZ2Ugb2YgdGhlIHNpbXBsZXguXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9nZXROZWFyZXN0RWRnZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtudW1iZXJbXX0gW2V4Y2VwdF0gRXhjZXB0aW9uczogZWRnZSBpbmRpY2VzIHRvIGlnbm9yZVxuICAgICAqIEByZXR1cm4ge09iamVjdH0gSW5mb3JtYXRpb25zIGFib3V0IHRoZSBuZWFyZXN0IGVkZ2UgKGRpc3RhbmNlLCBpbmRleCBhbmQgbm9ybWFsKS5cbiAgICAgKi9cbiAgICBfZ2V0TmVhcmVzdEVkZ2U6IGZ1bmN0aW9uKHNpbXBsZXgsIGV4Y2VwdCkge1xuICAgICAgICBpZiAoZXhjZXB0ID09PSB1bmRlZmluZWQpIGV4Y2VwdCA9IFtdO1xuXG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxuICAgICAgICAgICAgaW5kZXgsIG5vcm1hbDtcblxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHNpbXBsZXgubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIHZhciBqID0gKGkgKyAxKSAlIHNpbXBsZXgubGVuZ3RoO1xuICAgICAgICAgICAgaWYgKGV4Y2VwdC5pbmNsdWRlcyhqKSkgY29udGludWU7IC8vIGlnbm9yZSB0aGlzIGVkZ2VcblxuICAgICAgICAgICAgdmFyIHYxID0gc2ltcGxleFtpXTtcbiAgICAgICAgICAgIHZhciB2MiA9IHNpbXBsZXhbal07XG5cbiAgICAgICAgICAgIHZhciBlZGdlID0gdjIuc3VidHJhY3QodjEpO1xuICAgICAgICAgICAgaWYgKGVkZ2UubGVuZ3RoU3F1YXJlZCgpID09PSAwKSB7XG4gICAgICAgICAgICAgICAgY29udGludWU7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIHZhciBvcmlnaW5Ub3YxID0gdjE7XG5cbiAgICAgICAgICAgIHZhciBuID0gdGhpcy5fZ2V0Tm9ybWFsKGVkZ2UsIG9yaWdpblRvdjEsIGVkZ2UpO1xuXG4gICAgICAgICAgICBpZiAobi5sZW5ndGhTcXVhcmVkKCkgPT09IDApIHtcbiAgICAgICAgICAgICAgICAvLyBPcmlnaW4gaXMgb24gdGhlIGVkZ2VcbiAgICAgICAgICAgICAgICBuLnkgPSAtZWRnZS54O1xuICAgICAgICAgICAgICAgIG4ueCA9IGVkZ2UueTtcbiAgICAgICAgICAgICAgICAvLyBub3JtYWwgc2hvdWxkIGdvIG91dHNpZGUgdGhlIHNpbXBsZXhcbiAgICAgICAgICAgICAgICB2YXIgY2VudGVyID0gdGhpcy5fZ2V0QmFyeWNlbnRlcihzaW1wbGV4KVxuICAgICAgICAgICAgICAgIHZhciBjZW50ZXJUb3YxID0gdjEuc3VidHJhY3QoY2VudGVyKTtcbiAgICAgICAgICAgICAgICBpZiAobi5jb25zdHJ1Y3Rvci5Eb3QobiwgY2VudGVyVG92MSkgPCAwKSB7XG4gICAgICAgICAgICAgICAgICAgIG4ueSA9IC1uLnk7XG4gICAgICAgICAgICAgICAgICAgIG4ueCA9IC1uLng7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICAvL24gPSBuLnNjYWxlKDEgLyBuLmxlbmd0aCgpKTsgLy9ub3JtYWxpemVcbiAgICAgICAgICAgIHZhciBkaXN0ID0gTWF0aC5hYnMobi5jb25zdHJ1Y3Rvci5Eb3QobiwgdjEpKTsgLy9kaXN0YW5jZSBmcm9tIG9yaWdpbiB0byBlZGdlXG5cbiAgICAgICAgICAgIGlmIChkaXN0IDwgZGlzdGFuY2UpIHtcbiAgICAgICAgICAgICAgICBkaXN0YW5jZSA9IGRpc3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBqO1xuICAgICAgICAgICAgICAgIG5vcm1hbCA9IG47XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4ge1xuICAgICAgICAgICAgZGlzdGFuY2U6IGRpc3RhbmNlLFxuICAgICAgICAgICAgaW5kZXg6IGluZGV4LFxuICAgICAgICAgICAgbm9ybWFsOiBub3JtYWxcbiAgICAgICAgfTtcblxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBuZWFyZXN0IFRyaWFuZ2xlIG9mIHRoZSBwb2x5dG9wZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RUcmlhbmdsZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlRyaWFuZ2xlW119IHBvbHl0b3BlIFRoZSBwb2x5dG9wZS5cbiAgICAgKiBAcGFyYW0ge251bWJlcltdfSBbZXhjZXB0XSBFeGNlcHRpb25zOiB0cmlhbmdsZSBpbmRpY2VzIHRvIGlnbm9yZVxuICAgICAqIEByZXR1cm4ge09iamVjdH0gSW5mb3JtYXRpb25zIGFib3V0IHRoZSBuZWFyZXN0IGVkZ2UgKGRpc3RhbmNlIGFuZCBpbmRleCkuXG4gICAgICovXG4gICAgX2dldE5lYXJlc3RUcmlhbmdsZTogZnVuY3Rpb24ocG9seXRvcGUsIGV4Y2VwdCkge1xuICAgICAgICBpZiAoZXhjZXB0ID09PSB1bmRlZmluZWQpIGV4Y2VwdCA9IFtdO1xuXG4gICAgICAgIHZhciBkaXN0YW5jZSA9IEluZmluaXR5LFxuICAgICAgICAgICAgaW5kZXg7XG5cbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwb2x5dG9wZS5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgaWYgKGV4Y2VwdC5pbmNsdWRlcyhpKSkgY29udGludWU7XG5cbiAgICAgICAgICAgIHZhciB0cmlhbmdsZSA9IHBvbHl0b3BlW2ldO1xuICAgICAgICAgICAgdmFyIGRpc3QgPSBNYXRoLmFicyh0cmlhbmdsZS5uLmNvbnN0cnVjdG9yLkRvdCh0cmlhbmdsZS5uLCB0cmlhbmdsZS5hKSk7XG5cbiAgICAgICAgICAgIGlmIChkaXN0IDwgZGlzdGFuY2UpIHtcbiAgICAgICAgICAgICAgICBkaXN0YW5jZSA9IGRpc3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4ge1xuICAgICAgICAgICAgZGlzdGFuY2U6IGRpc3RhbmNlLFxuICAgICAgICAgICAgaW5kZXg6IGluZGV4XG4gICAgICAgIH07XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgbGluZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2NvbnRhaW5zTGluZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBGYWxzZSBpbiBhbnkgY2FzZSBiZWNhdXNlIHRoZSBhbGdvcml0aG0ganVzdCBiZWdpbi5cbiAgICAgKi9cbiAgICBfY29udGFpbnNMaW5lOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcbiAgICAgICAgdmFyIGEgPSBzaW1wbGV4WzFdO1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMF07XG4gICAgICAgIHZhciBhYiA9IGIuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xuICAgICAgICBpZiAoYWIubGVuZ3RoU3F1YXJlZCgpICE9PSAwKSB7XG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbShhbyk7XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgdHJpYW5nbGUuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc1RyaWFuZ2xlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uLlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IElmIGluIDJEIGNhc2UsIG1heSByZXR1cm4gdHJ1ZSBpZiB0aGUgb3JpZ2luIGlzIGluIHRoZSB0cmlhbmdsZS5cbiAgICAgKi9cbiAgICBfY29udGFpbnNUcmlhbmdsZTogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIHZhciBhID0gc2ltcGxleFsyXTtcbiAgICAgICAgdmFyIERvdCA9IHNpbXBsZXhbMl0uY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMV07XG4gICAgICAgIHZhciBjID0gc2ltcGxleFswXTtcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFjID0gYy5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFvID0gYS5zY2FsZSgtMSk7XG5cbiAgICAgICAgdmFyIGFicCA9IHRoaXMuX2dldE5vcm1hbChhYywgYWIsIGFiKTtcbiAgICAgICAgdmFyIGFjcCA9IHRoaXMuX2dldE5vcm1hbChhYiwgYWMsIGFjKTtcbiAgICAgICAgaWYgKERvdChhYnAsIGFvKSA+IDApIHtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDEpOyAvLyByZW1vdmUgQ1xuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFicCk7XG4gICAgICAgIH0gZWxzZSBpZiAoRG90KGFjcCwgYW8pID4gMCkge1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMSwgMSk7IC8vIHJlbW92ZSBCXG4gICAgICAgICAgICBkaXIuY29weUZyb20oYWNwKTtcbiAgICAgICAgfSBlbHNlIHtcbiAgICAgICAgICAgIGlmIChkaXIueiA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgICAgIHZhciBhYmMgPSBzaW1wbGV4WzJdLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYyk7XG4gICAgICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFiYyk7XG4gICAgICAgICAgICAgICAgaWYgKERvdChhYmMsIGFvKSA8PSAwKSB7XG4gICAgICAgICAgICAgICAgICAgIC8vdXBzaWRlIGRvd24gdGV0cmFoZWRyb25cbiAgICAgICAgICAgICAgICAgICAgc2ltcGxleFswXSA9IGI7XG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBjO1xuICAgICAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gYTtcbiAgICAgICAgICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFiYy5zY2FsZSgtMSkpO1xuICAgICAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIG9yaWdpbiBpcyBpbiBhIHRldHJhaGVkcm9uLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfY29udGFpbnNUZXRyYWhlZHJvblxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBSZXR1cm4gdHJ1ZSBpZiB0aGUgb3JpZ2luIGlzIGluIHRoZSB0ZXRyYWhlZHJvbi5cbiAgICAgKi9cbiAgICBfY29udGFpbnNUZXRyYWhlZHJvbjogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIHZhciBhID0gc2ltcGxleFszXTtcbiAgICAgICAgdmFyIERvdCA9IGEuY29uc3RydWN0b3IuRG90O1xuICAgICAgICB2YXIgQ3Jvc3MgPSBhLmNvbnN0cnVjdG9yLkNyb3NzO1xuICAgICAgICB2YXIgYiA9IHNpbXBsZXhbMl07XG4gICAgICAgIHZhciBjID0gc2ltcGxleFsxXTtcbiAgICAgICAgdmFyIGQgPSBzaW1wbGV4WzBdO1xuICAgICAgICB2YXIgYWIgPSBiLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYWMgPSBjLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYWQgPSBkLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYW8gPSBhLnNjYWxlKC0xKTtcblxuICAgICAgICB2YXIgYWJjID0gQ3Jvc3MoYWIsIGFjKTtcbiAgICAgICAgdmFyIGFjZCA9IENyb3NzKGFjLCBhZCk7XG4gICAgICAgIHZhciBhZGIgPSBDcm9zcyhhZCwgYWIpO1xuXG4gICAgICAgIHZhciBhYmNUZXN0ID0gMHgxLFxuICAgICAgICAgICAgYWNkVGVzdCA9IDB4MixcbiAgICAgICAgICAgIGFkYlRlc3QgPSAweDQ7XG5cbiAgICAgICAgdmFyIHBsYW5lVGVzdHMgPSAoRG90KGFiYywgYW8pID4gMCA/IGFiY1Rlc3QgOiAwKSB8XG4gICAgICAgICAgICAoRG90KGFjZCwgYW8pID4gMCA/IGFjZFRlc3QgOiAwKSB8XG4gICAgICAgICAgICAoRG90KGFkYiwgYW8pID4gMCA/IGFkYlRlc3QgOiAwKTtcblxuICAgICAgICBzd2l0Y2ggKHBsYW5lVGVzdHMpIHtcbiAgICAgICAgICAgIGNhc2UgYWJjVGVzdDpcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFjZFRlc3Q6XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGM7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGQ7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFjLCBhZCwgYWNkLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhZGJUZXN0OlxuICAgICAgICAgICAgICAgIC8vaW4gZnJvbnQgb2YgdHJpYW5nbGUgQURCXG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGI7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGQ7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhYmNUZXN0IHwgYWNkVGVzdDpcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUd29UZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFjZFRlc3QgfCBhZGJUZXN0OlxuICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBjO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBkO1xuICAgICAgICAgICAgICAgIHNpbXBsZXhbMF0gPSBiO1xuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1R3b1RldHJhaGVkcm9uKGFvLCBhYywgYWQsIGFjZCwgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWRiVGVzdCB8IGFiY1Rlc3Q6XG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGI7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGQ7XG4gICAgICAgICAgICAgICAgc2ltcGxleFswXSA9IGM7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFkLCBhYiwgYWRiLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgZGVmYXVsdDpcbiAgICAgICAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuXG4gICAgICAgIC8vb3JpZ2luIGluIHRldHJhaGVkcm9uXG4gICAgICAgIHJldHVybiB0cnVlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBAbWV0aG9kIF9jaGVja1R3b1RldHJhaGVkcm9uXG4gICAgICogQHByaXZhdGVcbiAgICAgKi9cbiAgICBfY2hlY2tUd29UZXRyYWhlZHJvbjogZnVuY3Rpb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpIHtcbiAgICAgICAgdmFyIGFiY19hYyA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYmMsIGFjKTtcblxuICAgICAgICBpZiAoYWJjX2FjLmNvbnN0cnVjdG9yLkRvdChhYmNfYWMsIGFvKSA+IDApIHtcbiAgICAgICAgICAgIC8vdGhlIG9yaWdpbiBpcyBiZXlvbmQgQUMgZnJvbSBBQkMnc1xuICAgICAgICAgICAgLy9wZXJzcGVjdGl2ZSwgZWZmZWN0aXZlbHkgZXhjbHVkaW5nXG4gICAgICAgICAgICAvL0FDRCBmcm9tIGNvbnNpZGVyYXRpb25cblxuICAgICAgICAgICAgLy93ZSB0aHVzIG5lZWQgdGVzdCBvbmx5IEFDRFxuICAgICAgICAgICAgc2ltcGxleFsyXSA9IHNpbXBsZXhbMV07XG4gICAgICAgICAgICBzaW1wbGV4WzFdID0gc2ltcGxleFswXTtcblxuICAgICAgICAgICAgYWIgPSBzaW1wbGV4WzJdLnN1YnRyYWN0KHNpbXBsZXhbM10pO1xuICAgICAgICAgICAgYWMgPSBzaW1wbGV4WzFdLnN1YnRyYWN0KHNpbXBsZXhbM10pO1xuICAgICAgICAgICAgYWJjID0gYWIuY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFjKTtcblxuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVGV0cmFoZWRyb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIGFiX2FiYyA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWJjKTtcblxuICAgICAgICBpZiAoYWJfYWJjLmNvbnN0cnVjdG9yLkRvdChhYl9hYmMsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMik7XG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJfYWJjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xuICAgICAgICAgICAgLy9BQnhBMHhBQiBkaXJlY3Rpb24gd2UgYXJlIGxvb2tpbmcgZm9yXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcblxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQG1ldGhvZCBfY2hlY2tUZXRyYWhlZHJvblxuICAgICAqIEBwcml2YXRlXG4gICAgICovXG4gICAgX2NoZWNrVGV0cmFoZWRyb246IGZ1bmN0aW9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KSB7XG5cbiAgICAgICAgdmFyIGFjcCA9IGFiYy5jb25zdHJ1Y3Rvci5Dcm9zcyhhYmMsIGFjKTtcblxuICAgICAgICBpZiAoYWNwLmNvbnN0cnVjdG9yLkRvdChhY3AsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleFsyXSA9IHNpbXBsZXhbM107XG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgzLCAxKTtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDEpO1xuICAgICAgICAgICAgLy9kaXIgaXMgbm90IGFiY19hYyBiZWNhdXNlIGl0J3Mgbm90IHBvaW50IHRvd2FyZHMgdGhlIG9yaWdpbjtcbiAgICAgICAgICAgIC8vQUN4QTB4QUMgZGlyZWN0aW9uIHdlIGFyZSBsb29raW5nIGZvclxuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKHRoaXMuX2dldE5vcm1hbChhYywgYW8sIGFjKSk7XG5cbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuXG4gICAgICAgIC8vYWxtb3N0IHRoZSBzYW1lIGxpa2UgdHJpYW5nbGUgY2hlY2tzXG4gICAgICAgIHZhciBhYl9hYmMgPSBhYi5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWJjKTtcblxuICAgICAgICBpZiAoYWJfYWJjLmNvbnN0cnVjdG9yLkRvdChhYl9hYmMsIGFvKSA+IDApIHtcblxuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMik7XG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJfYWJjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xuICAgICAgICAgICAgLy9BQnhBMHhBQiBkaXJlY3Rpb24gd2UgYXJlIGxvb2tpbmcgZm9yXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFiLCBhbywgYWIpKTtcblxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgLy9idWlsZCBuZXcgdGV0cmFoZWRyb24gd2l0aCBuZXcgYmFzZVxuICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAxKTtcblxuICAgICAgICBkaXIuY29weUZyb20oYWJjKTtcblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEFkZHMgYWRnZSB0byB0aGUgbGlzdCBhbmQgY2hlY2tzIGlmIHRoZSBlZGdlIGlmIG5vdCBpbi5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2FkZEVkZ2VcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7T2JqZWN0W119IGVkZ2VzIFRoZSBlZGdlcy5cbiAgICAgKiBAcGFyYW0ge09iamVjdH0gZGlyIFRoZSBlZGdlIHRvIGNoZWNrLlxuICAgICAqL1xuICAgIF9hZGRFZGdlOiBmdW5jdGlvbihlZGdlcywgZWRnZSkge1xuICAgICAgICBmb3IgKHZhciBqID0gMDsgaiA8IGVkZ2VzLmxlbmd0aDsgaisrKSB7XG4gICAgICAgICAgICBpZiAoZWRnZXNbal0uYSA9PT0gZWRnZS5iICYmIGVkZ2VzW2pdLmIgPT09IGVkZ2UuYSkge1xuICAgICAgICAgICAgICAgIGVkZ2VzLnNwbGljZShqLCAxKTtcbiAgICAgICAgICAgICAgICByZXR1cm47XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgZWRnZXMucHVzaChlZGdlKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogVGhlIHN1cHBvcnQgZnVuY3Rpb24uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIHN1cHBvcnRcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXJlY3Rpb24gVGhlIGRpcmVjdGlvbi5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgc3VwcG9ydCBwb2ludHMuXG4gICAgICovXG4gICAgc3VwcG9ydDogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBkaXJlY3Rpb24pIHtcbiAgICAgICAgLy8gZCBpcyBhIHZlY3RvciBkaXJlY3Rpb24gKGRvZXNuJ3QgaGF2ZSB0byBiZSBub3JtYWxpemVkKVxuICAgICAgICAvLyBnZXQgcG9pbnRzIG9uIHRoZSBlZGdlIG9mIHRoZSBzaGFwZXMgaW4gb3Bwb3NpdGUgZGlyZWN0aW9uc1xuICAgICAgICBkaXJlY3Rpb24ubm9ybWFsaXplKCk7XG4gICAgICAgIHZhciBwMSA9IHRoaXMuX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvbihjb2xsaWRlclBvaW50cywgZGlyZWN0aW9uKTtcbiAgICAgICAgdmFyIHAyID0gdGhpcy5fZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uKGNvbGxpZGVkUG9pbnRzLCBkaXJlY3Rpb24uc2NhbGUoLTEpKTtcbiAgICAgICAgLy8gcGVyZm9ybSB0aGUgTWlua293c2tpIERpZmZlcmVuY2VcbiAgICAgICAgdmFyIHAzID0gcDEuc3VidHJhY3QocDIpO1xuICAgICAgICAvLyBwMyBpcyBub3cgYSBwb2ludCBpbiBNaW5rb3dza2kgc3BhY2Ugb24gdGhlIGVkZ2Ugb2YgdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXG4gICAgICAgIHJldHVybiBwMztcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBzaW1wbGV4IGNvbnRhaW5zIHRoZSBvcmlnaW4uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGNvbnRhaW5zT3JpZ2luXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXhUaGUgc2ltcGxleCBvciBmYWxzZSBpZiBubyBpbnRlcnNlY3Rpb24uXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkaXIgVGhlIGRpcmVjdGlvbiB0byB0ZXN0LlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IENvbnRhaW5zIG9yIG5vdC5cbiAgICAgKi9cbiAgICBjb250YWluc09yaWdpbjogZnVuY3Rpb24oc2ltcGxleCwgZGlyKSB7XG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMikge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zTGluZShzaW1wbGV4LCBkaXIpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gMykge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zVHJpYW5nbGUoc2ltcGxleCwgZGlyKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAoc2ltcGxleC5sZW5ndGggPT09IDQpIHtcbiAgICAgICAgICAgIHJldHVybiB0aGlzLl9jb250YWluc1RldHJhaGVkcm9uKHNpbXBsZXgsIGRpcik7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBUaGUgR0pLIChHaWxiZXJ04oCTSm9obnNvbuKAk0tlZXJ0aGkpIGFsZ29yaXRobS5cbiAgICAgKiBDb21wdXRlcyBzdXBwb3J0IHBvaW50cyB0byBidWlsZCB0aGUgTWlua293c2t5IGRpZmZlcmVuY2UgYW5kXG4gICAgICogY3JlYXRlIGEgc2ltcGxleC4gVGhlIHBvaW50cyBvZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3RcbiAgICAgKiBtdXN0IGJlIGNvbnZleGUuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGNoZWNrXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gVGhlIHNpbXBsZXggb3IgZmFsc2UgaWYgbm8gaW50ZXJzZWN0aW9uLlxuICAgICAqL1xuICAgIGNoZWNrOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcbiAgICAgICAgdmFyIGl0ID0gMDtcbiAgICAgICAgdmFyIGEsIHNpbXBsZXggPSBbXTtcblxuICAgICAgICB2YXIgY29sbGlkZXJDZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKGNvbGxpZGVyUG9pbnRzKTtcbiAgICAgICAgdmFyIGNvbGxpZGVkQ2VudGVyID0gdGhpcy5fZ2V0QmFyeWNlbnRlcihjb2xsaWRlZFBvaW50cyk7XG5cbiAgICAgICAgLy8gaW5pdGlhbCBkaXJlY3Rpb24gZnJvbSB0aGUgY2VudGVyIG9mIDFzdCBib2R5IHRvIHRoZSBjZW50ZXIgb2YgMm5kIGJvZHlcbiAgICAgICAgdmFyIGRpciA9IGNvbGxpZGVyQ2VudGVyLnN1YnRyYWN0KGNvbGxpZGVkQ2VudGVyKS5ub3JtYWxpemUoKTtcblxuICAgICAgICAvLyBpZiBpbml0aWFsIGRpcmVjdGlvbiBpcyB6ZXJvIOKAkyBzZXQgaXQgdG8gYW55IGFyYml0cmFyeSBheGlzICh3ZSBjaG9vc2UgWClcbiAgICAgICAgaWYgKGRpci5sZW5ndGhTcXVhcmVkKCkgPCB0aGlzLkVQU0lMT04pIHtcbiAgICAgICAgICAgIGRpci54ID0gMTtcbiAgICAgICAgfVxuXG4gICAgICAgIC8vIHNldCB0aGUgZmlyc3Qgc3VwcG9ydCBhcyBpbml0aWFsIHBvaW50IG9mIHRoZSBuZXcgc2ltcGxleFxuICAgICAgICBhID0gc2ltcGxleFswXSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XG5cbiAgICAgICAgaWYgKGEuY29uc3RydWN0b3IuRG90KGEsIGRpcikgPD0gMCkge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgZGlyLnNjYWxlSW5QbGFjZSgtMSwgZGlyKTsgLy8gd2Ugd2lsbCBiZSBzZWFyY2hpbmcgaW4gdGhlIG9wcG9zaXRlIGRpcmVjdGlvbiBuZXh0XG5cbiAgICAgICAgdmFyIG1heCA9IGNvbGxpZGVkUG9pbnRzLmxlbmd0aCAqIGNvbGxpZGVyUG9pbnRzLmxlbmd0aDtcbiAgICAgICAgd2hpbGUgKGl0IDwgbWF4KSB7XG4gICAgICAgICAgICBpZiAoZGlyLmxlbmd0aFNxdWFyZWQoKSA9PT0gMCAmJiBzaW1wbGV4Lmxlbmd0aCA+PSAyKSB7XG4gICAgICAgICAgICAgICAgLy8gR2V0IHBlcnBlbmRpY3VsYXIgZGlyZWN0aW9uIHRvIGxhc3Qgc2ltcGxleFxuICAgICAgICAgICAgICAgIGRpciA9IHNpbXBsZXhbc2ltcGxleC5sZW5ndGggLSAxXS5zdWJ0cmFjdChzaW1wbGV4W3NpbXBsZXgubGVuZ3RoIC0gMl0pO1xuICAgICAgICAgICAgICAgIHZhciB0bXAgPSBkaXIueTtcbiAgICAgICAgICAgICAgICBkaXIueSA9IC1kaXIueDtcbiAgICAgICAgICAgICAgICBkaXIueCA9IHRtcDtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgYSA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcik7XG5cbiAgICAgICAgICAgIC8vIG1ha2Ugc3VyZSB0aGF0IHRoZSBsYXN0IHBvaW50IHdlIGFkZGVkIGFjdHVhbGx5IHBhc3NlZCB0aGUgb3JpZ2luXG4gICAgICAgICAgICBpZiAoYS5jb25zdHJ1Y3Rvci5Eb3QoYSwgZGlyKSA8PSAwKSB7XG4gICAgICAgICAgICAgICAgLy8gaWYgdGhlIHBvaW50IGFkZGVkIGxhc3Qgd2FzIG5vdCBwYXN0IHRoZSBvcmlnaW4gaW4gdGhlIGRpcmVjdGlvbiBvZiBkXG4gICAgICAgICAgICAgICAgLy8gdGhlbiB0aGUgTWlua293c2tpIFN1bSBjYW5ub3QgcG9zc2libHkgY29udGFpbiB0aGUgb3JpZ2luIHNpbmNlXG4gICAgICAgICAgICAgICAgLy8gdGhlIGxhc3QgcG9pbnQgYWRkZWQgaXMgb24gdGhlIGVkZ2Ugb2YgdGhlIE1pbmtvd3NraSBEaWZmZXJlbmNlXG4gICAgICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICBzaW1wbGV4LnB1c2goYSk7XG4gICAgICAgICAgICAvLyBvdGhlcndpc2Ugd2UgbmVlZCB0byBkZXRlcm1pbmUgaWYgdGhlIG9yaWdpbiBpcyBpblxuICAgICAgICAgICAgLy8gdGhlIGN1cnJlbnQgc2ltcGxleFxuXG4gICAgICAgICAgICBpZiAodGhpcy5jb250YWluc09yaWdpbihzaW1wbGV4LCBkaXIpKSB7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHNpbXBsZXg7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpdCsrO1xuICAgICAgICB9XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEZpbmRzIHRoZSByZXNwb25zZSB3aXRoIHRoZSBzaW1wbGV4IChlZGdlcykgb2YgdGhlIGdqayBhbGdvcml0aG0uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGZpbmRSZXNwb25zZVdpdGhFZGdlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEBwYXJhbSB7bnVtYmVyW119IFtleGNlcHRdIEV4Y2VwdGlvbnM6IGVkZ2UgaW5kaWNlcyB0byBpZ25vcmVcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGZpbmRSZXNwb25zZVdpdGhFZGdlOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgsIGV4Y2VwdCkge1xuICAgICAgICBpZiAoZXhjZXB0ID09PSB1bmRlZmluZWQpIGV4Y2VwdCA9IFtdO1xuXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5fZ2V0TmVhcmVzdEVkZ2Uoc2ltcGxleCwgZXhjZXB0KTtcbiAgICAgICAgaWYgKGVkZ2UuaW5kZXggPT09IHVuZGVmaW5lZCkgcmV0dXJuIGZhbHNlO1xuXG4gICAgICAgIHZhciBzdXAgPSB0aGlzLnN1cHBvcnQoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBlZGdlLm5vcm1hbCk7IC8vZ2V0IHN1cHBvcnQgcG9pbnQgaW4gZGlyZWN0aW9uIG9mIGVkZ2UncyBub3JtYWxcbiAgICAgICAgdmFyIGQgPSBNYXRoLmFicyhzdXAuY29uc3RydWN0b3IuRG90KHN1cCwgZWRnZS5ub3JtYWwpKTtcblxuICAgICAgICBpZiAoZCAtIGVkZ2UuZGlzdGFuY2UgPD0gdGhpcy5FUFNJTE9OKSB7XG4gICAgICAgICAgICBleGNlcHQucHVzaChlZGdlLmluZGV4KTtcbiAgICAgICAgICAgIHJldHVybiBlZGdlLm5vcm1hbC5zY2FsZShlZGdlLmRpc3RhbmNlKTtcbiAgICAgICAgfVxuICAgICAgICAvLyBhZGQgbmV3IHN1cHBvcnQgcG9pbnQgaW4gc2ltcGxleFxuICAgICAgICBzaW1wbGV4LnNwbGljZShlZGdlLmluZGV4LCAwLCBzdXApO1xuICAgICAgICAvLyB1cGRhdGUgaW5kaWNlcyBvZiBlZGdlcyBleGNlcHRpb25zXG4gICAgICAgIHRoaXMuX3VwZGF0ZUV4Y2VwdGlvbnNCeUluc2VydChleGNlcHQsIGVkZ2UuaW5kZXgsIHNpbXBsZXgpO1xuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogRmluZHMgdGhlIHJlc3BvbnNlIHdpdGggdGhlIHBvbHl0b3BlIGRvbmUgd2l0aCB0aGUgc2ltcGxleCBvZiB0aGUgZ2prIGFsZ29yaXRobS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgZmluZFJlc3BvbnNlV2l0aFRyaWFuZ2xlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge1RyaWFuZ2xlW119IHBvbHl0b3BlIFRoZSBwb2x5dG9wZSBkb25lIHdpdGggdGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtudW1iZXJbXX0gW2V4Y2VwdF0gRXhjZXB0aW9uczogdHJpYW5nbGUgaW5kaWNlcyB0byBpZ25vcmVcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZTogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBwb2x5dG9wZSwgZXhjZXB0KSB7XG4gICAgICAgIGlmIChleGNlcHQgPT09IHVuZGVmaW5lZCkgZXhjZXB0ID0gW107XG5cbiAgICAgICAgaWYgKHBvbHl0b3BlLmxlbmd0aCA9PT0gMCkge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIG5lYXJlc3QgPSB0aGlzLl9nZXROZWFyZXN0VHJpYW5nbGUocG9seXRvcGUsIGV4Y2VwdCk7XG4gICAgICAgIGlmIChuZWFyZXN0LmluZGV4ID09PSB1bmRlZmluZWQpIHJldHVybiBmYWxzZTtcblxuICAgICAgICB2YXIgdHJpYW5nbGUgPSBwb2x5dG9wZVtuZWFyZXN0LmluZGV4XTtcblxuICAgICAgICB2YXIgc3VwID0gdGhpcy5zdXBwb3J0KGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgdHJpYW5nbGUubik7XG5cbiAgICAgICAgdmFyIGQgPSBNYXRoLmFicyhzdXAuY29uc3RydWN0b3IuRG90KHN1cCwgdHJpYW5nbGUubikpO1xuXG4gICAgICAgIGlmICgoZCAtIG5lYXJlc3QuZGlzdGFuY2UgPD0gdGhpcy5FUFNJTE9OKSkge1xuICAgICAgICAgICAgZXhjZXB0LnB1c2gobmVhcmVzdC5pbmRleCk7XG4gICAgICAgICAgICByZXR1cm4gdHJpYW5nbGUubi5zY2FsZShuZWFyZXN0LmRpc3RhbmNlKTtcbiAgICAgICAgfVxuXG4gICAgICAgIHZhciBlZGdlcyA9IFtdO1xuICAgICAgICBmb3IgKHZhciBpID0gcG9seXRvcGUubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pIHtcbiAgICAgICAgICAgIHRyaWFuZ2xlID0gcG9seXRvcGVbaV07XG4gICAgICAgICAgICAvLyBjYW4gdGhpcyBmYWNlIGJlICdzZWVuJyBieSBlbnRyeV9jdXJfc3VwcG9ydD9cbiAgICAgICAgICAgIGlmICh0cmlhbmdsZS5uLmNvbnN0cnVjdG9yLkRvdCh0cmlhbmdsZS5uLCBzdXAuc3VidHJhY3QocG9seXRvcGVbaV0uYSkpID4gMCkge1xuICAgICAgICAgICAgICAgIHRoaXMuX2FkZEVkZ2UoZWRnZXMsIHtcbiAgICAgICAgICAgICAgICAgICAgYTogdHJpYW5nbGUuYSxcbiAgICAgICAgICAgICAgICAgICAgYjogdHJpYW5nbGUuYlxuICAgICAgICAgICAgICAgIH0pO1xuICAgICAgICAgICAgICAgIHRoaXMuX2FkZEVkZ2UoZWRnZXMsIHtcbiAgICAgICAgICAgICAgICAgICAgYTogdHJpYW5nbGUuYixcbiAgICAgICAgICAgICAgICAgICAgYjogdHJpYW5nbGUuY1xuICAgICAgICAgICAgICAgIH0pO1xuICAgICAgICAgICAgICAgIHRoaXMuX2FkZEVkZ2UoZWRnZXMsIHtcbiAgICAgICAgICAgICAgICAgICAgYTogdHJpYW5nbGUuYyxcbiAgICAgICAgICAgICAgICAgICAgYjogdHJpYW5nbGUuYVxuICAgICAgICAgICAgICAgIH0pO1xuICAgICAgICAgICAgICAgIC8vIHJlbW92ZSBhIHRyaWFuZ2xlXG4gICAgICAgICAgICAgICAgcG9seXRvcGUuc3BsaWNlKGksIDEpO1xuICAgICAgICAgICAgICAgIC8vIHVwZGF0ZSBleGNlcHRpb25zIHdpdGggcmVtb3ZlZCB0cmlhbmdsZSBpbmRleFxuICAgICAgICAgICAgICAgIHRoaXMuX3JlbW92ZUV4Y2VwdGlvbihleGNlcHQsIGkpO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG5cbiAgICAgICAgLy8gY3JlYXRlIG5ldyB0cmlhbmdsZXMgZnJvbSB0aGUgZWRnZXMgaW4gdGhlIGVkZ2UgbGlzdFxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGVkZ2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICB0cmlhbmdsZSA9IG5ldyBUcmlhbmdsZShzdXAsIGVkZ2VzW2ldLmEsIGVkZ2VzW2ldLmIpO1xuICAgICAgICAgICAgaWYgKHRyaWFuZ2xlLm4ubGVuZ3RoKCkgIT09IDApIHtcbiAgICAgICAgICAgICAgICBwb2x5dG9wZS5wdXNoKHRyaWFuZ2xlKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogVXBkYXRlIGV4Y2VwdGlvbiBmcm9tIGluc2VydGlvbiBvZiBuZXcgZWRnZVxuICAgICAqIEBwYXJhbSB7bnVtYmVyW119IGV4Y2VwdCBcbiAgICAgKiBAcGFyYW0ge251bWJlcn0gZWRnZUluZGV4IGluc2VydGVkIGVkZ2UgaW5kZXhcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBzaW1wbGV4IFxuICAgICAqL1xuICAgIF91cGRhdGVFeGNlcHRpb25zQnlJbnNlcnQ6IGZ1bmN0aW9uKGV4Y2VwdCwgZWRnZUluZGV4LCBzaW1wbGV4KSB7XG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZXhjZXB0Lmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICBpZiAoZXhjZXB0W2ldID49IGVkZ2VJbmRleCkge1xuICAgICAgICAgICAgICAgIGV4Y2VwdFtpXSA9IChleGNlcHRbaV0gKyAxKSAlIHNpbXBsZXgubGVuZ3RoO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIFVwZGF0ZSBleGNlcHRpb25zIHdoZW4gYW4gaW5kZXggaGFzIGJlZW4gcmVtb3ZlZFxuICAgICAqIEBwYXJhbSB7bnVtYmVyW119IGV4Y2VwdCBcbiAgICAgKiBAcGFyYW0ge251bWJlcn0gaW5kZXhUb1JlbW92ZSBcbiAgICAgKi9cbiAgICBfcmVtb3ZlRXhjZXB0aW9uOiBmdW5jdGlvbihleGNlcHQsIGluZGV4VG9SZW1vdmUpIHtcbiAgICAgICAgZm9yICh2YXIgaSA9IGV4Y2VwdC5sZW5ndGggLSAxOyBpID49IDA7IGktLSkge1xuICAgICAgICAgICAgaWYoZXhjZXB0W2ldID09PSBpbmRleFRvUmVtb3ZlKSB7XG4gICAgICAgICAgICAgICAgZXhjZXB0LnNwbGljZShpLCAxKTtcbiAgICAgICAgICAgIH0gZWxzZSBpZiAoZXhjZXB0W2ldID4gaW5kZXhUb1JlbW92ZSkge1xuICAgICAgICAgICAgICAgIGV4Y2VwdFtpXS0tO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIHJlc3BvbnNlIG9mIHRoZSBwZW5ldHJhdGlvbiB2ZWN0b3Igd2l0aCB0aGUgc2ltcGxleC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgZ2V0UmVzcG9uc2VcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHNpbXBsZXggVGhlIHNpbXBsZXggb2YgdGhlIE1pbmtvd3NreSBkaWZmZXJlbmNlLlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgZ2V0UmVzcG9uc2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCkge1xuICAgICAgICB2YXIgcmVzcG9uc2VzID0gdGhpcy5nZXRSZXNwb25zZXMoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4LCB7IG1heFJlc3BvbnNlczogMSB9KTtcbiAgICAgICAgaWYgKCFyZXNwb25zZXMpIHJldHVybiByZXNwb25zZXM7XG4gICAgICAgIGlmIChyZXNwb25zZXNbMF0pIHJldHVybiByZXNwb25zZXNbMF07XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgZ2V0UmVzcG9uc2VzOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgsIG9wdGlvbnMpIHtcbiAgICAgICAgaWYgKG9wdGlvbnMgPT09IHVuZGVmaW5lZCkgb3B0aW9ucyA9IHt9O1xuICAgICAgICBvcHRpb25zID0gT2JqZWN0LmFzc2lnbih7IG1heFJlc3BvbnNlczogLTEgfSwgb3B0aW9ucyk7XG5cbiAgICAgICAgdmFyIGl0ID0gMCxcbiAgICAgICAgICAgIHJlc3BvbnNlO1xuICAgICAgICB2YXIgcG9seXRvcGUgPSBzaW1wbGV4WzBdLnogIT09IHVuZGVmaW5lZCA/IFtuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFsxXSwgc2ltcGxleFsyXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFsyXSwgc2ltcGxleFszXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFszXSwgc2ltcGxleFsxXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFsxXSwgc2ltcGxleFszXSwgc2ltcGxleFsyXSlcbiAgICAgICAgXSA6IG51bGw7XG5cbiAgICAgICAgdmFyIG1heCA9IGNvbGxpZGVkUG9pbnRzLmxlbmd0aCAqIGNvbGxpZGVyUG9pbnRzLmxlbmd0aDtcbiAgICAgICAgdmFyIGV4Y2VwdCA9IFtdO1xuICAgICAgICB2YXIgcmVzcG9uc2VzID0gW107XG4gICAgICAgIHdoaWxlIChpdCA8IG1heCAmJiByZXNwb25zZXMubGVuZ3RoICE9PSBvcHRpb25zLm1heFJlc3BvbnNlcykge1xuICAgICAgICAgICAgaWYgKHNpbXBsZXhbMF0ueiA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhFZGdlKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCwgZXhjZXB0KTtcbiAgICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHBvbHl0b3BlLCBleGNlcHQpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgaWYgKHJlc3BvbnNlKSB7XG4gICAgICAgICAgICAgICAgdmFyIG5vcm0gPSByZXNwb25zZS5jbG9uZSgpLm5vcm1hbGl6ZSgpLnNjYWxlSW5QbGFjZSh0aGlzLkVQU0lMT04pO1xuICAgICAgICAgICAgICAgIHZhciByZXBvbnNlV2l0aE9mZnNldCA9IHJlc3BvbnNlLmFkZFRvUmVmKG5vcm0sIHJlc3BvbnNlKTtcbiAgICAgICAgICAgICAgICB0aGlzLl9hZGRSZXNwb25zZShyZXNwb25zZXMsIHJlcG9uc2VXaXRoT2Zmc2V0KTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGl0Kys7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIHJlc3BvbnNlcztcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQWRkIHJlc3BvbnNlIHRvIHJlc3BvbnNlcyBpZiBub3QgYWxyZWFkeSBwcmVzZW50XG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHJlc3BvbnNlc1xuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gcmVzcG9uc2VcbiAgICAgKi9cbiAgICBfYWRkUmVzcG9uc2U6IGZ1bmN0aW9uKHJlc3BvbnNlcywgcmVzcG9uc2UpIHtcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCByZXNwb25zZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIGlmIChyZXNwb25zZXNbaV0uc3VidHJhY3QocmVzcG9uc2UpLmxlbmd0aFNxdWFyZWQoKSA8IHRoaXMuRVBTSUxPTikge1xuICAgICAgICAgICAgICAgIHJldHVybjtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIHJlc3BvbnNlcy5wdXNoKHJlc3BvbnNlKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdCBhcmUgaW50ZXJzZWN0aW5nLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBpc0ludGVyc2VjdGluZ1xuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcmV0dXJuIHtCb29sZWFufSBJcyBpbnRlcnNlY3Rpbmcgb3Igbm90LlxuICAgICAqL1xuICAgIGlzSW50ZXJzZWN0aW5nOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpIHtcbiAgICAgICAgcmV0dXJuICEhdGhpcy5jaGVjayhjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIGNvbGxpZGVyIGFuZCB0aGUgY29sbGlkZWQgb2JqZWN0IGFyZSBpbnRlcnNlY3RpbmdcbiAgICAgKiBhbmQgZ2l2ZSB0aGUgcmVzcG9uc2UgdG8gYmUgb3V0IG9mIHRoZSBvYmplY3QuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGludGVyc2VjdFxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGludGVyc2VjdDogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKSB7XG4gICAgICAgIHZhciBzaW1wbGV4ID0gdGhpcy5jaGVjayhjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMpO1xuXG4gICAgICAgIC8vdGhpcy5jdWJlID0gdGhpcy5jdWJlIHx8IFtdO1xuICAgICAgICBpZiAoc2ltcGxleCkge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuZ2V0UmVzcG9uc2UoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4KTtcbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdCBhcmUgaW50ZXJzZWN0aW5nXG4gICAgICogYW5kIGdpdmUgdGhlIG11bHRpcGxlIHJlc3BvbnNlcyB0byBiZSBvdXQgb2YgdGhlIG9iamVjdC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgaW50ZXJzZWN0TXVsdGlwbGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHBhcmFtIHtPYmplY3R9IFtvcHRpb25zXVxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gVGhlIHBlbmV0cmF0aW9uIHZlY3RvcnMuXG4gICAgICovXG4gICAgaW50ZXJzZWN0TXVsdGlwbGU6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgb3B0aW9ucykge1xuICAgICAgICB2YXIgc2ltcGxleCA9IHRoaXMuY2hlY2soY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKTtcblxuICAgICAgICBpZiAoc2ltcGxleCkge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuZ2V0UmVzcG9uc2VzKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCwgb3B0aW9ucyk7XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBDb2xsaXNpb25HamtFcGE7XG4iXX0=
