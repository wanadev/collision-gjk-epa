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

        var edge = this._getNearestEdge(simplex);
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

        var nearest = this._getNearestTriangle(polytope);
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
        var responses = this.getResponses(colliderPoints, collidedPoints, simplex);
        if (!responses) return responses;
        if (responses[0]) return responses[0];
        return false;
    },

    getResponses: function(colliderPoints, collidedPoints, simplex, options) {
        if(options === undefined) options = {};
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
            if (responses[i].subtract(response).lengthSquared < this.EPSILON) {
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
    }
};

module.exports = CollisionGjkEpa;

},{}]},{},[1])(1)
});

//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJjb2xsaXNpb24tZ2prLWVwYS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbigpe2Z1bmN0aW9uIHIoZSxuLHQpe2Z1bmN0aW9uIG8oaSxmKXtpZighbltpXSl7aWYoIWVbaV0pe3ZhciBjPVwiZnVuY3Rpb25cIj09dHlwZW9mIHJlcXVpcmUmJnJlcXVpcmU7aWYoIWYmJmMpcmV0dXJuIGMoaSwhMCk7aWYodSlyZXR1cm4gdShpLCEwKTt2YXIgYT1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK2krXCInXCIpO3Rocm93IGEuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixhfXZhciBwPW5baV09e2V4cG9ydHM6e319O2VbaV1bMF0uY2FsbChwLmV4cG9ydHMsZnVuY3Rpb24ocil7dmFyIG49ZVtpXVsxXVtyXTtyZXR1cm4gbyhufHxyKX0scCxwLmV4cG9ydHMscixlLG4sdCl9cmV0dXJuIG5baV0uZXhwb3J0c31mb3IodmFyIHU9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZSxpPTA7aTx0Lmxlbmd0aDtpKyspbyh0W2ldKTtyZXR1cm4gb31yZXR1cm4gcn0pKCkiLCIndXNlIHN0cmljdCc7XG5cbi8qKlxuICogQ2xhc3MgdG8gaGVscCBpbiB0aGUgY29sbGlzaW9uIGluIDJEIGFuZCAzRC5cbiAqIFRvIHdvcmtzIHRoZSBhbGdvcml0aG0gbmVlZHMgdHdvIGNvbnZleGUgcG9pbnQgY2xvdWRcbiAqIGxpa2UgYm91bmRpbmcgYm94IG9yIHNtdGhnIGxpa2UgdGhpcy5cbiAqIFRoZSBmdW5jdGlvbiBmdW5jdGlvbnMgaW50ZXJzZWN0IGFuZCBpc0ludGVyc2VjdGluZ1xuICogaGVscHMgdG8gaGF2ZSBpbmZvcm1hdGlvbnMgYWJvdXQgdGhlIGNvbGxpc2lvbiBiZXR3ZWVuIHR3byBvYmplY3QuXG4gKlxuICogQGNsYXNzIHducC5oZWxwZXJzLkNvbGxpc2lvbkdqa0VwYVxuICogQGNvbnN0cnVjdG9yXG4gKi9cbnZhciBUcmlhbmdsZSA9IGZ1bmN0aW9uKGEsIGIsIGMpIHtcbiAgICAgIHRoaXMuYSA9IGE7XG4gICAgICB0aGlzLmIgPSBiO1xuICAgICAgdGhpcy5jID0gYztcbiAgICAgIHRoaXMubiA9IGEuY29uc3RydWN0b3IuQ3Jvc3MoYi5zdWJ0cmFjdChhKSwgYy5zdWJ0cmFjdChhKSkubm9ybWFsaXplKCk7XG4gIH07XG5cblxudmFyIENvbGxpc2lvbkdqa0VwYSA9IHtcblxuICAgIEVQU0lMT046IDAuMDAwMDAxLFxuXG4gICAgLyoqXG4gICAgICogTWV0aG9kIHRvIGdldCBhIG5vcm1hbCBvZiAzIHBvaW50cyBpbiAyRCBhbmQgM0QuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9nZXROb3JtYWxcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gYVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gYlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gY1xuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBub3JtYWwuXG4gICAgICovXG4gICAgX2dldE5vcm1hbDogZnVuY3Rpb24oYSwgYiwgYykge1xuICAgICAgICB2YXIgRG90ID0gYS5jb25zdHJ1Y3Rvci5Eb3Q7XG4gICAgICAgIHZhciBhYyA9IERvdChhLCBjKTsgLy8gcGVyZm9ybSBhRG90KGMpXG4gICAgICAgIHZhciBiYyA9IERvdChiLCBjKTsgLy8gcGVyZm9ybSBiRG90KGMpXG5cbiAgICAgICAgLy8gcGVyZm9ybSBiICogYS5Eb3QoYykgLSBhICogYi5Eb3QoYylcbiAgICAgICAgdmFyIHIgPSBiLnNjYWxlKGFjKS5zdWJ0cmFjdChhLnNjYWxlKGJjKSk7XG4gICAgICAgIGlmIChyLmxlbmd0aFNxdWFyZWQoKSA8IHRoaXMuRVBTSUxPTiAqIHRoaXMuRVBTSUxPTikge1xuICAgICAgICAgICAgcmV0dXJuIHIuc2NhbGUoMCk7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIHIubm9ybWFsaXplKCk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIGJhcnljZW50ZXIgb2YgYSBjbG91ZCBwb2ludHMuXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9nZXRCYXJ5Y2VudGVyXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gdmVydGljZXMgdGhlIGNsb3VkIHBvaW50c1xuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBiYXJ5Y2VudGVyLlxuICAgICAqL1xuICAgIF9nZXRCYXJ5Y2VudGVyOiBmdW5jdGlvbih2ZXJ0aWNlcykge1xuICAgICAgICB2YXIgYXZnID0gdmVydGljZXNbMF0uY29uc3RydWN0b3IuWmVybygpO1xuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHZlcnRpY2VzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICBhdmcuYWRkVG9SZWYodmVydGljZXNbaV0sIGF2Zyk7XG4gICAgICAgIH1cbiAgICAgICAgYXZnLnNjYWxlSW5QbGFjZSgxIC8gdmVydGljZXMubGVuZ3RoLCBhdmcpO1xuICAgICAgICByZXR1cm4gYXZnO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSBmYXJ0aGVzdCBwb2ludCBvZiBhIGNsb3VkIHBvaW50cy5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldEZhcnRoZXN0UG9pbnRJbkRpcmVjdGlvblxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHZlcnRpY2VzIHRoZSBjbG91ZCBwb2ludHMuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBkIFRoZSBkaXJlY3Rpb24gdG8gc2VhcmNoLlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBiYXJ5Y2VudGVyLlxuICAgICAqL1xuICAgIF9nZXRGYXJ0aGVzdFBvaW50SW5EaXJlY3Rpb246IGZ1bmN0aW9uKHZlcnRpY2VzLCBkKSB7XG4gICAgICAgIHZhciBEb3QgPSB2ZXJ0aWNlc1swXS5jb25zdHJ1Y3Rvci5Eb3Q7XG4gICAgICAgIHZhciBtYXhQcm9kdWN0ID0gRG90KHZlcnRpY2VzWzBdLCBkKTtcbiAgICAgICAgdmFyIGluZGV4ID0gMDtcbiAgICAgICAgZm9yICh2YXIgaSA9IDE7IGkgPCB2ZXJ0aWNlcy5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgdmFyIHByb2R1Y3QgPSBEb3QodmVydGljZXNbaV0sIGQpO1xuICAgICAgICAgICAgaWYgKHByb2R1Y3QgPiBtYXhQcm9kdWN0KSB7XG4gICAgICAgICAgICAgICAgbWF4UHJvZHVjdCA9IHByb2R1Y3Q7XG4gICAgICAgICAgICAgICAgaW5kZXggPSBpO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICAgIHJldHVybiB2ZXJ0aWNlc1tpbmRleF07XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIG5lYXJlc3QgZWRnZSBvZiB0aGUgc2ltcGxleC5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2dldE5lYXJlc3RFZGdlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcGFyYW0ge251bWJlcltdfSBbZXhjZXB0XSBFeGNlcHRpb25zOiBlZGdlIGluZGljZXMgdG8gaWdub3JlXG4gICAgICogQHJldHVybiB7T2JqZWN0fSBJbmZvcm1hdGlvbnMgYWJvdXQgdGhlIG5lYXJlc3QgZWRnZSAoZGlzdGFuY2UsIGluZGV4IGFuZCBub3JtYWwpLlxuICAgICAqL1xuICAgIF9nZXROZWFyZXN0RWRnZTogZnVuY3Rpb24oc2ltcGxleCwgZXhjZXB0KSB7XG4gICAgICAgIGlmIChleGNlcHQgPT09IHVuZGVmaW5lZCkgZXhjZXB0ID0gW107XG5cbiAgICAgICAgdmFyIGRpc3RhbmNlID0gSW5maW5pdHksXG4gICAgICAgICAgICBpbmRleCwgbm9ybWFsO1xuXG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgc2ltcGxleC5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgdmFyIGogPSAoaSArIDEpICUgc2ltcGxleC5sZW5ndGg7XG4gICAgICAgICAgICBpZiAoZXhjZXB0LmluY2x1ZGVzKGopKSBjb250aW51ZTsgLy8gaWdub3JlIHRoaXMgZWRnZVxuXG4gICAgICAgICAgICB2YXIgdjEgPSBzaW1wbGV4W2ldO1xuICAgICAgICAgICAgdmFyIHYyID0gc2ltcGxleFtqXTtcblxuICAgICAgICAgICAgdmFyIGVkZ2UgPSB2Mi5zdWJ0cmFjdCh2MSk7XG4gICAgICAgICAgICBpZiAoZWRnZS5sZW5ndGhTcXVhcmVkKCkgPT09IDApIHtcbiAgICAgICAgICAgICAgICBjb250aW51ZTtcbiAgICAgICAgICAgIH1cblxuICAgICAgICAgICAgdmFyIG9yaWdpblRvdjEgPSB2MTtcblxuICAgICAgICAgICAgdmFyIG4gPSB0aGlzLl9nZXROb3JtYWwoZWRnZSwgb3JpZ2luVG92MSwgZWRnZSk7XG5cbiAgICAgICAgICAgIGlmIChuLmxlbmd0aFNxdWFyZWQoKSA9PT0gMCkge1xuICAgICAgICAgICAgICAgIC8vIE9yaWdpbiBpcyBvbiB0aGUgZWRnZVxuICAgICAgICAgICAgICAgIG4ueSA9IC1lZGdlLng7XG4gICAgICAgICAgICAgICAgbi54ID0gZWRnZS55O1xuICAgICAgICAgICAgICAgIC8vIG5vcm1hbCBzaG91bGQgZ28gb3V0c2lkZSB0aGUgc2ltcGxleFxuICAgICAgICAgICAgICAgIHZhciBjZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKHNpbXBsZXgpXG4gICAgICAgICAgICAgICAgdmFyIGNlbnRlclRvdjEgPSB2MS5zdWJ0cmFjdChjZW50ZXIpO1xuICAgICAgICAgICAgICAgIGlmIChuLmNvbnN0cnVjdG9yLkRvdChuLCBjZW50ZXJUb3YxKSA8IDApIHtcbiAgICAgICAgICAgICAgICAgICAgbi55ID0gLW4ueTtcbiAgICAgICAgICAgICAgICAgICAgbi54ID0gLW4ueDtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIC8vbiA9IG4uc2NhbGUoMSAvIG4ubGVuZ3RoKCkpOyAvL25vcm1hbGl6ZVxuICAgICAgICAgICAgdmFyIGRpc3QgPSBNYXRoLmFicyhuLmNvbnN0cnVjdG9yLkRvdChuLCB2MSkpOyAvL2Rpc3RhbmNlIGZyb20gb3JpZ2luIHRvIGVkZ2VcblxuICAgICAgICAgICAgaWYgKGRpc3QgPCBkaXN0YW5jZSkge1xuICAgICAgICAgICAgICAgIGRpc3RhbmNlID0gZGlzdDtcbiAgICAgICAgICAgICAgICBpbmRleCA9IGo7XG4gICAgICAgICAgICAgICAgbm9ybWFsID0gbjtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiB7XG4gICAgICAgICAgICBkaXN0YW5jZTogZGlzdGFuY2UsXG4gICAgICAgICAgICBpbmRleDogaW5kZXgsXG4gICAgICAgICAgICBub3JtYWw6IG5vcm1hbFxuICAgICAgICB9O1xuXG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEdldHMgdGhlIG5lYXJlc3QgVHJpYW5nbGUgb2YgdGhlIHBvbHl0b3BlLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfZ2V0TmVhcmVzdFRyaWFuZ2xlXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVHJpYW5nbGVbXX0gcG9seXRvcGUgVGhlIHBvbHl0b3BlLlxuICAgICAqIEBwYXJhbSB7bnVtYmVyW119IFtleGNlcHRdIEV4Y2VwdGlvbnM6IHRyaWFuZ2xlIGluZGljZXMgdG8gaWdub3JlXG4gICAgICogQHJldHVybiB7T2JqZWN0fSBJbmZvcm1hdGlvbnMgYWJvdXQgdGhlIG5lYXJlc3QgZWRnZSAoZGlzdGFuY2UgYW5kIGluZGV4KS5cbiAgICAgKi9cbiAgICBfZ2V0TmVhcmVzdFRyaWFuZ2xlOiBmdW5jdGlvbihwb2x5dG9wZSwgZXhjZXB0KSB7XG4gICAgICAgIGlmIChleGNlcHQgPT09IHVuZGVmaW5lZCkgZXhjZXB0ID0gW107XG5cbiAgICAgICAgdmFyIGRpc3RhbmNlID0gSW5maW5pdHksXG4gICAgICAgICAgICBpbmRleDtcblxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHBvbHl0b3BlLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgICAgICBpZiAoZXhjZXB0LmluY2x1ZGVzKGkpKSBjb250aW51ZTtcblxuICAgICAgICAgICAgdmFyIHRyaWFuZ2xlID0gcG9seXRvcGVbaV07XG4gICAgICAgICAgICB2YXIgZGlzdCA9IE1hdGguYWJzKHRyaWFuZ2xlLm4uY29uc3RydWN0b3IuRG90KHRyaWFuZ2xlLm4sIHRyaWFuZ2xlLmEpKTtcblxuICAgICAgICAgICAgaWYgKGRpc3QgPCBkaXN0YW5jZSkge1xuICAgICAgICAgICAgICAgIGRpc3RhbmNlID0gZGlzdDtcbiAgICAgICAgICAgICAgICBpbmRleCA9IGk7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiB7XG4gICAgICAgICAgICBkaXN0YW5jZTogZGlzdGFuY2UsXG4gICAgICAgICAgICBpbmRleDogaW5kZXhcbiAgICAgICAgfTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBvcmlnaW4gaXMgaW4gYSBsaW5lLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfY29udGFpbnNMaW5lXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uLlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IEZhbHNlIGluIGFueSBjYXNlIGJlY2F1c2UgdGhlIGFsZ29yaXRobSBqdXN0IGJlZ2luLlxuICAgICAqL1xuICAgIF9jb250YWluc0xpbmU6IGZ1bmN0aW9uKHNpbXBsZXgsIGRpcikge1xuICAgICAgICB2YXIgYSA9IHNpbXBsZXhbMV07XG4gICAgICAgIHZhciBiID0gc2ltcGxleFswXTtcbiAgICAgICAgdmFyIGFiID0gYi5zdWJ0cmFjdChhKTtcbiAgICAgICAgdmFyIGFvID0gYS5zY2FsZSgtMSk7XG4gICAgICAgIGlmIChhYi5sZW5ndGhTcXVhcmVkKCkgIT09IDApIHtcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbSh0aGlzLl9nZXROb3JtYWwoYWIsIGFvLCBhYikpO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgZGlyLmNvcHlGcm9tKGFvKTtcbiAgICAgICAgfVxuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBvcmlnaW4gaXMgaW4gYSB0cmlhbmdsZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgX2NvbnRhaW5zVHJpYW5nbGVcbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4IFRoZSBzaW1wbGV4LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gZGlyIFRoZSBkaXJlY3Rpb24uXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gSWYgaW4gMkQgY2FzZSwgbWF5IHJldHVybiB0cnVlIGlmIHRoZSBvcmlnaW4gaXMgaW4gdGhlIHRyaWFuZ2xlLlxuICAgICAqL1xuICAgIF9jb250YWluc1RyaWFuZ2xlOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcbiAgICAgICAgdmFyIGEgPSBzaW1wbGV4WzJdO1xuICAgICAgICB2YXIgRG90ID0gc2ltcGxleFsyXS5jb25zdHJ1Y3Rvci5Eb3Q7XG4gICAgICAgIHZhciBiID0gc2ltcGxleFsxXTtcbiAgICAgICAgdmFyIGMgPSBzaW1wbGV4WzBdO1xuICAgICAgICB2YXIgYWIgPSBiLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYWMgPSBjLnN1YnRyYWN0KGEpO1xuICAgICAgICB2YXIgYW8gPSBhLnNjYWxlKC0xKTtcblxuICAgICAgICB2YXIgYWJwID0gdGhpcy5fZ2V0Tm9ybWFsKGFjLCBhYiwgYWIpO1xuICAgICAgICB2YXIgYWNwID0gdGhpcy5fZ2V0Tm9ybWFsKGFiLCBhYywgYWMpO1xuICAgICAgICBpZiAoRG90KGFicCwgYW8pID4gMCkge1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7IC8vIHJlbW92ZSBDXG4gICAgICAgICAgICBkaXIuY29weUZyb20oYWJwKTtcbiAgICAgICAgfSBlbHNlIGlmIChEb3QoYWNwLCBhbykgPiAwKSB7XG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgxLCAxKTsgLy8gcmVtb3ZlIEJcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbShhY3ApO1xuICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgaWYgKGRpci56ID09PSB1bmRlZmluZWQpIHtcbiAgICAgICAgICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAgICAgdmFyIGFiYyA9IHNpbXBsZXhbMl0uY29uc3RydWN0b3IuQ3Jvc3MoYWIsIGFjKTtcbiAgICAgICAgICAgICAgICBkaXIuY29weUZyb20oYWJjKTtcbiAgICAgICAgICAgICAgICBpZiAoRG90KGFiYywgYW8pIDw9IDApIHtcbiAgICAgICAgICAgICAgICAgICAgLy91cHNpZGUgZG93biB0ZXRyYWhlZHJvblxuICAgICAgICAgICAgICAgICAgICBzaW1wbGV4WzBdID0gYjtcbiAgICAgICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGM7XG4gICAgICAgICAgICAgICAgICAgIHNpbXBsZXhbMl0gPSBhO1xuICAgICAgICAgICAgICAgICAgICBkaXIuY29weUZyb20oYWJjLnNjYWxlKC0xKSk7XG4gICAgICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgb3JpZ2luIGlzIGluIGEgdGV0cmFoZWRyb24uXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIF9jb250YWluc1RldHJhaGVkcm9uXG4gICAgICogQHByaXZhdGVcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleCBUaGUgc2ltcGxleC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uLlxuICAgICAqIEByZXR1cm4ge0Jvb2xlYW59IFJldHVybiB0cnVlIGlmIHRoZSBvcmlnaW4gaXMgaW4gdGhlIHRldHJhaGVkcm9uLlxuICAgICAqL1xuICAgIF9jb250YWluc1RldHJhaGVkcm9uOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcbiAgICAgICAgdmFyIGEgPSBzaW1wbGV4WzNdO1xuICAgICAgICB2YXIgRG90ID0gYS5jb25zdHJ1Y3Rvci5Eb3Q7XG4gICAgICAgIHZhciBDcm9zcyA9IGEuY29uc3RydWN0b3IuQ3Jvc3M7XG4gICAgICAgIHZhciBiID0gc2ltcGxleFsyXTtcbiAgICAgICAgdmFyIGMgPSBzaW1wbGV4WzFdO1xuICAgICAgICB2YXIgZCA9IHNpbXBsZXhbMF07XG4gICAgICAgIHZhciBhYiA9IGIuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhYyA9IGMuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhZCA9IGQuc3VidHJhY3QoYSk7XG4gICAgICAgIHZhciBhbyA9IGEuc2NhbGUoLTEpO1xuXG4gICAgICAgIHZhciBhYmMgPSBDcm9zcyhhYiwgYWMpO1xuICAgICAgICB2YXIgYWNkID0gQ3Jvc3MoYWMsIGFkKTtcbiAgICAgICAgdmFyIGFkYiA9IENyb3NzKGFkLCBhYik7XG5cbiAgICAgICAgdmFyIGFiY1Rlc3QgPSAweDEsXG4gICAgICAgICAgICBhY2RUZXN0ID0gMHgyLFxuICAgICAgICAgICAgYWRiVGVzdCA9IDB4NDtcblxuICAgICAgICB2YXIgcGxhbmVUZXN0cyA9IChEb3QoYWJjLCBhbykgPiAwID8gYWJjVGVzdCA6IDApIHxcbiAgICAgICAgICAgIChEb3QoYWNkLCBhbykgPiAwID8gYWNkVGVzdCA6IDApIHxcbiAgICAgICAgICAgIChEb3QoYWRiLCBhbykgPiAwID8gYWRiVGVzdCA6IDApO1xuXG4gICAgICAgIHN3aXRjaCAocGxhbmVUZXN0cykge1xuICAgICAgICAgICAgY2FzZSBhYmNUZXN0OlxuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1RldHJhaGVkcm9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWNkVGVzdDpcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gYztcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gZDtcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWMsIGFkLCBhY2QsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFkYlRlc3Q6XG4gICAgICAgICAgICAgICAgLy9pbiBmcm9udCBvZiB0cmlhbmdsZSBBREJcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gYjtcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gZDtcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWQsIGFiLCBhZGIsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBjYXNlIGFiY1Rlc3QgfCBhY2RUZXN0OlxuICAgICAgICAgICAgICAgIHJldHVybiB0aGlzLl9jaGVja1R3b1RldHJhaGVkcm9uKGFvLCBhYiwgYWMsIGFiYywgZGlyLCBzaW1wbGV4KTtcbiAgICAgICAgICAgIGNhc2UgYWNkVGVzdCB8IGFkYlRlc3Q6XG4gICAgICAgICAgICAgICAgc2ltcGxleFsyXSA9IGM7XG4gICAgICAgICAgICAgICAgc2ltcGxleFsxXSA9IGQ7XG4gICAgICAgICAgICAgICAgc2ltcGxleFswXSA9IGI7XG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NoZWNrVHdvVGV0cmFoZWRyb24oYW8sIGFjLCBhZCwgYWNkLCBkaXIsIHNpbXBsZXgpO1xuICAgICAgICAgICAgY2FzZSBhZGJUZXN0IHwgYWJjVGVzdDpcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzFdID0gYjtcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzJdID0gZDtcbiAgICAgICAgICAgICAgICBzaW1wbGV4WzBdID0gYztcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUd29UZXRyYWhlZHJvbihhbywgYWQsIGFiLCBhZGIsIGRpciwgc2ltcGxleCk7XG4gICAgICAgICAgICBkZWZhdWx0OlxuICAgICAgICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG5cbiAgICAgICAgLy9vcmlnaW4gaW4gdGV0cmFoZWRyb25cbiAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIEBtZXRob2QgX2NoZWNrVHdvVGV0cmFoZWRyb25cbiAgICAgKiBAcHJpdmF0ZVxuICAgICAqL1xuICAgIF9jaGVja1R3b1RldHJhaGVkcm9uOiBmdW5jdGlvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCkge1xuICAgICAgICB2YXIgYWJjX2FjID0gYWJjLmNvbnN0cnVjdG9yLkNyb3NzKGFiYywgYWMpO1xuXG4gICAgICAgIGlmIChhYmNfYWMuY29uc3RydWN0b3IuRG90KGFiY19hYywgYW8pID4gMCkge1xuICAgICAgICAgICAgLy90aGUgb3JpZ2luIGlzIGJleW9uZCBBQyBmcm9tIEFCQydzXG4gICAgICAgICAgICAvL3BlcnNwZWN0aXZlLCBlZmZlY3RpdmVseSBleGNsdWRpbmdcbiAgICAgICAgICAgIC8vQUNEIGZyb20gY29uc2lkZXJhdGlvblxuXG4gICAgICAgICAgICAvL3dlIHRodXMgbmVlZCB0ZXN0IG9ubHkgQUNEXG4gICAgICAgICAgICBzaW1wbGV4WzJdID0gc2ltcGxleFsxXTtcbiAgICAgICAgICAgIHNpbXBsZXhbMV0gPSBzaW1wbGV4WzBdO1xuXG4gICAgICAgICAgICBhYiA9IHNpbXBsZXhbMl0uc3VidHJhY3Qoc2ltcGxleFszXSk7XG4gICAgICAgICAgICBhYyA9IHNpbXBsZXhbMV0uc3VidHJhY3Qoc2ltcGxleFszXSk7XG4gICAgICAgICAgICBhYmMgPSBhYi5jb25zdHJ1Y3Rvci5Dcm9zcyhhYiwgYWMpO1xuXG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY2hlY2tUZXRyYWhlZHJvbihhbywgYWIsIGFjLCBhYmMsIGRpciwgc2ltcGxleCk7XG4gICAgICAgIH1cblxuICAgICAgICB2YXIgYWJfYWJjID0gYWJjLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYmMpO1xuXG4gICAgICAgIGlmIChhYl9hYmMuY29uc3RydWN0b3IuRG90KGFiX2FiYywgYW8pID4gMCkge1xuXG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAyKTtcbiAgICAgICAgICAgIC8vZGlyIGlzIG5vdCBhYl9hYmMgYmVjYXVzZSBpdCdzIG5vdCBwb2ludCB0b3dhcmRzIHRoZSBvcmlnaW47XG4gICAgICAgICAgICAvL0FCeEEweEFCIGRpcmVjdGlvbiB3ZSBhcmUgbG9va2luZyBmb3JcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbSh0aGlzLl9nZXROb3JtYWwoYWIsIGFvLCBhYikpO1xuXG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgIH1cblxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBAbWV0aG9kIF9jaGVja1RldHJhaGVkcm9uXG4gICAgICogQHByaXZhdGVcbiAgICAgKi9cbiAgICBfY2hlY2tUZXRyYWhlZHJvbjogZnVuY3Rpb24oYW8sIGFiLCBhYywgYWJjLCBkaXIsIHNpbXBsZXgpIHtcblxuICAgICAgICB2YXIgYWNwID0gYWJjLmNvbnN0cnVjdG9yLkNyb3NzKGFiYywgYWMpO1xuXG4gICAgICAgIGlmIChhY3AuY29uc3RydWN0b3IuRG90KGFjcCwgYW8pID4gMCkge1xuXG4gICAgICAgICAgICBzaW1wbGV4WzJdID0gc2ltcGxleFszXTtcbiAgICAgICAgICAgIHNpbXBsZXguc3BsaWNlKDMsIDEpO1xuICAgICAgICAgICAgc2ltcGxleC5zcGxpY2UoMCwgMSk7XG4gICAgICAgICAgICAvL2RpciBpcyBub3QgYWJjX2FjIGJlY2F1c2UgaXQncyBub3QgcG9pbnQgdG93YXJkcyB0aGUgb3JpZ2luO1xuICAgICAgICAgICAgLy9BQ3hBMHhBQyBkaXJlY3Rpb24gd2UgYXJlIGxvb2tpbmcgZm9yXG4gICAgICAgICAgICBkaXIuY29weUZyb20odGhpcy5fZ2V0Tm9ybWFsKGFjLCBhbywgYWMpKTtcblxuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgLy9hbG1vc3QgdGhlIHNhbWUgbGlrZSB0cmlhbmdsZSBjaGVja3NcbiAgICAgICAgdmFyIGFiX2FiYyA9IGFiLmNvbnN0cnVjdG9yLkNyb3NzKGFiLCBhYmMpO1xuXG4gICAgICAgIGlmIChhYl9hYmMuY29uc3RydWN0b3IuRG90KGFiX2FiYywgYW8pID4gMCkge1xuXG4gICAgICAgICAgICBzaW1wbGV4LnNwbGljZSgwLCAyKTtcbiAgICAgICAgICAgIC8vZGlyIGlzIG5vdCBhYl9hYmMgYmVjYXVzZSBpdCdzIG5vdCBwb2ludCB0b3dhcmRzIHRoZSBvcmlnaW47XG4gICAgICAgICAgICAvL0FCeEEweEFCIGRpcmVjdGlvbiB3ZSBhcmUgbG9va2luZyBmb3JcbiAgICAgICAgICAgIGRpci5jb3B5RnJvbSh0aGlzLl9nZXROb3JtYWwoYWIsIGFvLCBhYikpO1xuXG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgIH1cblxuICAgICAgICAvL2J1aWxkIG5ldyB0ZXRyYWhlZHJvbiB3aXRoIG5ldyBiYXNlXG4gICAgICAgIHNpbXBsZXguc3BsaWNlKDAsIDEpO1xuXG4gICAgICAgIGRpci5jb3B5RnJvbShhYmMpO1xuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQWRkcyBhZGdlIHRvIHRoZSBsaXN0IGFuZCBjaGVja3MgaWYgdGhlIGVkZ2UgaWYgbm90IGluLlxuICAgICAqXG4gICAgICogQG1ldGhvZCBfYWRkRWRnZVxuICAgICAqIEBwcml2YXRlXG4gICAgICogQHBhcmFtIHtPYmplY3RbXX0gZWRnZXMgVGhlIGVkZ2VzLlxuICAgICAqIEBwYXJhbSB7T2JqZWN0fSBkaXIgVGhlIGVkZ2UgdG8gY2hlY2suXG4gICAgICovXG4gICAgX2FkZEVkZ2U6IGZ1bmN0aW9uKGVkZ2VzLCBlZGdlKSB7XG4gICAgICAgIGZvciAodmFyIGogPSAwOyBqIDwgZWRnZXMubGVuZ3RoOyBqKyspIHtcbiAgICAgICAgICAgIGlmIChlZGdlc1tqXS5hID09PSBlZGdlLmIgJiYgZWRnZXNbal0uYiA9PT0gZWRnZS5hKSB7XG4gICAgICAgICAgICAgICAgZWRnZXMuc3BsaWNlKGosIDEpO1xuICAgICAgICAgICAgICAgIHJldHVybjtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgICBlZGdlcy5wdXNoKGVkZ2UpO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBUaGUgc3VwcG9ydCBmdW5jdGlvbi5cbiAgICAgKlxuICAgICAqIEBtZXRob2Qgc3VwcG9ydFxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpcmVjdGlvbiBUaGUgZGlyZWN0aW9uLlxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBzdXBwb3J0IHBvaW50cy5cbiAgICAgKi9cbiAgICBzdXBwb3J0OiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIGRpcmVjdGlvbikge1xuICAgICAgICAvLyBkIGlzIGEgdmVjdG9yIGRpcmVjdGlvbiAoZG9lc24ndCBoYXZlIHRvIGJlIG5vcm1hbGl6ZWQpXG4gICAgICAgIC8vIGdldCBwb2ludHMgb24gdGhlIGVkZ2Ugb2YgdGhlIHNoYXBlcyBpbiBvcHBvc2l0ZSBkaXJlY3Rpb25zXG4gICAgICAgIGRpcmVjdGlvbi5ub3JtYWxpemUoKTtcbiAgICAgICAgdmFyIHAxID0gdGhpcy5fZ2V0RmFydGhlc3RQb2ludEluRGlyZWN0aW9uKGNvbGxpZGVyUG9pbnRzLCBkaXJlY3Rpb24pO1xuICAgICAgICB2YXIgcDIgPSB0aGlzLl9nZXRGYXJ0aGVzdFBvaW50SW5EaXJlY3Rpb24oY29sbGlkZWRQb2ludHMsIGRpcmVjdGlvbi5zY2FsZSgtMSkpO1xuICAgICAgICAvLyBwZXJmb3JtIHRoZSBNaW5rb3dza2kgRGlmZmVyZW5jZVxuICAgICAgICB2YXIgcDMgPSBwMS5zdWJ0cmFjdChwMik7XG4gICAgICAgIC8vIHAzIGlzIG5vdyBhIHBvaW50IGluIE1pbmtvd3NraSBzcGFjZSBvbiB0aGUgZWRnZSBvZiB0aGUgTWlua293c2tpIERpZmZlcmVuY2VcbiAgICAgICAgcmV0dXJuIHAzO1xuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBDaGVja3MgaWYgdGhlIHNpbXBsZXggY29udGFpbnMgdGhlIG9yaWdpbi5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgY29udGFpbnNPcmlnaW5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gc2ltcGxleFRoZSBzaW1wbGV4IG9yIGZhbHNlIGlmIG5vIGludGVyc2VjdGlvbi5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IGRpciBUaGUgZGlyZWN0aW9uIHRvIHRlc3QuXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gQ29udGFpbnMgb3Igbm90LlxuICAgICAqL1xuICAgIGNvbnRhaW5zT3JpZ2luOiBmdW5jdGlvbihzaW1wbGV4LCBkaXIpIHtcbiAgICAgICAgaWYgKHNpbXBsZXgubGVuZ3RoID09PSAyKSB7XG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY29udGFpbnNMaW5lKHNpbXBsZXgsIGRpcik7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHNpbXBsZXgubGVuZ3RoID09PSAzKSB7XG4gICAgICAgICAgICByZXR1cm4gdGhpcy5fY29udGFpbnNUcmlhbmdsZShzaW1wbGV4LCBkaXIpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChzaW1wbGV4Lmxlbmd0aCA9PT0gNCkge1xuICAgICAgICAgICAgcmV0dXJuIHRoaXMuX2NvbnRhaW5zVGV0cmFoZWRyb24oc2ltcGxleCwgZGlyKTtcbiAgICAgICAgfVxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIFRoZSBHSksgKEdpbGJlcnTigJNKb2huc29u4oCTS2VlcnRoaSkgYWxnb3JpdGhtLlxuICAgICAqIENvbXB1dGVzIHN1cHBvcnQgcG9pbnRzIHRvIGJ1aWxkIHRoZSBNaW5rb3dza3kgZGlmZmVyZW5jZSBhbmRcbiAgICAgKiBjcmVhdGUgYSBzaW1wbGV4LiBUaGUgcG9pbnRzIG9mIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdFxuICAgICAqIG11c3QgYmUgY29udmV4ZS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgY2hlY2tcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBUaGUgc2ltcGxleCBvciBmYWxzZSBpZiBubyBpbnRlcnNlY3Rpb24uXG4gICAgICovXG4gICAgY2hlY2s6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cykge1xuICAgICAgICB2YXIgaXQgPSAwO1xuICAgICAgICB2YXIgYSwgc2ltcGxleCA9IFtdO1xuXG4gICAgICAgIHZhciBjb2xsaWRlckNlbnRlciA9IHRoaXMuX2dldEJhcnljZW50ZXIoY29sbGlkZXJQb2ludHMpO1xuICAgICAgICB2YXIgY29sbGlkZWRDZW50ZXIgPSB0aGlzLl9nZXRCYXJ5Y2VudGVyKGNvbGxpZGVkUG9pbnRzKTtcblxuICAgICAgICAvLyBpbml0aWFsIGRpcmVjdGlvbiBmcm9tIHRoZSBjZW50ZXIgb2YgMXN0IGJvZHkgdG8gdGhlIGNlbnRlciBvZiAybmQgYm9keVxuICAgICAgICB2YXIgZGlyID0gY29sbGlkZXJDZW50ZXIuc3VidHJhY3QoY29sbGlkZWRDZW50ZXIpLm5vcm1hbGl6ZSgpO1xuXG4gICAgICAgIC8vIGlmIGluaXRpYWwgZGlyZWN0aW9uIGlzIHplcm8g4oCTIHNldCBpdCB0byBhbnkgYXJiaXRyYXJ5IGF4aXMgKHdlIGNob29zZSBYKVxuICAgICAgICBpZiAoZGlyLmxlbmd0aFNxdWFyZWQoKSA8IHRoaXMuRVBTSUxPTikge1xuICAgICAgICAgICAgZGlyLnggPSAxO1xuICAgICAgICB9XG5cbiAgICAgICAgLy8gc2V0IHRoZSBmaXJzdCBzdXBwb3J0IGFzIGluaXRpYWwgcG9pbnQgb2YgdGhlIG5ldyBzaW1wbGV4XG4gICAgICAgIGEgPSBzaW1wbGV4WzBdID0gdGhpcy5zdXBwb3J0KGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgZGlyKTtcblxuICAgICAgICBpZiAoYS5jb25zdHJ1Y3Rvci5Eb3QoYSwgZGlyKSA8PSAwKSB7XG4gICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgIH1cblxuICAgICAgICBkaXIuc2NhbGVJblBsYWNlKC0xLCBkaXIpOyAvLyB3ZSB3aWxsIGJlIHNlYXJjaGluZyBpbiB0aGUgb3Bwb3NpdGUgZGlyZWN0aW9uIG5leHRcblxuICAgICAgICB2YXIgbWF4ID0gY29sbGlkZWRQb2ludHMubGVuZ3RoICogY29sbGlkZXJQb2ludHMubGVuZ3RoO1xuICAgICAgICB3aGlsZSAoaXQgPCBtYXgpIHtcbiAgICAgICAgICAgIGlmIChkaXIubGVuZ3RoU3F1YXJlZCgpID09PSAwICYmIHNpbXBsZXgubGVuZ3RoID49IDIpIHtcbiAgICAgICAgICAgICAgICAvLyBHZXQgcGVycGVuZGljdWxhciBkaXJlY3Rpb24gdG8gbGFzdCBzaW1wbGV4XG4gICAgICAgICAgICAgICAgZGlyID0gc2ltcGxleFtzaW1wbGV4Lmxlbmd0aCAtIDFdLnN1YnRyYWN0KHNpbXBsZXhbc2ltcGxleC5sZW5ndGggLSAyXSk7XG4gICAgICAgICAgICAgICAgdmFyIHRtcCA9IGRpci55O1xuICAgICAgICAgICAgICAgIGRpci55ID0gLWRpci54O1xuICAgICAgICAgICAgICAgIGRpci54ID0gdG1wO1xuICAgICAgICAgICAgfVxuXG4gICAgICAgICAgICBhID0gdGhpcy5zdXBwb3J0KGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgZGlyKTtcblxuICAgICAgICAgICAgLy8gbWFrZSBzdXJlIHRoYXQgdGhlIGxhc3QgcG9pbnQgd2UgYWRkZWQgYWN0dWFsbHkgcGFzc2VkIHRoZSBvcmlnaW5cbiAgICAgICAgICAgIGlmIChhLmNvbnN0cnVjdG9yLkRvdChhLCBkaXIpIDw9IDApIHtcbiAgICAgICAgICAgICAgICAvLyBpZiB0aGUgcG9pbnQgYWRkZWQgbGFzdCB3YXMgbm90IHBhc3QgdGhlIG9yaWdpbiBpbiB0aGUgZGlyZWN0aW9uIG9mIGRcbiAgICAgICAgICAgICAgICAvLyB0aGVuIHRoZSBNaW5rb3dza2kgU3VtIGNhbm5vdCBwb3NzaWJseSBjb250YWluIHRoZSBvcmlnaW4gc2luY2VcbiAgICAgICAgICAgICAgICAvLyB0aGUgbGFzdCBwb2ludCBhZGRlZCBpcyBvbiB0aGUgZWRnZSBvZiB0aGUgTWlua293c2tpIERpZmZlcmVuY2VcbiAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgICAgICAgICB9XG5cbiAgICAgICAgICAgIHNpbXBsZXgucHVzaChhKTtcbiAgICAgICAgICAgIC8vIG90aGVyd2lzZSB3ZSBuZWVkIHRvIGRldGVybWluZSBpZiB0aGUgb3JpZ2luIGlzIGluXG4gICAgICAgICAgICAvLyB0aGUgY3VycmVudCBzaW1wbGV4XG5cbiAgICAgICAgICAgIGlmICh0aGlzLmNvbnRhaW5zT3JpZ2luKHNpbXBsZXgsIGRpcikpIHtcbiAgICAgICAgICAgICAgICByZXR1cm4gc2ltcGxleDtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGl0Kys7XG4gICAgICAgIH1cbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogRmluZHMgdGhlIHJlc3BvbnNlIHdpdGggdGhlIHNpbXBsZXggKGVkZ2VzKSBvZiB0aGUgZ2prIGFsZ29yaXRobS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgZmluZFJlc3BvbnNlV2l0aEVkZ2VcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMltdfSBjb2xsaWRlclBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlciBvYmplY3QuXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyW119IHNpbXBsZXggVGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtudW1iZXJbXX0gW2V4Y2VwdF0gRXhjZXB0aW9uczogZWRnZSBpbmRpY2VzIHRvIGlnbm9yZVxuICAgICAqIEByZXR1cm4ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjN9IFRoZSBwZW5ldHJhdGlvbiB2ZWN0b3IuXG4gICAgICovXG4gICAgZmluZFJlc3BvbnNlV2l0aEVkZ2U6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCwgZXhjZXB0KSB7XG4gICAgICAgIGlmIChleGNlcHQgPT09IHVuZGVmaW5lZCkgZXhjZXB0ID0gW107XG5cbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLl9nZXROZWFyZXN0RWRnZShzaW1wbGV4KTtcbiAgICAgICAgaWYgKGVkZ2UuaW5kZXggPT09IHVuZGVmaW5lZCkgcmV0dXJuIGZhbHNlO1xuXG4gICAgICAgIHZhciBzdXAgPSB0aGlzLnN1cHBvcnQoY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBlZGdlLm5vcm1hbCk7IC8vZ2V0IHN1cHBvcnQgcG9pbnQgaW4gZGlyZWN0aW9uIG9mIGVkZ2UncyBub3JtYWxcbiAgICAgICAgdmFyIGQgPSBNYXRoLmFicyhzdXAuY29uc3RydWN0b3IuRG90KHN1cCwgZWRnZS5ub3JtYWwpKTtcblxuICAgICAgICBpZiAoZCAtIGVkZ2UuZGlzdGFuY2UgPD0gdGhpcy5FUFNJTE9OKSB7XG4gICAgICAgICAgICBleGNlcHQucHVzaChlZGdlLmluZGV4KTtcbiAgICAgICAgICAgIHJldHVybiBlZGdlLm5vcm1hbC5zY2FsZShlZGdlLmRpc3RhbmNlKTtcbiAgICAgICAgfVxuICAgICAgICAvLyBhZGQgbmV3IHN1cHBvcnQgcG9pbnQgaW4gc2ltcGxleFxuICAgICAgICBzaW1wbGV4LnNwbGljZShlZGdlLmluZGV4LCAwLCBzdXApO1xuICAgICAgICAvLyB1cGRhdGUgaW5kaWNlcyBvZiBlZGdlcyBleGNlcHRpb25zXG4gICAgICAgIHRoaXMuX3VwZGF0ZUV4Y2VwdGlvbnNCeUluc2VydChleGNlcHQsIGVkZ2UuaW5kZXgsIHNpbXBsZXgpO1xuXG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogRmluZHMgdGhlIHJlc3BvbnNlIHdpdGggdGhlIHBvbHl0b3BlIGRvbmUgd2l0aCB0aGUgc2ltcGxleCBvZiB0aGUgZ2prIGFsZ29yaXRobS5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgZmluZFJlc3BvbnNlV2l0aFRyaWFuZ2xlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVkUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVkIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge1RyaWFuZ2xlW119IHBvbHl0b3BlIFRoZSBwb2x5dG9wZSBkb25lIHdpdGggdGhlIHNpbXBsZXguXG4gICAgICogQHBhcmFtIHtudW1iZXJbXX0gW2V4Y2VwdF0gRXhjZXB0aW9uczogdHJpYW5nbGUgaW5kaWNlcyB0byBpZ25vcmVcbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZTogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBwb2x5dG9wZSwgZXhjZXB0KSB7XG4gICAgICAgIGlmIChleGNlcHQgPT09IHVuZGVmaW5lZCkgZXhjZXB0ID0gW107XG5cbiAgICAgICAgaWYgKHBvbHl0b3BlLmxlbmd0aCA9PT0gMCkge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG5cbiAgICAgICAgdmFyIG5lYXJlc3QgPSB0aGlzLl9nZXROZWFyZXN0VHJpYW5nbGUocG9seXRvcGUpO1xuICAgICAgICBpZiAobmVhcmVzdC5pbmRleCA9PT0gdW5kZWZpbmVkKSByZXR1cm4gZmFsc2U7XG5cbiAgICAgICAgdmFyIHRyaWFuZ2xlID0gcG9seXRvcGVbbmVhcmVzdC5pbmRleF07XG5cbiAgICAgICAgdmFyIHN1cCA9IHRoaXMuc3VwcG9ydChjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHRyaWFuZ2xlLm4pO1xuXG4gICAgICAgIHZhciBkID0gTWF0aC5hYnMoc3VwLmNvbnN0cnVjdG9yLkRvdChzdXAsIHRyaWFuZ2xlLm4pKTtcblxuICAgICAgICBpZiAoKGQgLSBuZWFyZXN0LmRpc3RhbmNlIDw9IHRoaXMuRVBTSUxPTikpIHtcbiAgICAgICAgICAgIGV4Y2VwdC5wdXNoKG5lYXJlc3QuaW5kZXgpO1xuICAgICAgICAgICAgcmV0dXJuIHRyaWFuZ2xlLm4uc2NhbGUobmVhcmVzdC5kaXN0YW5jZSk7XG4gICAgICAgIH1cblxuICAgICAgICB2YXIgZWRnZXMgPSBbXTtcbiAgICAgICAgZm9yICh2YXIgaSA9IHBvbHl0b3BlLmxlbmd0aCAtIDE7IGkgPj0gMDsgaS0tKSB7XG4gICAgICAgICAgICB0cmlhbmdsZSA9IHBvbHl0b3BlW2ldO1xuICAgICAgICAgICAgLy8gY2FuIHRoaXMgZmFjZSBiZSAnc2VlbicgYnkgZW50cnlfY3VyX3N1cHBvcnQ/XG4gICAgICAgICAgICBpZiAodHJpYW5nbGUubi5jb25zdHJ1Y3Rvci5Eb3QodHJpYW5nbGUubiwgc3VwLnN1YnRyYWN0KHBvbHl0b3BlW2ldLmEpKSA+IDApIHtcbiAgICAgICAgICAgICAgICB0aGlzLl9hZGRFZGdlKGVkZ2VzLCB7XG4gICAgICAgICAgICAgICAgICAgIGE6IHRyaWFuZ2xlLmEsXG4gICAgICAgICAgICAgICAgICAgIGI6IHRyaWFuZ2xlLmJcbiAgICAgICAgICAgICAgICB9KTtcbiAgICAgICAgICAgICAgICB0aGlzLl9hZGRFZGdlKGVkZ2VzLCB7XG4gICAgICAgICAgICAgICAgICAgIGE6IHRyaWFuZ2xlLmIsXG4gICAgICAgICAgICAgICAgICAgIGI6IHRyaWFuZ2xlLmNcbiAgICAgICAgICAgICAgICB9KTtcbiAgICAgICAgICAgICAgICB0aGlzLl9hZGRFZGdlKGVkZ2VzLCB7XG4gICAgICAgICAgICAgICAgICAgIGE6IHRyaWFuZ2xlLmMsXG4gICAgICAgICAgICAgICAgICAgIGI6IHRyaWFuZ2xlLmFcbiAgICAgICAgICAgICAgICB9KTtcbiAgICAgICAgICAgICAgICAvLyByZW1vdmUgYSB0cmlhbmdsZVxuICAgICAgICAgICAgICAgIHBvbHl0b3BlLnNwbGljZShpLCAxKTtcbiAgICAgICAgICAgICAgICAvLyB1cGRhdGUgZXhjZXB0aW9ucyB3aXRoIHJlbW92ZWQgdHJpYW5nbGUgaW5kZXhcbiAgICAgICAgICAgICAgICB0aGlzLl9yZW1vdmVFeGNlcHRpb24oZXhjZXB0LCBpKTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuXG4gICAgICAgIC8vIGNyZWF0ZSBuZXcgdHJpYW5nbGVzIGZyb20gdGhlIGVkZ2VzIGluIHRoZSBlZGdlIGxpc3RcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBlZGdlcy5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgdHJpYW5nbGUgPSBuZXcgVHJpYW5nbGUoc3VwLCBlZGdlc1tpXS5hLCBlZGdlc1tpXS5iKTtcbiAgICAgICAgICAgIGlmICh0cmlhbmdsZS5uLmxlbmd0aCgpICE9PSAwKSB7XG4gICAgICAgICAgICAgICAgcG9seXRvcGUucHVzaCh0cmlhbmdsZSk7XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIFVwZGF0ZSBleGNlcHRpb24gZnJvbSBpbnNlcnRpb24gb2YgbmV3IGVkZ2VcbiAgICAgKiBAcGFyYW0ge251bWJlcltdfSBleGNlcHQgXG4gICAgICogQHBhcmFtIHtudW1iZXJ9IGVkZ2VJbmRleCBpbnNlcnRlZCBlZGdlIGluZGV4XG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJbXX0gc2ltcGxleCBcbiAgICAgKi9cbiAgICBfdXBkYXRlRXhjZXB0aW9uc0J5SW5zZXJ0OiBmdW5jdGlvbihleGNlcHQsIGVkZ2VJbmRleCwgc2ltcGxleCkge1xuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGV4Y2VwdC5sZW5ndGg7IGkrKykge1xuICAgICAgICAgICAgaWYgKGV4Y2VwdFtpXSA+PSBlZGdlSW5kZXgpIHtcbiAgICAgICAgICAgICAgICBleGNlcHRbaV0gPSAoZXhjZXB0W2ldICsgMSkgJSBzaW1wbGV4Lmxlbmd0aDtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBVcGRhdGUgZXhjZXB0aW9ucyB3aGVuIGFuIGluZGV4IGhhcyBiZWVuIHJlbW92ZWRcbiAgICAgKiBAcGFyYW0ge251bWJlcltdfSBleGNlcHQgXG4gICAgICogQHBhcmFtIHtudW1iZXJ9IGluZGV4VG9SZW1vdmUgXG4gICAgICovXG4gICAgX3JlbW92ZUV4Y2VwdGlvbjogZnVuY3Rpb24oZXhjZXB0LCBpbmRleFRvUmVtb3ZlKSB7XG4gICAgICAgIGZvciAodmFyIGkgPSBleGNlcHQubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pIHtcbiAgICAgICAgICAgIGlmKGV4Y2VwdFtpXSA9PT0gaW5kZXhUb1JlbW92ZSkge1xuICAgICAgICAgICAgICAgIGV4Y2VwdC5zcGxpY2UoaSwgMSk7XG4gICAgICAgICAgICB9IGVsc2UgaWYgKGV4Y2VwdFtpXSA+IGluZGV4VG9SZW1vdmUpIHtcbiAgICAgICAgICAgICAgICBleGNlcHRbaV0tLTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgIH0sXG5cbiAgICAvKipcbiAgICAgKiBHZXRzIHRoZSByZXNwb25zZSBvZiB0aGUgcGVuZXRyYXRpb24gdmVjdG9yIHdpdGggdGhlIHNpbXBsZXguXG4gICAgICpcbiAgICAgKiBAbWV0aG9kIGdldFJlc3BvbnNlXG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IGNvbGxpZGVyUG9pbnRzIFRoZSBjb252ZXhlIGNvbGxpZGVyIG9iamVjdC5cbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZWRQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZWQgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBzaW1wbGV4IFRoZSBzaW1wbGV4IG9mIHRoZSBNaW5rb3dza3kgZGlmZmVyZW5jZS5cbiAgICAgKiBAcmV0dXJuIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzfSBUaGUgcGVuZXRyYXRpb24gdmVjdG9yLlxuICAgICAqL1xuICAgIGdldFJlc3BvbnNlOiBmdW5jdGlvbihjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHNpbXBsZXgpIHtcbiAgICAgICAgdmFyIHJlc3BvbnNlcyA9IHRoaXMuZ2V0UmVzcG9uc2VzKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCk7XG4gICAgICAgIGlmICghcmVzcG9uc2VzKSByZXR1cm4gcmVzcG9uc2VzO1xuICAgICAgICBpZiAocmVzcG9uc2VzWzBdKSByZXR1cm4gcmVzcG9uc2VzWzBdO1xuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfSxcblxuICAgIGdldFJlc3BvbnNlczogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzLCBzaW1wbGV4LCBvcHRpb25zKSB7XG4gICAgICAgIGlmKG9wdGlvbnMgPT09IHVuZGVmaW5lZCkgb3B0aW9ucyA9IHt9O1xuICAgICAgICBvcHRpb25zID0gT2JqZWN0LmFzc2lnbih7IG1heFJlc3BvbnNlczogLTEgfSwgb3B0aW9ucyk7XG5cbiAgICAgICAgdmFyIGl0ID0gMCxcbiAgICAgICAgICAgIHJlc3BvbnNlO1xuICAgICAgICB2YXIgcG9seXRvcGUgPSBzaW1wbGV4WzBdLnogIT09IHVuZGVmaW5lZCA/IFtuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFsxXSwgc2ltcGxleFsyXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFsyXSwgc2ltcGxleFszXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFswXSwgc2ltcGxleFszXSwgc2ltcGxleFsxXSksXG4gICAgICAgICAgICBuZXcgVHJpYW5nbGUoc2ltcGxleFsxXSwgc2ltcGxleFszXSwgc2ltcGxleFsyXSlcbiAgICAgICAgXSA6IG51bGw7XG5cbiAgICAgICAgdmFyIG1heCA9IGNvbGxpZGVkUG9pbnRzLmxlbmd0aCAqIGNvbGxpZGVyUG9pbnRzLmxlbmd0aDtcbiAgICAgICAgdmFyIGV4Y2VwdCA9IFtdO1xuICAgICAgICB2YXIgcmVzcG9uc2VzID0gW107XG4gICAgICAgIHdoaWxlIChpdCA8IG1heCAmJiByZXNwb25zZXMubGVuZ3RoICE9PSBvcHRpb25zLm1heFJlc3BvbnNlcykge1xuICAgICAgICAgICAgaWYgKHNpbXBsZXhbMF0ueiA9PT0gdW5kZWZpbmVkKSB7XG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhFZGdlKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCwgZXhjZXB0KTtcbiAgICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAgICAgcmVzcG9uc2UgPSB0aGlzLmZpbmRSZXNwb25zZVdpdGhUcmlhbmdsZShjb2xsaWRlclBvaW50cywgY29sbGlkZWRQb2ludHMsIHBvbHl0b3BlLCBleGNlcHQpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgICAgaWYgKHJlc3BvbnNlKSB7XG4gICAgICAgICAgICAgICAgdmFyIG5vcm0gPSByZXNwb25zZS5jbG9uZSgpLm5vcm1hbGl6ZSgpLnNjYWxlSW5QbGFjZSh0aGlzLkVQU0lMT04pO1xuICAgICAgICAgICAgICAgIHZhciByZXBvbnNlV2l0aE9mZnNldCA9IHJlc3BvbnNlLmFkZFRvUmVmKG5vcm0sIHJlc3BvbnNlKTtcbiAgICAgICAgICAgICAgICB0aGlzLl9hZGRSZXNwb25zZShyZXNwb25zZXMsIHJlcG9uc2VXaXRoT2Zmc2V0KTtcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGl0Kys7XG4gICAgICAgIH1cbiAgICAgICAgcmV0dXJuIHJlc3BvbnNlcztcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQWRkIHJlc3BvbnNlIHRvIHJlc3BvbnNlcyBpZiBub3QgYWxyZWFkeSBwcmVzZW50XG4gICAgICogQHBhcmFtIHtCQUJZTE9OLlZlY3RvcjJ8QkFCWUxPTi5WZWN0b3IzW119IHJlc3BvbnNlc1xuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gcmVzcG9uc2VcbiAgICAgKi9cbiAgICBfYWRkUmVzcG9uc2U6IGZ1bmN0aW9uKHJlc3BvbnNlcywgcmVzcG9uc2UpIHtcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCByZXNwb25zZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgICAgIGlmIChyZXNwb25zZXNbaV0uc3VidHJhY3QocmVzcG9uc2UpLmxlbmd0aFNxdWFyZWQgPCB0aGlzLkVQU0lMT04pIHtcbiAgICAgICAgICAgICAgICByZXR1cm47XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cblxuICAgICAgICByZXNwb25zZXMucHVzaChyZXNwb25zZSk7XG4gICAgfSxcblxuICAgIC8qKlxuICAgICAqIENoZWNrcyBpZiB0aGUgY29sbGlkZXIgYW5kIHRoZSBjb2xsaWRlZCBvYmplY3QgYXJlIGludGVyc2VjdGluZy5cbiAgICAgKlxuICAgICAqIEBtZXRob2QgaXNJbnRlcnNlY3RpbmdcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHJldHVybiB7Qm9vbGVhbn0gSXMgaW50ZXJzZWN0aW5nIG9yIG5vdC5cbiAgICAgKi9cbiAgICBpc0ludGVyc2VjdGluZzogZnVuY3Rpb24oY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKSB7XG4gICAgICAgIHJldHVybiAhIXRoaXMuY2hlY2soY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKTtcbiAgICB9LFxuXG4gICAgLyoqXG4gICAgICogQ2hlY2tzIGlmIHRoZSBjb2xsaWRlciBhbmQgdGhlIGNvbGxpZGVkIG9iamVjdCBhcmUgaW50ZXJzZWN0aW5nXG4gICAgICogYW5kIGdpdmUgdGhlIHJlc3BvbnNlIHRvIGJlIG91dCBvZiB0aGUgb2JqZWN0LlxuICAgICAqXG4gICAgICogQG1ldGhvZCBpbnRlcnNlY3RcbiAgICAgKiBAcGFyYW0ge0JBQllMT04uVmVjdG9yMnxCQUJZTE9OLlZlY3RvcjNbXX0gY29sbGlkZXJQb2ludHMgVGhlIGNvbnZleGUgY29sbGlkZXIgb2JqZWN0LlxuICAgICAqIEBwYXJhbSB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM1tdfSBjb2xsaWRlZFBvaW50cyBUaGUgY29udmV4ZSBjb2xsaWRlZCBvYmplY3QuXG4gICAgICogQHJldHVybiB7QkFCWUxPTi5WZWN0b3IyfEJBQllMT04uVmVjdG9yM30gVGhlIHBlbmV0cmF0aW9uIHZlY3Rvci5cbiAgICAgKi9cbiAgICBpbnRlcnNlY3Q6IGZ1bmN0aW9uKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cykge1xuICAgICAgICB2YXIgc2ltcGxleCA9IHRoaXMuY2hlY2soY29sbGlkZXJQb2ludHMsIGNvbGxpZGVkUG9pbnRzKTtcblxuICAgICAgICAvL3RoaXMuY3ViZSA9IHRoaXMuY3ViZSB8fCBbXTtcbiAgICAgICAgaWYgKHNpbXBsZXgpIHtcbiAgICAgICAgICAgIHJldHVybiB0aGlzLmdldFJlc3BvbnNlKGNvbGxpZGVyUG9pbnRzLCBjb2xsaWRlZFBvaW50cywgc2ltcGxleCk7XG4gICAgICAgIH1cblxuICAgICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBDb2xsaXNpb25HamtFcGE7XG4iXX0=
