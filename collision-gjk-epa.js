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
