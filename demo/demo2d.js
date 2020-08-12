'use strict';

var polygons = [],
    response = new BABYLON.Vector2();


var canvas2D = document.getElementById('canvas2d');
var context = canvas2D.getContext('2d');

var resize = function() {
    canvas2D = document.getElementById('canvas2d');
    context = canvas2D.getContext('2d');
    canvas2D.width = canvas2D.offsetWidth;
    canvas2D.height = canvas2D.offsetHeight;
};

window.addEventListener('resize', resize);
window.addEventListener('load', resize);

var roundTo = function(a) {
    return Math.round(a * 1e5) / 1e5;
};

var isPointInPolygon = function(points, x, y) {
    // Détermine si un point est dans un polygone
    // Fonctionne même quand le polygone a des trous

    // Assertion : points[i] et points[(i + 1) % length] forment un coté du polygone
    var rdp0x, rdp0y, rdp1x, rdp1y;
    var pt = {
        x: x,
        y: y
    };
    pt.x = roundTo(pt.x);
    pt.y = roundTo(pt.y);

    var length = points.length;

    for (var c = false, i = 0; i < length; i++) {
        rdp0x = roundTo(points[i].x);
        rdp0y = roundTo(points[i].y);
        rdp1x = roundTo(points[(i + 1) % length].x);
        rdp1y = roundTo(points[(i + 1) % length].y);

        if (rdp1y === rdp0y && rdp1y === pt.y) {
            ((rdp0x <= pt.x && pt.x < rdp1x) || (rdp1x <= pt.x && pt.x < rdp0x)) && (c = !c);
        } else {
            ((rdp0y <= pt.y && pt.y < rdp1y) || (rdp1y <= pt.y && pt.y < rdp0y)) && (pt.x <= roundTo((rdp1x - rdp0x) * (pt.y - rdp0y) / (rdp1y - rdp0y) + rdp0x)) && (c = !c);
        }
    }
    return c;
};

function drawPolygon(context, polygon, strokeStyle) {
    context.save();
    context.strokeStyle = strokeStyle || '#0000FF';

    context.beginPath();
    context.save();
    context.lineWidth = 2;
    context.moveTo(polygon[polygon.length - 1].x + 0.5, polygon[polygon.length - 1].y + 0.5); //first vertex
    for (var i = 0; i < polygon.length; i++) {
        context.lineTo(polygon[i].x + 0.5, polygon[i].y + 0.5);
    }
    context.stroke();
    context.restore();

    if (selected2D === polygon && response.length) {
        context.beginPath();
        context.save();
        context.setLineDash([5, 15]);
        context.moveTo(polygon[polygon.length - 1].x - response.x + 0.5, polygon[polygon.length - 1].y - response.y + 0.5); //first vertex
        for (var i = 0; i < polygon.length; i++) {
            context.lineTo(polygon[i].x - response.x + 0.5, polygon[i].y - response.y + 0.5);
        }
        context.stroke();
        context.restore();
    }
}

var load = function() {
    polygons.push(generateRandomPolygon("circle"));
    polygons.push(generateRandomPolygon("quadrilatere"));

    render();
    return;
}

window.addEventListener('load', load);

var getTargeted = function(x, y) {
    for (var i = 0; i < polygons.length; i++) {
        if (isPointInPolygon(polygons[i], x, y)) {
            return polygons[i];
        }
    }

    return false;
}

var selected2D = false;
var callbacks2D = {
    onMouseDown: function(event) {
        selected2D = getTargeted(event.pageX, event.pageY);
    },
    onMouseUp: function(event) {
        selected2D = false;
    },
    onMouseMove: function(event) {
        if (selected2D) {
            for (var i = 0; i < selected2D.length; i++) {
                selected2D[i].x += event.movementX;
                selected2D[i].y += event.movementY;
            }
            response.copyFromFloats(0, 0);
            for (var i = 0; i < polygons.length; i++) {
                if (polygons[i] !== selected2D) {
                    var res = Collisiongjkepa.intersect(selected2D, polygons[i]);
                    if (res) {
                        response.addInPlace(res);
                    }
                }
            }
        }
    }
}

canvas2D.addEventListener('pointermove', callbacks2D.onMouseMove);
canvas2D.addEventListener('pointerdown', callbacks2D.onMouseDown);
canvas2D.addEventListener('pointerup', callbacks2D.onMouseUp);
///ui

var generateRandomPolygon = function(type) {
    var amount = 30 / 100 * Math.min(canvas2D.width, canvas2D.height);
    var polygon = [];
    switch (type) {
        case "triangle":
            polygon = [
                new BABYLON.Vector2(canvas2D.width / 2 - Math.random() * amount, canvas2D.height / 2 - Math.random() * amount),
                new BABYLON.Vector2(canvas2D.width / 2 + Math.random() * amount, canvas2D.height / 2 - Math.random() * amount),
                new BABYLON.Vector2(canvas2D.width / 2, canvas2D.height / 2 + Math.random() * amount),
            ]
            break;
        case "quadrilatere":
            polygon = [
                new BABYLON.Vector2(amount - amount / 2 * Math.random() + canvas2D.width / 2, amount - amount / 2 * Math.random() + canvas2D.height / 2),
                new BABYLON.Vector2(-amount + amount / 2 * Math.random() + canvas2D.width / 2, amount - amount / 2 * Math.random() + canvas2D.height / 2),
                new BABYLON.Vector2(-amount + amount / 2 * Math.random() + canvas2D.width / 2, -amount + amount / 2 * Math.random() + canvas2D.height / 2),
                new BABYLON.Vector2(amount - amount / 2 * Math.random() + canvas2D.width / 2, -amount + amount / 2 * Math.random() + canvas2D.height / 2),
            ]
            break;
        case "circle":
            for (var i = 0; i < Math.PI * 2; i += Math.PI / 8) {
                polygon.push(new BABYLON.Vector2(Math.cos(i) * amount / 2 + canvas2D.width / 2, Math.sin(i) * amount / 2 + canvas2D.height / 2));
            }
            break;
        default:
            break;
    }

    return polygon;
}

window.requestAnimFrame = (function() {
    return window.requestAnimationFrame ||
        window.webkitRequestAnimationFrame ||
        window.mozRequestAnimationFrame ||
        function(callback) {
            window.setTimeout(callback, 1000 / 60);
        };
})();

function render() {
    if (!context) return;

    context.clearRect(0, 0, canvas2D.width, canvas2D.height);

    for (var i = 0; i < polygons.length; i++) {
        if (selected2D === polygons[i]) {
            drawPolygon(context, polygons[i], '#333');
        } else {
            drawPolygon(context, polygons[i], '#888');
        }
    }
}

(function animloop() {
    requestAnimFrame(animloop);
    render();
})();
