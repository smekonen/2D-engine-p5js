/**
    Project: 2D physics engine (work in progress)
		Author: Simon Mekonen, 2018
		
		Thanks to: 
		- https://www.chrishecker.com/images/e/e7/Gdmphys3.pdf
		- Box2D, Erin Catto pdfs
		- Randygaul.net
		
		
    Planned updates:
        * Angular momentum -- DONE
        * Friction -- DONE
        * Polygons
        * More efficient (current efficiency: max 80 bodies)
        * Antigravity shapes
        * Selective collision checking
        * Simpler to use in other programs
        * More efficient contact point selection
    
    Known bugs:
        * Stacking is glitchy. Don't compress objects.
        * No resting state
        * Dont spawn objects inside other objects
*/

//frameRate(30);


new p5();
//angleMode(DEGREES);



var debugging = false;
var showContactPoints = false;
var gravity = true;

var Circle = function(x, y, radius, density, bounciness, angle) {
  this.type = 1;

  this.loc = createVector(x, y);

  var s = radius;
  this.min = createVector(this.loc.x - s + 5, this.loc.y - s + 5);
  this.max = createVector(this.loc.x + s + 5, this.loc.y + s + 5);


  this.rad = radius;
  this.vel = createVector(0, 0);
  this.acc = createVector(0, 0);

  this.angle = angle || 0;
  this.angularVel = 0;
  this.torque = 0;

  this.density = density;

  this.restitution = (bounciness > 1 ? 0.8 : bounciness) || 0.8;
  if (bounciness === 0) {
    this.restitution = 0;
  }

  this.staticFriction = 0.5;
  this.dynamicFriction = 0.1;

  this.mass = radius * radius * this.density;

  if (this.mass > 0) {
    this.momentOfInertia = this.mass * (radius * radius) / 4;
  } else {
    this.momentOfInertia = Number.MAX_SAFE_INTEGER;
  }

  this.invMass = this.mass > 0 ? 1 / this.mass : 0;

};

Circle.prototype.update = function() {
  this.display();


  var s = this.radius;
  this.min = createVector(this.loc.x - s + 5, this.loc.y - s + 5);
  this.max = createVector(this.loc.x + s + 5, this.loc.y + s + 5);

  this.loc.add(this.vel);
  this.vel.add(this.acc);
  this.angle += this.angularVel;
  constrain(this.vel, 0, 15);
  this.acc.mult(0);

  if (this.mass === 0) {
    this.vel.mult(0);
  }
};

Circle.prototype.display = function() {
  noFill();
  stroke(255, 0, 0);
  strokeWeight(6);

  pushMatrix();
  translate(this.loc.x, this.loc.y);
  rotate(this.angle * -1);
  ellipse(0, 0, this.rad * 2, this.rad * 2);
  ellipse(this.rad / 2, this.rad / 2, 4, 4);
  popMatrix();
};

Circle.prototype.applyForce = function(force) {
  var f = force.get();
  this.acc = f;
};




var Box = function(x, y, w, h, density, bounciness, angle) {
  this.type = 0;

  this.loc = createVector(x, y);

  var s = max(w, h);
  this.min = createVector(x - s - 3, y - s - 3); //(check this)
  this.max = createVector(x + s + 3, y + s + 3);

  this.w = w;
  this.h = h;

  this.halfW = w / 2;
  this.halfH = h / 2;

  this.cornerDisp = [
    createVector(-1 * this.halfW, -1 * this.halfH),
    createVector(this.halfW, -1 * this.halfH),
    createVector(this.halfW, this.halfH),
    createVector(-1 * this.halfW, this.halfH)
  ];

  this.verticies = [
    p5.Vector.add(this.loc, this.cornerDisp[0]),
    p5.Vector.add(this.loc, this.cornerDisp[1]),
    p5.Vector.add(this.loc, this.cornerDisp[2]),
    p5.Vector.add(this.loc, this.cornerDisp[3])
  ];




  this.xAxis = createVector(1, 0);
  this.yAxis = createVector(0, 1);

  this.vel = createVector(0, 0);
  this.acc = createVector(0, 0);

  this.angle = angle || 0;
  this.angularVel = 0;
  this.torque = 0;

  for (var i = 0; i < this.cornerDisp.length; i++) {
    this.cornerDisp[i].rotate(this.angle * -1);
  }

  this.xAxis.rotate(this.angle * -1);
  this.yAxis.rotate(this.angle * -1);


  this.density = density || 1;
  if (density === 0) {
    this.density = 0;
  }

  this.restitution = (bounciness > 1 ? 0.8 : bounciness) || 0.8;

  this.staticFriction = 0.4;
  this.dynamicFriction = 0.15;

  this.angularRest = true;

  this.mass = w * h * this.density;
  if (this.mass > 0) {
    this.momentOfInertia = this.mass * (w * w + h * h) / 12;
  } else {
    this.momentOfInertia = Number.MAX_SAFE_INTEGER;
  }

  this.invMass = this.mass > 0 ? 1 / this.mass : 0;
};

Box.prototype.update = function() {
  this.display();

  this.loc.add(this.vel);
  this.angle += this.angularVel;


  var s = max(this.w, this.h);
  this.min = createVector(this.loc.x - s - 3, this.loc.y - s - 3);
  this.max = createVector(this.loc.x + s + 3, this.loc.y + s + 3);

  this.xAxis.rotate(this.angularVel * -1);
  this.yAxis.rotate(this.angularVel * -1);

  for (var i = 0; i < this.cornerDisp.length; i++) {
    this.cornerDisp[i].rotate(this.angularVel * -1);
  }

  this.verticies = [
    p5.Vector.add(this.loc, this.cornerDisp[0]),
    p5.Vector.add(this.loc, this.cornerDisp[1]),
    p5.Vector.add(this.loc, this.cornerDisp[2]),
    p5.Vector.add(this.loc, this.cornerDisp[3])
  ];

  if (debugging) {
    for (var i = 0; i < 4; i++) {
      stroke(255, 255, 255);
      ellipse(this.verticies[i].x, this.verticies[i].y, 10, 10);
    }
  }


  this.vel.add(this.acc);
  constrain(this.vel, 0, 20);
  this.acc.mult(0);

  if (this.angularRest) {
    //this.angularVel = 0;
  } else {


  }
  this.angularVel += this.torque;
  this.angularVel *= (0.999);
  //apply inertia here


  if (this.mass === 0) {
    this.vel.mult(0);
  }
};

Box.prototype.display = function() {
  noFill();
  stroke(52, 152, 219);
  strokeWeight(6);
  rectMode(CENTER);

  push();
  translate(this.loc.x, this.loc.y);
  rotate(this.angle * -1);
  rect(0, 0, this.w, this.h);
  pop();

  stroke(255, 0, 0);


  if (debugging) {
    line(this.loc.x - (this.xAxis.x * 100), this.loc.y - (this.xAxis.y * 100), this.loc.x + (this.xAxis.x * 100), this.loc.y + (this.xAxis.y * 100));
    line(this.loc.x - (this.yAxis.x * 100), this.loc.y - (this.yAxis.y * 100), this.loc.x + (this.yAxis.x * 100), this.loc.y + (this.yAxis.y * 100));
    rectMode(CORNERS);
    rect(this.min.x, this.min.y, this.max.x, this.max.y);
    rectMode(CENTER);
  }

};

Box.prototype.applyForce = function(force) {
  var f = force.copy();
  this.acc = f;
};

// Box.prototype.updateBoundingBox = function(){  
//     this.min = createVector(this.loc.x, this.loc.y);
// };




/**---------------------THE ENGINE-----------------------------**/
{


  var Engine = function() {

    this.bodies = [];

    for (var i = 0; i < 5; i++) {
      var s = random(20, 60);
      this.bodies.push(new Box(random(100, width - 100), random(height - 200), s, s, 250, 0.2, random(0, 2 * PI)));

      //this.bodies.push(new Circle(random(100,width-100),random(height-200),s,250,0.2,random(0,2*PI)));




      this.bodies[i].vel = p5.Vector.random2D();

    }

    this.bodies.push(new Box(0, 318, 80, 80, 10));
    this.bodies.push(new Box(width / 2, height / 2, 80, 80, 10));

    this.bodies.push(new Box(120, 300, 80, 80, 25, 0.5, 0));
    this.bodies.push(new Box(300, 300, 80, 80, 250, 0.2, 0.8));

    this.bodies.push(new Box(300, 600, 600, 200, 0, 0.5, 0));



    this.collisionInfo = {
      body1: undefined,
      body2: undefined,
      overlap: undefined,
      normal: undefined
    };

    this.detectCollision = [
      [this.boxVboxCollision.bind(this),
        this.boxVcircleCollision.bind(this)
      ],


      [this.circleVboxCollision.bind(this),
        this.circleVcircleCollision.bind(this)
      ]
    ];

  };


  Engine.prototype.projectShape = function(body, axis, vertexFlag) {
    // The vertex flag is for when the vertex index is needed
    var maxInd = 0;
    var minInd = 0;

    switch (body.type) {

      case 0:

        var min = Number.MAX_SAFE_INTEGER;
        var max = Number.MIN_SAFE_INTEGER;

        for (var i = 0; i < body.verticies.length; i++) {
          var comp = body.verticies[i].dot(axis);
          if (comp < min) {
            min = comp;
            minInd = i;
          }
          if (comp > max) {
            max = comp;
            maxInd = i;
          }
        }

        if (vertexFlag) {
          return {
            min: min,
            max: max,
            minInd: minInd,
            maxInd: maxInd
          };
        } else {
          return {
            min: min,
            max: max
          };
        }

        break;


      case 1:
        var projection = body.loc.dot(axis);
        var min = projection - body.rad;
        var max = projection + body.rad;

        return {
          min: min,
          max: max
        };
    }
  };

  Engine.prototype.collidingEdge = function(vertices, vertexIndex, collisionNormal) {

    //returns which edge of the body is colliding two vectors. If its a corner that is colliding, the best edge from either side of the corner is chosen. Best being most 'upright' to the normal.

    //vertexInd = vertex index of colliding vertex


    //vertices sequence in array from top left clockwise
    var v = vertices[vertexIndex];
    //adjacent vertices
    var v0 = vertices[vertexIndex !== 0 ? vertexIndex - 1 : 3];
    var v1 = vertices[vertexIndex !== 3 ? vertexIndex + 1 : 0];


    var edge1 = p5.Vector.sub(v, v0);
    var edge2 = p5.Vector.sub(v, v1);



    edge1.normalize();
    edge2.normalize();


    //if edge1 is more perpendicular to normal than edge2...

    //return edge vector, edge start vector and edge end vector(order of start and end CLOCKWISE)




    var e1 = Math.abs(edge1.dot(collisionNormal));
    var e2 = Math.abs(edge2.dot(collisionNormal));

    if (e1 < e2) {
      var edge = p5.Vector.sub(v, v0);
      edge.normalize();


      return {
        edge: edge,
        start: v0,
        end: v
      };

    } else {

      var edge = p5.Vector.sub(v1, v);
      edge.normalize();


      return {
        edge: edge,
        start: v,
        end: v1
      };
    }

  };


  Engine.prototype.getContactPoints = function(vertices1, vertices2, collidingVertex1, collidingVertex2, normal) {


    var collidingEdge1 = this.collidingEdge(vertices1, collidingVertex1, normal);

    var collidingEdge2 = this.collidingEdge(vertices2, collidingVertex2, normal);



    var edge1 = collidingEdge1.edge;
    var edge2 = collidingEdge2.edge;


    var refEdge; //reference and incident edges
    var incEdge;

    //refEdge clips incident edge

    var flipped = false;
    if (Math.abs(edge1.dot(normal)) < Math.abs(edge2.dot(normal))) {
      refEdge = collidingEdge1;
      incEdge = collidingEdge2;

    } else {
      refEdge = collidingEdge2;
      incEdge = collidingEdge1;
      flipped = true;
    }


    //contact points are along the incident edge, the incident edge needs to be 'clipped' to find them

    //the reference edge is normalized (stays in the direction of vertex winding)

    //first clipping boundary

    var bound1 = refEdge.start.dot(refEdge.edge);


    var contactpoints = this.clipIncident(refEdge.edge, bound1, incEdge.start, incEdge.end);


    if (contactpoints.length < 2) {
      println("collision error");
      return;
    }

    var bound2 = refEdge.end.dot(refEdge.edge);
    bound2 *= -1;
    var flippedNormal = p5.Vector.mult(refEdge.edge, -1);


    contactpoints = this.clipIncident(flippedNormal, bound2, contactpoints[0], contactpoints[1]);

    if (contactpoints.length < 1) {
      println("collision error");
      return;
    }


    var refEdgeNormal = createVector(normal.x, normal.y);
    if (flipped) {
      refEdgeNormal.mult(-1);
    }
    var separationNormalBound = refEdge.start.dot(refEdgeNormal);

    var outOfBoundIndex = -1;
    if (contactpoints[0].dot(refEdgeNormal) < separationNormalBound) {
      outOfBoundIndex = 0;
    }

    if (contactpoints[1].dot(refEdgeNormal) < separationNormalBound) {
      outOfBoundIndex = 1;
    }

    if (outOfBoundIndex > -1) {
      contactpoints.splice(outOfBoundIndex, 1);
    }


    if (showContactPoints) {
      for (var i = 0; i < contactpoints.length; i++) {
        fill(134, 35, 184);
        ellipse(contactpoints[i].x, contactpoints[i].y, 15, 15);
      }
    }


    return contactpoints;
  };


  Engine.prototype.clipIncident = function(refEdge, bound, incStart, incEnd) {

    //c means the component of the point along the reference edge
    var c1 = incStart.dot(refEdge);
    var c2 = incEnd.dot(refEdge);

    var clippedPoints = [];

    //if the point is to the right of the bound...

    if (c1 - bound >= 0) {
      clippedPoints.push(incStart);

    }

    if (c2 - bound >= 0) {
      clippedPoints.push(incEnd);
    }


    var d1 = c1 - bound;
    var d2 = c2 - bound;


    //if one of the points are out of the clipping bound and the other is within the bound... (if both are the same sign then both are within the bound.



    if (d1 * d2 < 0) {

      var v = p5.Vector.sub(incEnd, incStart);
      var ratio = d1 / (d1 - d2);
      var clippedPoint = p5.Vector.mult(v, ratio);
      clippedPoint.add(incStart);
      clippedPoints.push(clippedPoint);
    }

    return clippedPoints;
  };


  Engine.prototype.boxVboxCollision = function(box1, box2) {

    //checking the axes perpendicular to the edges of each box for an interection. If there is no rotation just basically checking if they overlap in the x axis or y axis 


    var x1A = this.projectShape(box1, box1.xAxis, 1);
    var x1B = this.projectShape(box2, box1.xAxis, 1);

    var x2A = this.projectShape(box1, box2.xAxis, 1);
    var x2B = this.projectShape(box2, box2.xAxis, 1);

    var y1A = this.projectShape(box1, box1.yAxis, 1);
    var y1B = this.projectShape(box2, box1.yAxis, 1);

    var y2A = this.projectShape(box1, box2.yAxis, 1);
    var y2B = this.projectShape(box2, box2.yAxis, 1);




    if (
      x1A.max < x1B.min || x1A.min > x1B.max ||
      x2A.max < x2B.min || x2A.min > x2B.max ||
      y1A.max < y1B.min || y1A.min > y1B.max ||
      y2A.max < y2B.min || y2A.min > y2B.max) {
      return false;
    }

    //difference between the "inner" max and min to get the overlap on each axis

    var x1overlap = min(x1A.max, x1B.max) - max(x1A.min, x1B.min);
    var x2overlap = min(x2A.max, x2B.max) - max(x2A.min, x2B.min);
    var y1overlap = min(y1A.max, y1B.max) - max(y1A.min, y1B.min);
    var y2overlap = min(y2A.max, y2B.max) - max(y2A.min, y2B.min);


    var minOverlap = min(min(x1overlap, x2overlap), min(y1overlap, y2overlap));

    this.collisionInfo.overlap = minOverlap;

    //making normal other direction depending on whether box1 is left/right or higher/lower compared to box2

    var bodyOrder; //body A before or after body B?
    var normal;





    switch (minOverlap) {

      case x1overlap:
        bodyOrder = 1;
        normal = box1.xAxis;
        if (x1A.max < x1B.max) {
          bodyOrder = -1;
        }
        normal.mult(bodyOrder);


        this.collisionInfo.normal = normal;
        var collidingVertex1 = bodyOrder === -1 ? x1A.maxInd : x1A.minInd;
        var collidingVertex2 = bodyOrder === -1 ? x1B.minInd : x1B.maxInd;


        this.collisionInfo.contactPoints = this.getContactPoints(box1.verticies, box2.verticies, collidingVertex1, collidingVertex2, normal);

        break;


      case x2overlap:
        bodyOrder = 1;
        normal = box2.xAxis;
        if (x2A.max < x2B.max) {
          bodyOrder = -1;
        }
        normal.mult(bodyOrder);

        this.collisionInfo.normal = normal;


        var collidingVertex1 = bodyOrder === -1 ? x2A.maxInd : x2A.minInd;
        var collidingVertex2 = bodyOrder === -1 ? x2B.minInd : x2B.maxInd;

        this.collisionInfo.contactPoints = this.getContactPoints(box1.verticies, box2.verticies, collidingVertex1, collidingVertex2, normal);


        break;



      case y1overlap:
        bodyOrder = 1;
        normal = box1.yAxis;
        if (y1A.max < y1B.max) {
          bodyOrder = -1;
        }
        normal.mult(bodyOrder);

        this.collisionInfo.normal = normal;

        var collidingVertex1 = bodyOrder === -1 ? y1A.maxInd : y1A.minInd;
        var collidingVertex2 = bodyOrder === -1 ? y1B.minInd : y1B.maxInd;


        this.collisionInfo.contactPoints = this.getContactPoints(box1.verticies, box2.verticies, collidingVertex1, collidingVertex2, normal);



        break;



      case y2overlap:
        bodyOrder = 1;
        normal = box2.yAxis;
        if (y2A.max < y2B.max) {
          bodyOrder = -1;
        }
        normal.mult(bodyOrder);

        this.collisionInfo.normal = normal;


        var collidingVertex1 = bodyOrder === -1 ? y2A.maxInd : y2A.minInd;
        var collidingVertex2 = bodyOrder === -1 ? y2B.minInd : y2B.maxInd;


        this.collisionInfo.contactPoints = this.getContactPoints(box1.verticies, box2.verticies, collidingVertex1, collidingVertex2, normal);




        break;


      default:
        println("Error: Something went wrong");
    }

    this.collisionInfo.body1 = box1;
    this.collisionInfo.body2 = box2;


    return true;
  };

  Engine.prototype.boxVcircleCollision = function(box, circle) {

    //checking six axes for intersection between box and circle - the x and y axes, and 4 axes parallel to the line between each vertex and the center of the circle

    for (var i = 0; i < box.verticies.length; i++) {
      var axis = p5.Vector.sub(circle.loc, box.verticies[i]);
      axis.normalize();

      var boxProj = this.projectShape(box, axis);
      var cirProj = this.projectShape(circle, axis);

      if (boxProj.max < cirProj.min || boxProj.min > cirProj.max) {
        return false;
      }
    }

    var boxProjX = this.projectShape(box, box.xAxis);
    var cirProjX = this.projectShape(circle, box.xAxis);
    var boxProjY = this.projectShape(box, box.yAxis);
    var cirProjY = this.projectShape(circle, box.yAxis);


    if (boxProjX.max < cirProjX.min || boxProjX.min > cirProjX.max ||
      boxProjY.max < cirProjY.min || boxProjY.min > cirProjY.max) {
      return false;
    }


    this.collisionInfo.body1 = box;
    this.collisionInfo.body2 = circle;
    //this.collisionInfo.normal; 

    this.collisionInfo.overlap = 3;

    //println("detected");

    return true;
  };

  Engine.prototype.circleVboxCollision = function(circle, box) {
    return this.boxVcircleCollision(box, circle);
  };

  Engine.prototype.circleVcircleCollision = function(circle1, circle2) {

    var sqDist = Math.pow(circle2.loc.x - circle1.loc.x, 2) + Math.pow(circle2.loc.y - circle1.loc.y, 2);

    if (Math.pow(circle1.rad + circle2.rad, 2) < sqDist) {
      return false;
    }

    this.collisionInfo.body1 = circle1;
    this.collisionInfo.body2 = circle2;
    this.collisionInfo.normal = p5.Vector.sub(circle1.loc, circle2.loc);

    this.collisionInfo.normal.normalize();


    var contactRadius = p5.Vector.mult(this.collisionInfo.normal, circle2.rad);

    this.collisionInfo.contactPoints = [p5.Vector.add(circle2.loc, contactRadius)];

    this.collisionInfo.overlap = circle1.rad + circle2.rad - Math.sqrt(sqDist);


    return true;
  };

  Engine.prototype.aabbCollision = function(body1, body2) {
    if (body1.max.x < body2.min.x || body1.min.x > body2.max.x || body1.max.y < body2.min.y || body1.min.y > body2.max.y) {
      return false;
    }

    return true;
  };

  Engine.prototype.handleCollision = function(b1, b2) {
    /**This is about one method of collision handling. Collision has been detected. To handle it first find the normal vector (just like the normal line in light reflection). Normalize it to get it in unit vector form. 
    
    Next find the component of both entity vectors along the normal unit vector (this is a scalar value representing how much the velocity goes in that direction. The component of a vector along the x-axis is the x value of the vector. (Use dot product)
    
    Now it is known "how much"(a magnitude) the bodies are moving in the normal direction. Now ignore everything and consider this a simple 1D elastic collision (momentum and kinetic energy is conserved. Like two bouncy cars going towards each other. Their velocities are now known (the component velocities). To calculate their velocities after collision use the formula derived from conservation of momentum and kinetic energy formulas:  

Vafter = v1(m1-m2)+2m2*v2 / (m1+m2)

This gives the new normal component with respect to the mass of the bodies. Now that the new normal component is known all that is left is the tangential component. In a simple ball collision with the ground the tangential component is the x component of the velocity of the ball and the normal is the y component. The x component doesn't change, the ball keeps moving to the side but y does cause it bounces. To find the tangential component just find the component of the velocity on to the tangent unit vector which is (-y,x) of the normal unit vector. Add the two then done.
        
    var normalVector = p5.Vector.sub(circle2.loc, circle1.loc);
    normalVector.normalize();
    var tangentVector = createVector(normalVector.y*-1,normalVector.x);

    
    var v1 = circle1.vel.dot(normalVector); 
    //no need to divide by magnitude of normal vector because the magnitude is one
    var v2 = circle2.vel.dot(normalVector);
    
    var newNormalV1 = (v1*(circle1.mass-circle2.mass)+2*circle2.mass*v2)/(circle1.mass+circle2.mass);
    
    var newNormalV2 = (v2*(circle2.mass-circle1.mass)+2*circle1.mass*v1)/(circle1.mass+circle2.mass);
    
    var nv1 = p5.Vector.mult(normalVector, newNormalV1);
    var nv2 = p5.Vector.mult(normalVector, newNormalV2);
    
    var tv1 = p5.Vector.mult(tangentVector,circle1.vel.dot(tangentVector));
    var tv2 = p5.Vector.mult(tangentVector,circle2.vel.dot(tangentVector));
    

    
    
    circle1.vel = p5.Vector.add(nv1,tv1);
    circle2.vel = p5.Vector.add(nv2,tv2);
    
    circle1.vel.mult(circle1.restitution);
    circle2.vel.mult(circle2.restitution);
    
    circle1.vel = nv1;
    circle2.vel = nv2;
    */

    var body1 = this.collisionInfo.body1;
    var body2 = this.collisionInfo.body2;




    //Normal of collision - basically a vector representing a line perpendicular to the point of contact. normalized?

    var normal = this.collisionInfo.normal;




    // All the variables needed to account for angular motion including radius, perpendicular radius and moment of inertia

    var r1 = [];
    var r2 = [];
    var perpR1 = [];
    var perpR2 = [];
    var a = [];
    var b = [];


    for (var i = 0; i < this.collisionInfo.contactPoints.length; i++) {

      r1.push(p5.Vector.sub(this.collisionInfo.contactPoints[i], b1.loc));
      r2.push(p5.Vector.sub(this.collisionInfo.contactPoints[i], b2.loc));
      perpR1.push(createVector(r1[i].y, r1[i].x * -1));
      perpR2.push(createVector(r2[i].y, r2[i].x * -1));
      a.push(Math.pow(perpR1[i].dot(normal), 2) / body1.momentOfInertia);
      b.push(Math.pow(perpR2[i].dot(normal), 2) / body2.momentOfInertia);
    }




    // Relative velocity of body1 with respect to body2


    var relVel = p5.Vector.sub(body1.vel, body2.vel);


    // Linear velocities due to rotation
    var v1 = p5.Vector.mult(perpR1[0], body1.angularVel);
    var v2 = p5.Vector.mult(perpR2[0], body2.angularVel);


    relVel.add(p5.Vector.sub(v1, v2));
    //println("working");


    // The component of the relative velocity along the normal
    var relVelAlongNorm = relVel.dot(normal);

    // If the relative velocity along normal is pointing in the direction of body1 (it is positive) that means body1 is moving away from body 2 so no force is needed

    if (relVelAlongNorm > 0) {
      return;
    }


    /*Because there is no friction, during collision  only normal component of the relative velocity changes. It becomes flipped (times -1) and depends on type of material. Tangential stays the same.
    
    Vf = normal rel velocity after && Vaf = normal vel of body1 after && Vbf = normal vel of body2 after
    
    Vf = -e*Vi;  Where e represents restitution or bounciness
    
    (Vaf-Vbf).normal = -e(Vai-Vbi).normal
    
    Vaf = Vai+(force/mass)*normal
    
    solving for force gives: f=-(1+e)*Vi.n / n.n(1/massA+1/massB)
    
    n.n can be avoided if n is normalized
    
    (changed some parts of it to account for angular momentum)
    */



    // frameRate(1);


    //Following handles angular momentum. Needs more work


    var e = min(body1.restitution, body2.restitution);

    var impulseN = -1 * (1 + e) * relVelAlongNorm;
    impulseN /= ((body1.invMass + body2.invMass) + a[0] + b[0]);




    var omegaChange1;
    var omegaChange2;

    if (b1.mass > 0) {
      b1.vel.add(p5.Vector.mult(normal, impulseN * body1.invMass));

      if (perpR1.length <= 1) {

        omegaChange1 = perpR1[0].dot(p5.Vector.mult(normal, impulseN)) / body1.momentOfInertia;

        b1.angularVel += omegaChange1;

      } else {
        omegaChange1 = perpR1[0].dot(p5.Vector.mult(normal, impulseN / 2)) / body1.momentOfInertia;

        omegaChange2 = perpR1[1].dot(p5.Vector.mult(normal, impulseN / 2)) / body1.momentOfInertia;


        b1.angularVel += omegaChange1;
        b1.angularVel += omegaChange2;

        b1.angularVel *= 0.8;
      }

    }




    var omegaChange1;
    var omegaChange2;

    if (b2.mass > 0) {
      b2.vel.sub(p5.Vector.mult(normal, impulseN * body2.invMass));

      if (perpR2.length <= 1) {

        omegaChange1 = perpR2[0].dot(p5.Vector.mult(normal, impulseN)) / body2.momentOfInertia;

        b2.angularVel -= omegaChange1;


      } else {
        omegaChange1 = perpR2[0].dot(p5.Vector.mult(normal, impulseN / 2)) / body2.momentOfInertia;
        omegaChange2 = perpR2[1].dot(p5.Vector.mult(normal, impulseN / 2)) / body2.momentOfInertia;


        b2.angularVel -= omegaChange1;
        b2.angularVel -= omegaChange2;

        b2.angularVel *= 0.8;
      }

    }


    /* Now to acount for friction the tangential component of the velocity needs to be changed (like when a ball bounces on a rough ground not only does the y-vel of the ball change but also the xvel
    
    */


    //tangent needs to point in the general direction of the relVel but also be perpendicular to the normal



    var tangent = p5.Vector.sub(relVel, p5.Vector.mult(normal, relVelAlongNorm));
    tangent.normalize();




    var relVelAlongTangent = relVel.dot(tangent);

    var impulseT = -1 * relVelAlongTangent;
    impulseT /= ((body1.invMass + body2.invMass + a[0] + b[0]));
    //TODO: add a+b



    //restricting the friction force using the average coefficient of friction of the two bodies

    var fr = Math.sqrt((Math.pow(body1.staticFriction, 2) + Math.pow(body2.staticFriction, 2)));


    if (Math.abs(impulseT) > impulseN * fr) {
      var fr2 = Math.sqrt((Math.pow(body1.dynamicFriction, 2) + Math.pow(body2.dynamicFriction, 2)));


      impulseT = impulseN * fr2 * -1;
    }


    if (b1.mass > 0) {
      b1.vel.add(p5.Vector.mult(tangent, impulseT * b1.invMass));

      var omegaChange = perpR1[0].dot(p5.Vector.mult(tangent, impulseT)) / body1.momentOfInertia;

      b1.angularVel += omegaChange;
    }

    if (b2.mass > 0) {
      b2.vel.sub(p5.Vector.mult(tangent, impulseT * b2.invMass));

      var omegaChange = perpR2[0].dot(p5.Vector.mult(tangent, impulseT)) / body2.momentOfInertia;

      b2.angularVel -= omegaChange;

    }

  };


  Engine.prototype.offsetPos = function(b1, b2, level) {

    var slop = 0.05;

    //no correction or slight correction depending on magnitude of the overlap 
    var penetration = 0;
    if (this.collisionInfo.overlap > slop) {
      penetration = this.collisionInfo.overlap;
    }

    var offsetMag = (penetration / (b1.invMass + b2.invMass)) * level;

    var offset = p5.Vector.mult(this.collisionInfo.normal, offsetMag);

    b1.loc.add(p5.Vector.mult(offset, b1.invMass));
    b2.loc.sub(p5.Vector.mult(offset, b2.invMass));
  };


  Engine.prototype.run = function() {
    var collision = false;
    for (var i = 0; i < this.bodies.length; i++) {

      if (gravity) {
        this.bodies[i].applyForce(createVector(0, 0.05));
      }

      if (i === 0) {
        //this.bodies[i].loc = createVector(mouseX, mouseY);
        //this.bodies[i].applyForce(p5.Vector.mult(p5.Vector.sub(createVector(mouseX, mouseY), this.bodies[i].loc),0.0001));

      }
      this.bodies[i].update();

    }



    //Run offset position several times. The more times the better the stacking but more jittering and errors

    for (var i = 0; i < this.bodies.length; i++) {
      for (var j = i + 1; j < this.bodies.length; j++) {
        if (!this.aabbCollision(this.bodies[i], this.bodies[j])) {
          continue;
        }


        collision = this.detectCollision[this.bodies[i].type][this.bodies[j].type](this.bodies[i], this.bodies[j]);


        if (collision) {
          this.handleCollision(this.bodies[i], this.bodies[j]);
          for (var n = 0; n < 2; n++) {
            var s = 1 / (2);
            this.offsetPos(this.bodies[i], this.bodies[j], s);
          }
          collision = false;
        }

      }
    }


  };



}


/**----------------------------------------------------------**/

var mainEngine;
var lastLoop;

function setup() {
  createCanvas(600, 600);
  mainEngine = new Engine();
  lastLoop = millis();
}



function draw() {
  background(0);
  mainEngine.run();

  //noLoop();


  var thisLoop = millis();
  var fps = 1000 / (thisLoop - lastLoop);
  if (debugging && frameCount % 30 === 0) {
    print("FPS: " + fps);
  }
  lastLoop = thisLoop;

};

var mouseClicked = function() {
  mainEngine.bodies.push(new Box(mouseX, mouseY, 30, 30, 200, 0.5));
};