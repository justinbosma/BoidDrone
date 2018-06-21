import oscP5.*;
import netP5.*;

Flock flock;
OscP5 oscP5 = new OscP5(this, 12000);
NetAddress superCollider = new NetAddress("127.0.0.1", 57120);

void setup() {
  //frameRate(1);
  size(1500, 900);
  flock = new Flock();
  
  for (int i = 0; i < 150; i++) {
    flock.addBoid(new Boid(width/2, height/2, i));
  }
}

void draw() {
  background(50);
  flock.run();
}

void mousePressed() {
  int boidId = flock.flock.size();
  flock.addBoid(new Boid(mouseX, mouseY, boidId));
}


class Boid {
  PVector position;
  PVector acceleration;
  PVector velocity;
  int size = 5;
  float speedLimit = 2;
  float minDistanceLimit = 25.0f;
  int neighborhood = 40;
  float maxForce = 0.03;
  int id;

  Boid(float x, float y, int id) {
    this.acceleration = new PVector(0, 0);
    this.velocity = PVector.random2D();
    this.position = new PVector(x, y);
    this.id = id;
    OscMessage message = new OscMessage("/makeSin");
    message.add(id);
    message.add(position.x + 200);
    message.add(position.y/height);
    message.add(position.y/height);
    oscP5.send(message, superCollider);
  }

  void run(Flock boids) {
    Flock neighbors = getNeighborhood(boids);
    flock(neighbors);
    update();
    getBoundry();
    drawBoid();
  }

  PVector cohesion(Flock boids) {
    PVector neighborSum = new PVector(0, 0);
    for (int i = 0; i < boids.flock.size(); i++) {
      Boid b = boids.flock.get(i);
      neighborSum.add(b.position);
    }
    if (boids.flock.size() > 0) {
      neighborSum.div(boids.flock.size());
      PVector desired = neighborSum.sub(this.position);
      desired.normalize();
      desired.mult(this.speedLimit);
      PVector steer = desired.sub(this.velocity);
      steer.limit(maxForce);
      return steer;
    } else {
      return new PVector(0, 0);
    }
  }

  //takes in flock within neighborhood
  PVector separation(Flock boids) {
    float numTooClose = 0;
    PVector steer = new PVector(0, 0, 0);
    for (int i = 0; i < boids.flock.size(); i++) {
      Boid b = boids.flock.get(i);
      float distance = this.position.dist(b.position);
      if (distance < this.minDistanceLimit && distance > 0) {
        PVector difference = new PVector(0, 0);
        difference = PVector.sub(position, b.position);
        difference.normalize();
        difference.div(distance);
        numTooClose++;
        steer.add(difference);
      }
    }
    if (numTooClose > 0) {
      steer.div((float)numTooClose);
    }
    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(this.speedLimit);
      steer.sub(this.velocity);
      steer.limit(this.maxForce);
    }
    return steer;
  }

  //takes in neighborhood of boids
  PVector alignment(Flock boids) {
    PVector velocitySum = new PVector(0, 0);
    for (int i = 0; i < boids.flock.size(); i++) {
      Boid b = boids.flock.get(i);
      velocitySum.add(b.velocity);
    }
    if (boids.flock.size() > 0) {
      velocitySum.div((float)boids.flock.size());
      velocitySum.normalize();
      velocitySum.mult(this.speedLimit);
      PVector steer = PVector.sub(velocitySum, this.velocity);
      steer.limit(maxForce);
      return steer;
    } else {
      return new PVector(0, 0);
    }
  }

  void update() {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.speedLimit);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
    OscMessage message = new OscMessage("/control");
    message.add(id);//ID
    message.add(position.x + 200);//Freq
    message.add(position.y/height);//amp
    message.add(position.y/height);//rez
    oscP5.send(message, superCollider);
  }

  void flock(Flock boids) {
    PVector sep = separation(boids);
    PVector ali = alignment(boids);
    PVector coh = cohesion(boids);
    sep.mult(1.5);
    ali.mult(1.0);
    coh.mult(1.0);
    this.acceleration.add(sep);
    this.acceleration.add(coh);
    this.acceleration.add(ali);
  }

  void getBoundry() {
    if (this.position.x < -size) {
      this.position.x = width + size;
    }
    if (this.position.x > width + size) {
      this.position.x = -size;
    }
    if (this.position.y < -size) {
      this.position.y = height + size;
    }
    if (this.position.y > height + size) {
      this.position.y = -size;
    }
  }

  void drawBoid() {
    fill(0);
    stroke(255);
    ellipse(this.position.x, this.position.y, this.size, this.size);
  }

  Flock getNeighborhood(Flock boids) {
    Flock neighbors = new Flock();
    for (int i = 0; i < boids.flock.size(); i++) {
      Boid b = boids.flock.get(i);
      float dist = this.position.dist(b.position);
      if (dist < neighborhood) {
        neighbors.flock.add(b);
      }
    }
    return neighbors;
  }
}

class Flock {
  ArrayList<Boid> flock;

  Flock() {
    this.flock = new ArrayList<Boid>();
  }

  void addBoid(Boid boid) {
    this.flock.add(boid);
  }

  void run() {
    for (int i = 0; i < this.flock.size(); i++) {
      this.flock.get(i).run(this);
    }
  }
}