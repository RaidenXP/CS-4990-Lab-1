/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }

   void update(float dt)
   { 
     if (target != null)
     {  
        // TODO: Implement seek here
        float rotational_speed = kinematic.max_rotational_speed;
        float linear_speed = kinematic.max_speed;
        
        float arrivalAngle = PI/2;
        float arrivalDistance = 300;
        
        PVector currentPos = kinematic.getPosition(); //<>//
        float distLeft = PVector.sub(currentPos, target).mag();
        float target_angle = atan2(target.y - currentPos.y, target.x - currentPos.x);
        float dr = normalize_angle_left_right(target_angle - kinematic.getHeading());
        
        PVector currentDir = PVector.sub(target, currentPos).normalize();
        // tried to use this to help with moving through waypoints
        PVector futureDir = currentDir.copy();
        
        //println(currentDir);
        
        if (waypoints.size() > 0){
          futureDir = PVector.sub(waypoints.get(0), target).normalize();
        }
        
        if(dr < 0){
          rotational_speed = -1.0 * kinematic.max_rotational_speed; 
        }
        
        if(abs(dr) < arrivalAngle){
          rotational_speed = ((dr/arrivalAngle * kinematic.max_rotational_speed) - kinematic.getRotationalVelocity()) * 15;
        }
        
        //printing out dot product to see if the directions are going the same
        //println(PVector.dot(currentDir, futureDir));
        
        if(distLeft < arrivalDistance && waypoints.isEmpty()){
          linear_speed = (distLeft/arrivalDistance * kinematic.max_speed) - kinematic.getSpeed();
        }
        else if(distLeft < 100 && PVector.dot(currentDir, futureDir) < 0.6 && PVector.dot(currentDir, futureDir) > -1.0){
          linear_speed = ((distLeft/arrivalDistance * kinematic.max_speed) - kinematic.getSpeed());
        }
        
        //printing out speeds and distance left
        //println(kinematic.getRotationalVelocity() + ", " +  kinematic.getSpeed() + ", " + distLeft + ", " + dr);
        
        kinematic.increaseSpeed(linear_speed * dt, rotational_speed * dt);
        
        if(distLeft < 10 && !(waypoints.isEmpty())){
          follow(waypoints);
        }
        
     }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   }
   
   void draw()
   {
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   void seek(PVector target)
   {
      //helps with pathfinding
      //this.target = target; //<>//
      waypoints = nm.findPath(kinematic.getPosition(), target);
      follow(waypoints);
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      while(waypoints.size() > 1 && waypoints.get(0).x == waypoints.get(1).x && waypoints.get(0).y == waypoints.get(1).y){
       waypoints.remove(0); 
      }
      
      this.target = waypoints.get(0);
      waypoints.remove(0);
   }
}
