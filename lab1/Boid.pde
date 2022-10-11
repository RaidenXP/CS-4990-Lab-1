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
        
        if(dr < 0){
          rotational_speed = -1.0 * kinematic.max_rotational_speed; 
        }
        
        if(abs(dr) < arrivalAngle){
          //old method
          //rotational_speed = (dr/arrivalAngle * kinematic.max_rotational_speed) - kinematic.getRotationalVelocity();
          
          //method 1
          rotational_speed = ((dr/arrivalAngle * kinematic.max_rotational_speed) - kinematic.getRotationalVelocity()) * 20;
          
          //method 2
          //rotational_speed = (dr/arrivalAngle * kinematic.max_rotational_speed) - kinematic.getRotationalVelocity() * 5;
        }
        
        if(distLeft < arrivalDistance && waypoints.isEmpty()){
          linear_speed = (distLeft/arrivalDistance * kinematic.max_speed) - kinematic.getSpeed();
        }
        //else if (distLeft < 100){
        //  linear_speed = (distLeft/arrivalDistance * kinematic.max_speed) - kinematic.getSpeed();
        //}
        
        println(kinematic.getRotationalVelocity() + ", " +  kinematic.getSpeed() + ", " + distLeft + ", " + dr);
        
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
      this.target = target;
      
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      this.target = waypoints.get(0);
      waypoints.remove(0);
   }
}
