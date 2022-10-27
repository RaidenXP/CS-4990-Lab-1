// Useful to sort lists by a custom key
import java.util.Comparator;


/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
   float r;
   float g;
   float b;
}



class NavMesh
{ 
   ArrayList<Node> navMesh;
   ArrayList<PVector> reflexAng;
   //ArrayList<ArrayList<Wall>> polygons;
   
   NavMesh()
   {
       navMesh = new ArrayList<Node>();
       reflexAng = new ArrayList<PVector>();
       //polygons = new ArrayList<ArrayList<Wall>>();
   }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       
       // set up/reset for each map
       reflexAng.clear();
       navMesh.clear();
       
       // using this for now to store all polygons
       //polygons.clear();
       
       // find reflex angles to draw
       // here for now but might get rid later on
       for(int i = 0; i < map.walls.size(); ++i){
          int next = i + 1;
          
          if(next >= map.walls.size()){
            next = 0;
          }
          
          float value = PVector.dot(map.walls.get(i).normal, map.walls.get(next).direction);
          
          if(value > 0){
            PVector angle = new PVector(map.walls.get(i).end.x, map.walls.get(i).end.y);
            reflexAng.add(angle);
          }
       }
       
       ArrayList<Wall> firstPoly = new ArrayList<Wall>(map.walls);
       
       // split the initial polygon into two
       genGraph(firstPoly);
   }
    //<>//
   void genGraph(ArrayList<Wall> polygon){
     // marker to check if the wall before has a relfex angle coming
     int wallIndex = -1; //<>//
     
     // two polygons
     ArrayList<Wall> freshPolygon = new ArrayList<Wall>();
     ArrayList<Wall> otherPolygon = new ArrayList<Wall>();
     
     //find first reflex angle for current polygon and divide accordingly
     for(int i = 0; i < polygon.size(); ++i){
          int next = i + 1;
          
          if(next >= polygon.size()){
            next = 0;
          }
          
          float value = PVector.dot(polygon.get(i).normal, polygon.get(next).direction);
          
          if(value > 0){
              // just a marker
              // i marks the wall before the reflex angle
              wallIndex = i;
              
              //shortest edge we are looking for
              int chosen = 0;
              
              //initial min edge length
              float min_size = Float.POSITIVE_INFINITY;
              
              // temp set
              Wall temp = new Wall(polygon.get(i).end, polygon.get(chosen).start);
              
              // loop that does the main process of dividing the parent polygon
              for(int j = 0; j < polygon.size(); ++ j){
                 PVector direction = PVector.sub(polygon.get(j).start, polygon.get(i).end);  
                 PVector shortStart = PVector.add(polygon.get(i).end, PVector.mult(direction, 0.01));
                 PVector shortEnd = PVector.add(polygon.get(j).start, PVector.mult(direction, -0.01));
                 
                 // wall that will be made if placeable and shortest
                 temp = new Wall(polygon.get(i).end, polygon.get(j).start);
                 if(placeable(map, polygon, shortStart, shortEnd, temp) && temp.len < min_size){
                     min_size = temp.len;
                     chosen = j;
                 }
              }
              
              // reset temp and reverse direcction of temp above
              temp = new Wall(polygon.get(i).end, polygon.get(chosen).start);
              Wall tempPrime = new Wall(polygon.get(chosen).start, polygon.get(i).end);
             
              // chosen will be the wall that the reflex angle is connected to
              // k will be the point where we should "remove" consecutive edges until we have our otherPolygon
              int k = 0;
             
              if(chosen == 0){
                 k = polygon.size() - 1; 
              }
              else{
                 k = chosen - 1; 
              }
             
              while(i != chosen){
                 freshPolygon.add(polygon.get(chosen));
                  ++chosen;
                 
                  if(chosen >= polygon.size()){
                     chosen = 0; 
                  }
              }
             
              freshPolygon.add(polygon.get(chosen));
             
              ++chosen;
              if(chosen >= polygon.size()){
                chosen = 0; 
              }
             
              while(chosen != k){
                  otherPolygon.add(polygon.get(chosen));
                  ++chosen;
                 
                  if(chosen >= polygon.size()){
                     chosen = 0; 
                  }
              }
             
              otherPolygon.add(polygon.get(chosen));
             
              freshPolygon.add(temp);
              otherPolygon.add(tempPrime);
              
              break;
          }
     }
     
     if(wallIndex != -1){
         genGraph(freshPolygon);
         genGraph(otherPolygon);
     }
     else{
         Node n = new Node();
         n.id = navMesh.size();
         n.polygon = new ArrayList<Wall>(polygon);
         n.center = new PVector(0,0);
         
         for(int z = 0; z < polygon.size(); ++z){
             n.center.x += polygon.get(z).end.x;
             n.center.y += polygon.get(z).end.x;
         }
         
         n.center.x /= polygon.size();
         n.center.y /= polygon.size();
         
         n.r = random(0, 255);
         n.g = random(0, 255);
         n.b = random(0, 255);
         
         navMesh.add(n);
         
         //polygons.add(polygon);
         return; 
     }
     
   }
   
   
   boolean placeable(Map map, ArrayList<Wall> polygon, PVector from, PVector to, Wall tempW){
     boolean flag = false;
     boolean otherFlag = true;
     
     for(Wall w : polygon){
         flag = !(w.crosses(from,to)); 
     }
     
     for(Wall w : polygon){
         if(w.start.x == tempW.start.x && w.start.y == tempW.start.y && w.end.x == tempW.end.x && w.end.y == tempW.end.y
           || w.end.x == tempW.start.x && w.end.y == tempW.start.y && w.start.x == tempW.end.x && w.start.y == tempW.end.y)
         {
           otherFlag = false;
           break;
         }
     }
     
     return flag && map.isReachable(to) && !map.collides(from, to) && otherFlag;
   }
   
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      ArrayList<PVector> result = null;
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   } //<>//
   
   void draw()
   {
      /// use this to draw the nav mesh graph
      for(int i = 0; i < reflexAng.size(); ++i){ //<>//
          stroke(23, 212, 233);
          circle(reflexAng.get(i).x, reflexAng.get(i).y, 10);
      }
      
      //float r = 23;
      //float g = 212;
      //float b = 233;
      
      for(Node n : navMesh){
         stroke(n.r, n.g, n.b);
         for(Wall w : n.polygon){
           line(w.start.x, w.start.y,  w.end.x, w.end.y); 
         }
      }
      
      //float r = 23;
      //float g = 212;
      //float b = 233;
      
      //for(ArrayList<Wall> p : polygons){
      //    stroke(r,g,b);
          
      //    for(Wall w : p){
      //      line(w.start.x, w.start.y,  w.end.x, w.end.y); 
      //    }
      //}
   }
}
