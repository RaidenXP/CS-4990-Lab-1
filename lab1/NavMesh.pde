// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.HashSet;

class NodeComparator implements Comparator<Node>{
    int compare(Node n1, Node n2){
        float totalN1 = n1.cost + n1.heuristic;
        float totalN2 = n2.cost + n2.heuristic;
        
        if(totalN1 < totalN2){
            return -1; 
        }
        else if(totalN1 > totalN2){
            return 1; 
        }
        
        return 0;
    }
}

/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   // Current Node info
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
   
   // info for the frontier
   Node previous;
   float cost;
   float heuristic;
   PVector prevPoint;
   
   // just for coloring
   float r;
   float g;
   float b;
}


class NavMesh
{ 
   ArrayList<Node> navMesh;
   
   NavMesh()
   {
       navMesh = new ArrayList<Node>();
   }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       
       // set up/reset for each map
       navMesh.clear();
       
       ArrayList<Wall> firstPoly = new ArrayList<Wall>(map.walls);
       
       // split the initial polygon into two
       genGraph(firstPoly);
       
       // finds neighbors, but very inefficient
       for(int i = 0; i < navMesh.size(); ++i){
           for(int j = 0; j < navMesh.size(); ++j){
               if(i == j){
                 continue; 
               }
               else{
                 boolean exit = false;
                 
                 for(int k = 0; k < navMesh.get(i).polygon.size(); ++k){
                   for(int l = 0; l < navMesh.get(j).polygon.size(); ++l){
                       if(navMesh.get(i).polygon.get(k).start.x == navMesh.get(j).polygon.get(l).end.x &&
                         navMesh.get(i).polygon.get(k).start.y == navMesh.get(j).polygon.get(l).end.y &&
                         navMesh.get(i).polygon.get(k).end.x == navMesh.get(j).polygon.get(l).start.x &&
                         navMesh.get(i).polygon.get(k).end.y == navMesh.get(j).polygon.get(l).start.y &&
                         !navMesh.get(i).neighbors.contains(navMesh.get(j)) && !navMesh.get(j).neighbors.contains(navMesh.get(i))){
                           
                           navMesh.get(i).connections.add(navMesh.get(i).polygon.get(k));
                           navMesh.get(j).connections.add(navMesh.get(j).polygon.get(l));
                           navMesh.get(i).neighbors.add(navMesh.get(j));
                           navMesh.get(j).neighbors.add(navMesh.get(i));
                           exit = true;
                       }
                       
                       if(exit){
                          break; 
                       }
                   }
                   
                   if(exit){
                     break; 
                   }
                 }
               }
           }
       }
   }
   
   void genGraph(ArrayList<Wall> polygon){
     // marker to check if the wall before has a relfex angle coming
     int wallIndex = -1;
     
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
              
              // reset temp and set reverse direcction of temp above
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
         // used to create the child of the tree
         Node n = new Node();
         n.id = navMesh.size();
         n.polygon = new ArrayList<Wall>(polygon);
         n.neighbors = new ArrayList<Node>();
         n.connections = new ArrayList<Wall>();
         n.center = new PVector(0,0);
         
         for(int z = 0; z < polygon.size(); ++z){
             n.center.x += polygon.get(z).end.x;
             n.center.y += polygon.get(z).end.y;
         }
         
         n.center.x /= polygon.size();
         n.center.y /= polygon.size();
         
         n.r = random(0, 255);
         n.g = random(0, 255);
         n.b = random(0, 255);
         
         navMesh.add(n);
         return; 
     }
     
   }
   
   
   boolean placeable(Map map, ArrayList<Wall> polygon, PVector from, PVector to, Wall tempW){
     boolean flag = false;
     boolean otherFlag = true;
     
     // check if the new wall croses
     for(Wall w : polygon){
         flag = !(w.crosses(from,to)); 
     }
     
     // check if the wall is already there in the polygon
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
      ArrayList<PVector> result = new ArrayList<PVector>(); //<>//
      
      PriorityQueue<Node> frontier = new PriorityQueue<Node>(new NodeComparator());
      HashSet<Integer> visited = new HashSet<Integer>();
      
      int startIdx = 0;
      
      int destIdx = 0;
      int destID = 0;
      
      for(int i = 0; i < navMesh.size(); ++i){
          if(isPointInPolygon(start, navMesh.get(i).polygon)){
              startIdx = i;
          }
          
          if(isPointInPolygon(destination, navMesh.get(i).polygon)){
              destIdx = i;
              destID = navMesh.get(i).id;
          }
          
          // reset the values or set
          navMesh.get(i).cost = 0;
          navMesh.get(i).heuristic = 0;
          navMesh.get(i).previous = null;
          navMesh.get(i).prevPoint = null;
      }
      
      
      frontier.add(navMesh.get(startIdx));
      
      if(startIdx == destIdx){
         result.add(start);
         result.add(destination);
      }
      else{
         Node visiting;
         while(frontier.size() != 0){
            //get the front of queue
            visiting = frontier.poll();
            
            // if our visiting item is our destination
            if(visiting.id == destID){
                Node temp = visiting;
                //resverse path
                while(temp.prevPoint != null){
                    result.add(0, temp.prevPoint);  
                    temp = temp.previous;
                }
                
                result.add(destination);
                return result;
            }
            
            // assign visiting node to visited
            visited.add(visiting.id);
            
            // grab all of the neighbors
            for(int i = 0; i < visiting.neighbors.size(); ++i){
                // cost = cost so far + magnitude(CenterOfNeighbor - CurrentCenter)
                float cost = visiting.cost + PVector.sub(visiting.neighbors.get(i).center, visiting.center).mag();
                
                // heuristic = magnitude(destinationCenter - CenterOfNeighbor)
                float heuristic = PVector.sub(navMesh.get(destIdx).center, visiting.neighbors.get(i).center).mag();
                
                // if our neighbor has already been visited and has a lower cost then the cost just calculated we can just continue
                // we could add it if we wanted to but there is no need
                if(visited.contains(visiting.neighbors.get(i).id) && visiting.neighbors.get(i).cost <= cost){
                    continue;
                }
                
                // assign cost, heuristic, previous node, and previous Mid point to add to frontier
                visiting.neighbors.get(i).cost = cost;
                visiting.neighbors.get(i).heuristic = heuristic;
                visiting.neighbors.get(i).previous = visiting;
                
                boolean exit = false;
                PVector previousMid = new PVector();
                
                // look for the shared edge's midpoint
                for(int j = 0; j < visiting.connections.size(); ++j){
                  for(int k = 0; k < visiting.neighbors.get(i).connections.size(); ++k){
                      if(visiting.connections.get(j).start.x == visiting.neighbors.get(i).connections.get(k).end.x &&
                          visiting.connections.get(j).start.y == visiting.neighbors.get(i).connections.get(k).end.y &&
                          visiting.connections.get(j).end.x == visiting.neighbors.get(i).connections.get(k).start.x &&
                          visiting.connections.get(j).end.y == visiting.neighbors.get(i).connections.get(k).start.y){
                            previousMid = visiting.connections.get(j).center();
                            exit = true;
                      }
                      
                      if(exit) break;
                  }
                  
                  if(exit) break;
                }
                
                // assign the previous mid point
                visiting.neighbors.get(i).prevPoint = previousMid;
                
                // add to frontier
                frontier.add(visiting.neighbors.get(i));
                
            }
            
         }
      }
      
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
      /// use this to draw the nav mesh graph
      for(Node n : navMesh){
         stroke(n.r, n.g, n.b);
         
         //draws the polygon
         for(Wall w : n.polygon){
           line(w.start.x, w.start.y,  w.end.x, w.end.y); 
         }
         
         stroke(233, 22, 233);
         fill(233, 0, 233);
         circle(n.center.x, n.center.y, 7);
         
         //draws the lines from center to midpoint edge
         for( Wall w : n.connections){
           line(w.center().x, w.center().y, n.center.x, n.center.y);
         }
      }
      
      // find reflex angles to draw
      for(int i = 0; i < map.walls.size(); ++i){
         int next = i + 1;
          
         if(next >= map.walls.size()){
           next = 0;
         }
          
         float value = PVector.dot(map.walls.get(i).normal, map.walls.get(next).direction);
          
         if(value > 0){
           PVector angle = new PVector(map.walls.get(i).end.x, map.walls.get(i).end.y);
           stroke(23, 212, 233);
           fill(23, 212, 233);
           circle(angle.x, angle.y, 10);
         }
       }
   }
}
