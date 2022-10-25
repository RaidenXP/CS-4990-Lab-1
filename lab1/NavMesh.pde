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
}



class NavMesh
{ 
   ArrayList<Node> navMesh;
   ArrayList<PVector> reflexAng;
   
   NavMesh()
   {
       navMesh = new ArrayList<Node>();
       
       Node first = new Node();
       first.id = 0;
       first.polygon = map.walls;
       
       navMesh.add(first);
       
       reflexAng = new ArrayList<PVector>();
   }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       reflexAng.clear();
       
       navMesh.clear();
       
       Node first = new Node();
       first.id = 0;
       first.polygon = map.walls;
       
       navMesh.add(first);
       
       for(int i = 0; i < navMesh.get(0).polygon.size(); ++i){
          int next = i + 1;
          
          if(next >= navMesh.get(0).polygon.size()){
            next = 0;
          }
          
          float value = PVector.dot(navMesh.get(0).polygon.get(i).normal, navMesh.get(0).polygon.get(next).direction);
          
          if(value > 0){
            PVector angle = new PVector(navMesh.get(0).polygon.get(i).end.x, navMesh.get(0).polygon.get(i).end.y);
            reflexAng.add(angle);
          }
       }
       
       for(int i = 0; i < reflexAng.size(); ++i){
           PVector start = new PVector(reflexAng.get(i).x, reflexAng.get(i).y);
           for(Wall w : navMesh.get(0).polygon){
               PVector direction = PVector.sub(w.start, start);  
               PVector shortStart = PVector.add(start, PVector.mult(direction, 0.01));
               PVector shortEnd = PVector.add(w.start, PVector.mult(direction, -0.01));
             
               if(placeable(map, navMesh.get(0).polygon, shortStart, shortEnd)){
                  Node next = new Node();
                  next.id = navMesh.size();
                  next.polygon = new ArrayList<Wall>();
                  next.polygon.add(new Wall(start, w.start));
                  navMesh.add(next);
                  break;
               }
           }
       }
   }
   
   boolean placeable(Map map, ArrayList<Wall> polygon, PVector from, PVector to){
     boolean flag = false;
     
     for(Wall w : polygon){
         flag = !(w.crosses(from,to)); 
     }
     
     return flag && map.isReachable(to) && !map.collides(from, to);
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
   }
   
   void draw()
   {
      /// use this to draw the nav mesh graph
      for(int i = 0; i < reflexAng.size(); ++i){ //<>//
          stroke(23, 212, 233);
          circle(reflexAng.get(i).x, reflexAng.get(i).y, 10);
      }
      
      for(Node n : navMesh){
         for(Wall w : n.polygon){
           line(w.start.x, w.start.y,  w.end.x, w.end.y); 
         }
      }
   }
}
