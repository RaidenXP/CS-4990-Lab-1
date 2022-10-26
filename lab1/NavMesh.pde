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
   ArrayList<Wall> navEdges;
   ArrayList<ArrayList<Wall>> polygons;
   
   NavMesh()
   {
       navMesh = new ArrayList<Node>();
       reflexAng = new ArrayList<PVector>();
       navEdges = new ArrayList<Wall>();
       polygons = new ArrayList<ArrayList<Wall>>();
   }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       
       // set up/reset for each map
       reflexAng.clear();
       navMesh.clear();
       navEdges.clear();
       polygons.clear();
       
       Node first = new Node();
       first.id = 0;
       first.polygon = new ArrayList<Wall>(map.walls);
       
       navMesh.add(first);
       
       // find reflex angles
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
       
       ArrayList<Wall> firstPoly = new ArrayList<Wall>(map.walls);
       
       // split the initial polygon into two
       if(reflexAng.size() > 0){
         genGraph(firstPoly);
       }
   }
   
   void genGraph(ArrayList<Wall> polygon){
     //reflex angle
     int wallIndex = -1; //<>//
     ArrayList<Wall> freshPolygon = new ArrayList<Wall>();
     ArrayList<Wall> otherPolygon = new ArrayList<Wall>();
     
     //find first reflex angle for current polygon
     for(int i = 0; i < polygon.size(); ++i){
          int next = i + 1;
          
          if(next >= polygon.size()){
            next = 0;
          }
          
          float value = PVector.dot(polygon.get(i).normal, polygon.get(next).direction);
          
          if(value > 0){
              wallIndex = i;
              for(int j = 0; j < polygon.size(); ++ j){
                 PVector direction = PVector.sub(polygon.get(j).start, polygon.get(i).end);  
                 PVector shortStart = PVector.add(polygon.get(i).end, PVector.mult(direction, 0.01));
                 PVector shortEnd = PVector.add(polygon.get(j).start, PVector.mult(direction, -0.01));
                 
                 Wall temp = new Wall(polygon.get(i).end, polygon.get(j).start);
                 if(placeable(map, polygon, shortStart, shortEnd, temp)){
                     Wall tempPrime = new Wall(polygon.get(j).start, polygon.get(i).end);
                     
                     int k = 0;
                     if(j == 0){
                         k = polygon.size() - 1; 
                     }
                     else{
                         k = j - 1; 
                     }
                     
                     while(i != j){
                         freshPolygon.add(polygon.get(j));
                         ++j;
                         
                         if(j >= polygon.size()){
                             j = 0; 
                         }
                     }
                     
                     freshPolygon.add(polygon.get(j));
                     
                     ++j;
                     if(j >= polygon.size()){
                       j = 0; 
                     }
                     
                     while(j != k){
                         otherPolygon.add(polygon.get(j));
                         ++j;
                         
                         if(j >= polygon.size()){
                             j = 0; 
                         }
                     }
                     
                     otherPolygon.add(polygon.get(j));
                     
                     freshPolygon.add(temp);
                     otherPolygon.add(tempPrime);
                     break;
                 }
              }
              break;
          }
     }
     
     if(wallIndex != -1){
         genGraph(freshPolygon);
         genGraph(otherPolygon);
     }
     else{
         polygons.add(polygon);
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
   }
   
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
      
      //for(Node n : navMesh){
      //   stroke(r += 50,g += 30,b -= 45);
      //   for(Wall w : n.polygon){
      //     line(w.start.x, w.start.y,  w.end.x, w.end.y); 
      //   }
      //}
      
      for(ArrayList<Wall> p : polygons){
          for(Wall w : p){
            line(w.start.x, w.start.y,  w.end.x, w.end.y); 
          }
      }
   }
}
