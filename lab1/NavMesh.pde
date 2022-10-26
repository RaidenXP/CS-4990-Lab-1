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
   
   NavMesh()
   {
       navMesh = new ArrayList<Node>();
       reflexAng = new ArrayList<PVector>();
       navEdges = new ArrayList<Wall>();
   }
   
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       
       // set up/reset for each map
       reflexAng.clear();
       navMesh.clear();
       navEdges.clear();
       
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
       
       // split the initial polygon into two
       if(reflexAng.size() > 0){
         genGraph(navMesh.get(0));
       }
   }
   
   void genGraph(Node n){
     //reflex angle
     PVector tempAng = null; //<>//
     
     //find first reflex angle for current polygon
     for(int i = 0; i < n.polygon.size(); ++i){
          int next = i + 1;
          
          if(next >= n.polygon.size()){
            next = 0;
          }
          
          float value = PVector.dot(n.polygon.get(i).normal, n.polygon.get(next).direction);
          
          if(value > 0){
            PVector angle = new PVector(n.polygon.get(i).end.x, n.polygon.get(i).end.y);
            tempAng = angle;
            break;
          }
     }
     
     if(tempAng != null){
         PVector start = tempAng;
         Node next = new Node();
         
         //place line
         for(Wall w : n.polygon){
             PVector direction = PVector.sub(w.start, start);  
             PVector shortStart = PVector.add(start, PVector.mult(direction, 0.01));
             PVector shortEnd = PVector.add(w.start, PVector.mult(direction, -0.01));
             
             Wall temp = new Wall(start, w.start);
             
             if(placeable(map, n.polygon, shortStart, shortEnd, temp)){
                 next.id = navMesh.size();
                 next.polygon = new ArrayList<Wall>();
                 next.polygon.add(temp);
                 
                 navEdges.add(temp);
                 
                 //if(next.polygon.get(0).start.x == n.polygon.get(0).start.x && next.polygon.get(0).start.y == n.polygon.get(0).start.y
                 //  && next.polygon.get(0).end.x == n.polygon.get(0).end.x && next.polygon.get(0).end.y == n.polygon.get(0).end.y)
                 //{
                 //  continue; 
                 //}
                 
                 navMesh.add(next);
                 break;
             }
         }
        
         //create two new polygons process starts here
         PVector point = new PVector(next.polygon.get(0).end.x, next.polygon.get(0).end.y);
         int index = 0;
        
         for(int i = 0; i < n.polygon.size(); ++i){
            if(point.x == n.polygon.get(i).start.x && point.y == n.polygon.get(i).start.y){
                index = i;
                break;
            }
         }
        
         // shared edge reversed
         n.polygon.add(new Wall(next.polygon.get(0).end, next.polygon.get(0).start));
        
         //create the polygon starting from the end point of new edge coming back to the start of new edge
         while(point.x != start.x || point.y != start.y){
             
             // something may be wrong with this logic
             if(index >= n.polygon.size() - 1){
                index = 0; 
             }
          
             point = n.polygon.get(index).start;
            
             if(point.x != start.x || point.y != start.y){
               next.polygon.add(n.polygon.get(index));
               n.polygon.remove(index);
             } 
         }
         
         // split the polygon in two again until doesn't have to split anymore
         genGraph(next);
         genGraph(n);
     }
     else{
       return; 
     }
   }
   
   
   boolean placeable(Map map, ArrayList<Wall> polygon, PVector from, PVector to, Wall tempW){
     boolean flag = false;
     boolean otherFlag = true;
     
     for(Wall w : polygon){
         flag = !(w.crosses(from,to)); 
     }
     
     for(Wall w : navEdges){
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
      
      float r = 23;
      float g = 212;
      float b = 233;
      
      for(Node n : navMesh){
         stroke(r += 50,g += 30,b -= 45);
         for(Wall w : n.polygon){
           line(w.start.x, w.start.y,  w.end.x, w.end.y); 
         }
      }
   }
}
