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
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
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
      for(int i = 0; i < map.outline.size(); ++i){
        int next = i + 1;
        
        if(next >= map.outline.size()){
          next = 0;
        }
        
        float value = PVector.dot(map.outline.get(i).normal, map.outline.get(next).direction);
        
        if(value > 0){
          //println(value + "," + map.outline.get(i).normal + "," + map.outline.get(next).direction + "," + i);
          stroke(23, 212, 233);
          circle(map.outline.get(i).end.x, map.outline.get(i).end.y, 10);
        }
      }
   }
}
