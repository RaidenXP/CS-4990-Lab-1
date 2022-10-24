// Useful to sort lists by a custom key
import java.util.Comparator;
import java.util.Iterator;

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
      // for(int i = 0; i < map.outline.size(); ++i){
      //  int next = i + 1;
        
      //  if(next >= map.outline.size()){
      //    next = 0;
      //  }
      //  float value = PVector.dot(map.outline.get(i).normal, map.outline.get(next).direction);
      //  if(value > 0){
          
      //  }
      //}
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
   
   boolean collides(ArrayList<Wall> outline, PVector from, PVector to)
   {
      for (Wall w : outline)
      {
         if (w.crosses(from, to)) return true;
      }
      return false;
   }
   
   void draw()
   {
      /// use this to draw the nav mesh graph
       ArrayList<PVector> vertices = new ArrayList<>();
       for (int i = 0; i < map.outline.size() - 1; i++) 
       {
         if (vertices.isEmpty())
         {
           vertices.add(map.outline.get(i).start);
         }
          vertices.add(map.outline.get(i).end);
      }
      for(int i = 0; i < map.outline.size(); ++i) {
          int next = i + 1;
          if(next >= map.outline.size()){
            next = 0;
          }
          
          float value = PVector.dot(map.outline.get(i).normal, map.outline.get(next).direction);
          
          if(value > 0){
            PVector start = map.outline.get(i).end;
            stroke(0, 255, 0);
            circle(start.x, start.y, 10);
        }
      }
      ArrayList<Wall> newEdges = new ArrayList<>();
      //while (!nonConvex.isEmpty()) {
      //  Iterator<PVector> iterator = nonConvex.listIterator();
      //  while (iterator.hasNext()) {
      //    PVector start = iterator.next();
        //println(value + "," + map.outline.get(i).normal + "," + map.outline.get(next).direction + "," + i);
      for (int i = 0; i < vertices.size(); i++) {
          PVector start = vertices.get(i);
          for (int j = i - 2; j > -1; j--)
          {
           PVector end = vertices.get(j);
           PVector direction = PVector.sub(end, start);
           PVector shortStart = PVector.add(start, PVector.mult(direction, 0.01));
           PVector shortEnd = PVector.add(end, PVector.mult(direction, -0.01));
           // if (map.isReachable(PVector.mult(PVector.add(start, end), 0.5)) && !map.collides(shortStart, shortEnd) && !collides(map.outline, shortStart, shortEnd)) {
           if (map.isReachable(PVector.mult(PVector.add(start, end), 0.5)) && !map.collides(shortStart, shortEnd) && !collides(newEdges, shortStart, shortEnd)) {  
             line(start.x, start.y, end.x, end.y);
             newEdges.add(new Wall(start, end));
             // j = -1;
           }
         }
          }
        }
     }
