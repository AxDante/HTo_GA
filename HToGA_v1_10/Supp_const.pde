final class Morphology{
  
  static float[][] RelAng;
  
  Morphology(){
    RelAng = new float[][]{{0, 0, 0, 0}, 
                             {0, 0, -PI, -PI},
                             {PI, 0, -PI/2.0, PI/2.0},
                             {PI, 0, 0, 0},
                             {PI, 0, 0, PI},
                             {PI/2.0, 0, -PI, 0},
                             {PI, 0, -PI/2.0, -PI/2.0}};
  }
                             
}

final class MapDB{
  
  static Map[] Maps;
  Obstacle[] Obss;
  Waypoint[] Wps;
  boolean gridBasedMode = false;
  
  MapDB(){
      
    Wps = new Waypoint[4];
    Wps[0] = new Waypoint(new PVector(700, 700), 2); 
    Wps[1] = new Waypoint(new PVector(160, 500), 2);
    Wps[2] = new Waypoint(new PVector(660, 300), 2);
    Wps[3] = new Waypoint(new PVector(200, 160), 2);
    
    Obss = new Obstacle[3];
    Obss[0] = new Obstacle(new PVector(0, 300), new PVector(500, 10));
    Obss[1] = new Obstacle(new PVector(300,500), new PVector(500, 10));
    Obss[2] = new Obstacle(new PVector(500,100), new PVector(100, 100));
    
    Maps = new Map[1];
    Maps[0] = new Map(gridBasedMode, Wps, Obss);
    
  }
}
