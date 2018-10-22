class Morphology{
  
  float[][] RelAng;
  
  Morphology(){
    RelAng = new float[][]{ {0, 0, 0, 0}, 
                             {0, 0, -PI, -PI},
                             {PI, 0, -PI/2.0, PI/2.0},
                             {PI, 0, 0, 0},
                             {PI, 0, 0, PI},
                             {PI/2.0, 0, -PI, 0},
                             {PI, 0, -PI/2.0, -PI/2.0}};
  }                          
}

class MapDB{
  
  PVector mapSize;
  Map[] Maps;
  Obstacle[] Obss;
  Waypoint[] Wps;
  boolean gridBasedMode ;
  
  MapDB(){
    
    Maps = new Map[5];
    
    // ---------map 01----------
    mapSize = new PVector(800, 800);
    
    gridBasedMode = true;
    
    Wps = new Waypoint[4];
    Wps[0] = new Waypoint(new PVector(700, 700), 2); 
    Wps[1] = new Waypoint(new PVector(160, 500), 2);
    Wps[2] = new Waypoint(new PVector(660, 300), 2);
    Wps[3] = new Waypoint(new PVector(120, 120), 2);
    
    Obss = new Obstacle[3];
    Obss[0] = new Obstacle(new PVector(0, 300), new PVector(500, 10));
    Obss[1] = new Obstacle(new PVector(300,500), new PVector(500, 10));
    Obss[2] = new Obstacle(new PVector(500,100), new PVector(100, 100));
    
    
    Maps[0] = new Map(mapSize, gridBasedMode, Wps, Obss);
    
    // ---------map 02----------
    mapSize = new PVector(800,800);
    
    gridBasedMode = true;
    
    Wps = new Waypoint[2];
    Wps[0] = new Waypoint(new PVector(25*blkWidth, 13*blkWidth), 0); 
    Wps[1] = new Waypoint(new PVector(4*blkWidth, 4*blkWidth), 0);
    
    Obss = new Obstacle[2];
    Obss[0] = new Obstacle(new PVector(350, 0), new PVector(100, 600));
    Obss[1] = new Obstacle(new PVector(350, 700), new PVector(100, 500));
    
    Maps[1] = new Map(mapSize, gridBasedMode, Wps, Obss);
    
    // ---------map 03----------
    mapSize = new PVector(800,800);
    
    gridBasedMode = true;
    
    Wps = new Waypoint[2];
    Wps[0] = new Waypoint(new PVector(28*blkWidth, 28*blkWidth), 0); 
    Wps[1] = new Waypoint(new PVector(4*blkWidth, 4*blkWidth), 0);
    
    Obss = new Obstacle[2];
    Obss[0] = new Obstacle(new PVector(10*blkWidth, 0), new PVector(2*blkWidth, 26*blkWidth));
    Obss[1] = new Obstacle(new PVector(20*blkWidth, 6*blkWidth), new PVector(2*blkWidth, 26*blkWidth));
    
    Maps[2] = new Map(mapSize, gridBasedMode, Wps, Obss);
    
        // ---------map 04----------
    mapSize = new PVector(800,800);
    
    gridBasedMode = true;
     
    Wps = new Waypoint[2];
    Wps[0] = new Waypoint(new PVector(28*blkWidth, 28*blkWidth), 0); 
    Wps[1] = new Waypoint(new PVector(4*blkWidth, 20*blkWidth), 0);
    
    Obss = new Obstacle[2];
    Obss[0] = new Obstacle(new PVector(10*blkWidth, 0), new PVector(2*blkWidth, 20*blkWidth));
    Obss[1] = new Obstacle(new PVector(20*blkWidth, 6*blkWidth), new PVector(2*blkWidth, 20*blkWidth));
    
    Maps[3] = new Map(mapSize, gridBasedMode, Wps, Obss);
    
    // ---------map 05----------
    mapSize = new PVector(800,800);
    
    gridBasedMode = true;
     
    Wps = new Waypoint[2];
    Wps[0] = new Waypoint(new PVector(28*blkWidth, 28*blkWidth), 0); 
    Wps[1] = new Waypoint(new PVector(4*blkWidth, 4*blkWidth), 0);
    
    Obss = new Obstacle[3];
    Obss[0] = new Obstacle(new PVector(10*blkWidth, 10*blkWidth), new PVector(12*blkWidth, 2*blkWidth));
    Obss[1] = new Obstacle(new PVector(20*blkWidth, 6*blkWidth), new PVector(2*blkWidth, 10*blkWidth));
    Obss[2] = new Obstacle(new PVector(10*blkWidth, 20*blkWidth), new PVector(3*blkWidth, 3*blkWidth));
    
    Maps[4] = new Map(mapSize, gridBasedMode, Wps, Obss);
  }
}