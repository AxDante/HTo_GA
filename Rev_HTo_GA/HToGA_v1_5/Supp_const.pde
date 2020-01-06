class MapDB{
  
  PVector mapSize;
  
  ArrayList<Map> mapList = new ArrayList<Map>();
  ArrayList<DynamicObstacle> DyObssList;
  ArrayList<Obstacle> ObsList;
  ArrayList<Waypoint> WpList;
  
  boolean gridBasedMode ;
  
  MapDB(){
    
   // ---------map 0---------- (H shape)
    mapSize = new PVector(600,600);  //24*24
    
    gridBasedMode = true;
     
    WpList = new ArrayList<Waypoint>();
    WpList.add(new Waypoint(new PVector(11*blkW, 7*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(11*blkW, 15*blkW), 0));
    
    ObsList = new ArrayList<Obstacle>();
    ObsList.add(new Obstacle(new PVector(5*blkW, 5*blkW), new PVector(2*blkW, 14*blkW)));
    ObsList.add(new Obstacle(new PVector(17*blkW, 5*blkW), new PVector(2*blkW, 14*blkW)));
    ObsList.add(new Obstacle(new PVector(5*blkW, 11*blkW), new PVector(12*blkW, 2*blkW)));
    
    DyObssList = new ArrayList<DynamicObstacle>();
    mapList.add(new Map(mapSize, gridBasedMode, WpList, ObsList, DyObssList));

    // ---------map 1---------- (Spiral)
    mapSize = new PVector(600,600);  //24*24
    
    gridBasedMode = true;
     
    WpList = new ArrayList<Waypoint>();
    WpList.add(new Waypoint(new PVector(1*blkW, 1*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(8*blkW, 4*blkW), 0));
    
    ObsList = new ArrayList<Obstacle>();
    ObsList.add(new Obstacle(new PVector(0*blkW, 4*blkW), new PVector(20*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(18*blkW, 4*blkW), new PVector(2*blkW, 16*blkW)));
    ObsList.add(new Obstacle(new PVector(4*blkW, 18*blkW), new PVector(16*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(4*blkW, 10*blkW), new PVector(2*blkW, 10*blkW)));
    ObsList.add(new Obstacle(new PVector(6*blkW, 10*blkW), new PVector(8*blkW, 2*blkW)));
    
    DyObssList = new ArrayList<DynamicObstacle>();
    mapList.add(new Map(mapSize, gridBasedMode, WpList, ObsList, DyObssList));
    
    // ---------map 2---------- (3-slit)
    mapSize = new PVector(600,600);  //24*24
    
    gridBasedMode = true;
     
    WpList = new ArrayList<Waypoint>();
    WpList.add(new Waypoint(new PVector(5*blkW, 5*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(5*blkW, 18*blkW), 0));
    
    ObsList = new ArrayList<Obstacle>();
    ObsList.add(new Obstacle(new PVector(11*blkW, 0*blkW), new PVector(2*blkW, 5*blkW)));
    ObsList.add(new Obstacle(new PVector(11*blkW, 6*blkW), new PVector(2*blkW, 12*blkW)));
    ObsList.add(new Obstacle(new PVector(11*blkW, 19*blkW), new PVector(2*blkW, 5*blkW)));
    ObsList.add(new Obstacle(new PVector(0*blkW, 11*blkW), new PVector(18*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(19*blkW, 11*blkW), new PVector(5*blkW, 2*blkW)));
    
    DyObssList = new ArrayList<DynamicObstacle>();
    mapList.add(new Map(mapSize, gridBasedMode, WpList, ObsList, DyObssList));
    
    
    // ---------map 3---------- (Random Obs)
    mapSize = new PVector(600,600);  //24*24
    
    gridBasedMode = true;
     
    WpList = new ArrayList<Waypoint>();
    WpList.add(new Waypoint(new PVector(20*blkW, 3*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(3*blkW, 20*blkW), 0));
    
    ObsList = new ArrayList<Obstacle>();
    ObsList.add(new Obstacle(new PVector(4*blkW, 4*blkW), new PVector(3*blkW, 3*blkW)));
    ObsList.add(new Obstacle(new PVector(13*blkW, 6*blkW), new PVector(6*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(11*blkW, 10*blkW), new PVector(2*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(7*blkW, 13*blkW), new PVector(2*blkW, 7*blkW)));
    ObsList.add(new Obstacle(new PVector(14*blkW, 15*blkW), new PVector(3*blkW, 4*blkW)));
    ObsList.add(new Obstacle(new PVector(16*blkW, 15*blkW), new PVector(5*blkW, 2*blkW)));
    ObsList.add(new Obstacle(new PVector(3*blkW, 9*blkW), new PVector(3*blkW, 2*blkW)));
    
    DyObssList = new ArrayList<DynamicObstacle>();
    mapList.add(new Map(mapSize, gridBasedMode, WpList, ObsList, DyObssList));
    
    
    // ---------map 4---------- (2 dynamic obs)
    mapSize = new PVector(600,600);
    
    gridBasedMode = true;
     
    WpList = new ArrayList<Waypoint>();
    WpList.add(new Waypoint(new PVector(4*blkW, 4*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(4*blkW, 19*blkW), 0));
    WpList.add(new Waypoint(new PVector(19*blkW, 19*blkW), 0)); 
    WpList.add(new Waypoint(new PVector(19*blkW, 4*blkW), 0));
    
    
    ObsList = new ArrayList<Obstacle>();

    DyObssList = new ArrayList<DynamicObstacle>();
    DyObssList.add(new DynamicObstacle(new PVector(3, 10).mult(blkW), new PVector(18, 4).mult(blkW),
                                    1, 0, 25, 10));
    DyObssList.add(new DynamicObstacle(new PVector(10, 3).mult(blkW), new PVector(4, 18).mult(blkW),
                                    0, -1, 25, 10));
    mapList.add(new Map(mapSize, gridBasedMode, WpList, ObsList, DyObssList));
  
    
  }
}


class Morphology{
  
  float[][] RelAng;
  
  Morphology(){
  RelAng = new float[][]{ {0, 0, 0, 0},               // I-shape        -(1)
                          {0, 0, -PI, -PI},           // O-shape        -(2)
                          {PI, 0, -PI/2.0, PI/2.0},   // L-shape        -(3)
                          {PI, 0, 0, 0},              // J-shape        -(4)
                          {PI, 0, 0, PI},             // S-shape        -(5)
                          {PI/2.0, 0, -PI, 0},        // T-shape        -(6)
                          {PI, 0, -PI/2.0, -PI/2.0},  // Z-shape        -(7)
                          {PI/2.0, PI/2.0, PI/2.0, PI/2.0}};         // I-shape(90deg) -(8)
  }                          
}

class SearchPattern{
  ArrayList<PVector[]> SP = new ArrayList<PVector[]>();
  
  SearchPattern(){
    // dist = 1 array
    PVector[] sp0 = new PVector[]{new PVector(0,-blkW), new PVector(blkW, 0), 
                        new PVector(0, blkW), new PVector(-blkW,0)};
    SP.add(sp0);
    
    // dist = 2 array
    PVector[] sp1 = new PVector[]{new PVector(0,-blkW), new PVector(blkW, -blkW), 
                                new PVector(blkW, 0), new PVector(blkW, blkW), 
                                new PVector(0, blkW), new PVector(-blkW, blkW), 
                                new PVector(-blkW,0), new PVector(-blkW, -blkW)};
    SP.add(sp1);
  
    // dist = 3 array
    PVector[] sp2 = new PVector[]{new PVector(0,-blkW), new PVector(blkW, -blkW), 
                                new PVector(blkW, 0), new PVector(blkW, blkW), 
                                new PVector(0, blkW), new PVector(-blkW, blkW), 
                                new PVector(-blkW,0), new PVector(-blkW, -blkW),
                                new PVector(0, 2*blkW), new PVector(-2*blkW, 0), 
                                new PVector(2*blkW,0), new PVector(0, -2*blkW)};
    SP.add(sp2);
  
  }   
}
