Population test;
MapDB mapDB;
Map map;
int mapID = 0;
int currentWpID = 0;
int time = 0;

boolean debugMode = false;
boolean gridBasedMode = true;
boolean terminate = false;

boolean mutTransInitialze = false;
boolean mutTransProcess = false;


// These toggle options are under gridBasedMode
boolean noRepeatingGrids = true;


float rotAngVel = PI/12.0;  // Rotation angular velocity
float rotThreshold = PI/22.0; // Rotation threshold angle for the robot to stop 
float baseMutMoveRate = 0.005; // Mutation rate of robot moving direction
float baseMutTransRate = 0.0001; // Mutation rate of robt transformation
float moveTransRatio = 300.0;

int frameRefreshRate = 100;
int totPopulation = 10;
float blkWidth = 20;

void settings() {
  
  mapDB = new MapDB();
  map = mapDB.Maps[mapID];
  
  size((int)map.mapSize.x, (int)map.mapSize.y); //size of the window
  
}

void setup(){
  
  test = new Population(totPopulation, 1000 , map.Obss);
  frameRate(frameRefreshRate);
  
}

void draw() { 
  
  background(255);
  // draw goal
  fill(255, 0, 0);
  for (int wpidx = 0; wpidx < mapDB.Wps.length ; wpidx++){
    ellipse(mapDB.Wps[wpidx].pos.x, mapDB.Wps[wpidx].pos.y, 10, 10);
  }
  
  // draw obstacle(s)
  fill(0, 0, 255);
  for (int intobs = 0; intobs < mapDB.Obss.length; intobs++){
    rect(mapDB.Obss[intobs].pos.x, mapDB.Obss[intobs].pos.y, mapDB.Obss[intobs].size.x, mapDB.Obss[intobs].size.y);
  }
  
  // draw grid
  stroke(125);
  for (int rowidx = 0; rowidx <= (int)map.mapSize.x/blkWidth; rowidx++){
    line(rowidx*blkWidth, 0, rowidx*blkWidth, map.mapSize.y);
  }
  for (int colidx = 0; colidx <= (int)map.mapSize.y/blkWidth; colidx++){
    line(0, colidx*blkWidth,  map.mapSize.x, colidx*blkWidth);
  }
  
  //for(int wpidx = 0; wpidx < map.Wps.length; wpidx++){
  if (!terminate){
    if (test.allRobotsDead()) {
      println("Navigation " + test.gen + " result:");
      test.calculateFitness();
      test.naturalSelection();
      test.mutateDemBabies();
      println("==================================");
      time = 0;
    } else {
      test.update();
      test.show();
      time++;
    }
    
    if (test.isConverged()){
      time = 0;
      if (currentWpID < map.Wps.length-2){
        println("GA converged! Now navigating from wp " + currentWpID + " to wp " + currentWpID+1);
        currentWpID += 1;
        test = new Population(totPopulation, 1000, map.Obss);
      }else{
        println("GA converged! Waypoint navigation process terminates.");
        terminate = true;
      }
    }
  }
}
