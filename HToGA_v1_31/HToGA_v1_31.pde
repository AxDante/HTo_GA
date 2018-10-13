Population test;
MapDB mapDB;
Map map;
int mapID = 1;
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


float baseMutMoveRate = 0.01; //0.05; // Mutation rate of robot moving direction
float baseMutTransRate = 0; //0.005; // Mutation rate of robt transformation
float baseCrossoverRate = 0.5;
float moveTransRatio = 100.0;

float bestPercentage = 0.1;


int frameRefreshRate = 500;
int totPopulation = 100;
float blkWidth = 25;

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
  for (int wpidx = 0; wpidx < map.Wps.length ; wpidx++){
    ellipse(map.Wps[wpidx].pos.x, map.Wps[wpidx].pos.y, 10, 10);
  }
  
  // draw obstacle(s)
  fill(0, 0, 255);
  for (int intobs = 0; intobs < mapDB.Obss.length; intobs++){
    rect(map.Obss[intobs].pos.x, map.Obss[intobs].pos.y, map.Obss[intobs].size.x, map.Obss[intobs].size.y);
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
      println("Navigation " + test.gen + " result:" );
      test.calculateFitness();
      test.naturalSelection();
      test.GAMutation();
      test.GACrossover();
      println("==================================");
      if (test.isConverged()){
        if (currentWpID < map.Wps.length-2){
          println("GA converged! Now navigating from wp " + currentWpID + " to wp " + currentWpID+1);
          currentWpID += 1;
          test = new Population(totPopulation, 1000, map.Obss);
        }else{
          println("GA converged! Waypoint navigation process terminates.");
          terminate = true;
        }
      }
      time = 0;
    } else {
      test.update();
      test.show();
      time++;
    }
  }
}
