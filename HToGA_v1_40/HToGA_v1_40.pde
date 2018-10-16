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
int mapW, mapH;
float diag = (float)Math.sqrt(2);

Grid[][] grids;
ArrayList<Grid> open = new ArrayList<Grid>();
ArrayList<Grid> closed = new ArrayList<Grid>();

void settings() {
  
  mapDB = new MapDB();
  map = mapDB.Maps[mapID];
  
  size((int)map.mapSize.x, (int)map.mapSize.y); //size of the window

}

void setup(){
  test = new Population(totPopulation, 1000 , map.Obss);
  frameRate(frameRefreshRate);
  
  mapW = (int)(map.mapSize.x/blkWidth);
  mapH = (int)(map.mapSize.y/blkWidth);
  grids = new Grid[mapH][mapW];
  
  for (int i = 0; i < grids.length; i++) {
    for (int j = 0; j < grids[i].length; j++) {
      grids[i][j] = new Grid (i, j, map.Obss);
      grids[i][j].heuristic();
    }
  }
  
  
  closed.add(grids[0][0]);
  grids[0][0].g = 0;
  grids[0][0].checked = true;
  grids[0][0].link();
  grids[mapW-1][mapH-1].isWall = false;
  
}

void draw() { 
  
  /*
  background(255);
  // draw goal
  fill(255, 0, 0);
  for (int wpidx = 0; wpidx < map.Wps.length ; wpidx++){
    ellipse(map.Wps[wpidx].pos.x, map.Wps[wpidx].pos.y, 10, 10);
  }
  
  // draw obstacle(s)
  fill(0, 0, 255);
  for (int intobs = 0; intobs < map.Obss.length; intobs++){
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
  */
  
    background(255);
  for (int i = 0; i < grids.length; i++) {
    for (int j = 0; j < grids[i].length; j++) {
      grids[i][j].calcF();
      
      if (grids[i][j].isWall) {
        fill(0);
        rect((i)*(blkWidth), j*(blkWidth), blkWidth, blkWidth);
      } else if (grids[i][j].checked) {
        fill(153);
        rect((i)*(blkWidth), j*(blkWidth), blkWidth, blkWidth);
      } else {
        fill(255);
        rect((i)*(blkWidth), j*(blkWidth), blkWidth, blkWidth);
      }
      fill(0);
      textSize(32);
      //  text("" + points[i][j].f, (i)*blkWidth, (j+1)*blkWidth);
    }
  }
  
  fill(255, 0, 0);
  rect(0, 0, blkWidth, blkWidth);
  
  fill(0, 255, 0);
  rect((mapW-1)*blkWidth, (mapH-1)*blkWidth, blkWidth, blkWidth);
  
  if (open.size() == 0) {
    noLoop();
  }
  
  Grid lowest = getMin();
  if (lowest == null) {
    System.out.println("?");
    noLoop();
  }
  closed.add(lowest);
  open.remove(lowest);
  lowest.link();
  lowest.show();
  
  if (open.size() == 0) {
    noLoop();
    println("Not solvable");
  }
  
  for (int i = 0; i < open.size(); i++) {
    if (open.get(i) == grids[mapH-1][mapW-1]) {
      open.get(i).show();
      noLoop();
    }
  }
  
}
