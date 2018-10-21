Population test;
MapDB mapDB;
Map map;

int mapID = 3;
int currentWpID = 0;
int time = 0;

boolean debugMode = false;
boolean terminate = false;

boolean mutTransInitialze = false;
boolean mutTransProcess = false;

boolean progressingFitness = true;

boolean noRepeatingGrids = true;

float rotAngVel = PI/12.0;  // Rotation angular velocity
float rotThreshold = PI/22.0; // Rotation threshold angle for the robot to stop 

float baseMutMoveRate = 0.03; //0.05; // Mutation rate of robot moving direction
float MutMoveRate;
float baseMutTransRate = 0; //0.005; // Mutation rate of robt transformation
float baseCrossoverRate = 0.0;
float moveTransRatio = 100.0;

float bestPercentage = 0.1;

int frameRefreshRate = 1000;
int totPopulation = 100;
float blkWidth = 25;
int mapW, mapH;
float diag = (float)Math.sqrt(2);

Grid[][] grids;
int[] startGrid;
int[] goalGrid;
ArrayList<Grid> open = new ArrayList<Grid>();
ArrayList<Grid> closed = new ArrayList<Grid>();
double[][] AstarFitness;

int[][] gridObsTable;
PFont dispFont;

PVector[] fourDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth, 0), new PVector(0, blkWidth), new PVector(-blkWidth,0)};
PVector[] eightDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth, -blkWidth), new PVector(blkWidth, 0), new PVector(blkWidth, blkWidth), 
                                         new PVector(0, blkWidth), new PVector(-blkWidth, blkWidth), new PVector(-blkWidth,0), new PVector(-blkWidth, -blkWidth)};

String[] fourDirString = new String[]{"F", "R", "B", "L"};
String[] eightDirString = new String[]{"F", "FR", "R", "BR", "B", "BL", "L", "FL"};

void settings() {
  
  mapDB = new MapDB();
  map = mapDB.Maps[mapID];
  
  size((int)map.mapSize.x, (int)map.mapSize.y); //size of the window

}

void setup(){
  
  dispFont = createFont("Arial",8,true); 
  test = new Population(totPopulation, 1000 , map.Obss);
  frameRate(frameRefreshRate);
  
  mapW = floor(map.mapSize.x/blkWidth);
  mapH = floor(map.mapSize.y/blkWidth);
  
  updateObstacleTable();
  updateFitnessTable(map.Wps.length-1);
}

void draw() { 
  
  
  background(255);
  fill(0);
  textAlign(LEFT); 
  textFont(dispFont);
  for (int i = 0; i < grids.length; i++) {
    for (int j = 0; j < grids[i].length; j++) {
      text("("+i+","+j+")", i*blkWidth, j*blkWidth+18);
    }
  }
  
  
  // Draw waypoints
  
  fill(255, 0, 0);
  for (int wpidx = 0; wpidx < map.Wps.length ; wpidx++){
    rect(map.Wps[wpidx].pos.x, map.Wps[wpidx].pos.y, blkWidth, blkWidth);
  }
  fill(255, 153, 50);
  rect(map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y, blkWidth, blkWidth);
  
  
  // draw obstacle(s)
  fill(0, 0, 255);
  for (int intobs = 0; intobs < map.Obss.length; intobs++){
    rect(map.Obss[intobs].pos.x, map.Obss[intobs].pos.y, map.Obss[intobs].size.x, map.Obss[intobs].size.y);
  }
 
  // draw grids
  stroke(125);
  for (int rowidx = 0; rowidx <= (int)map.mapSize.x/blkWidth; rowidx++){
    line(rowidx*blkWidth, 0, rowidx*blkWidth, map.mapSize.y);
  }
  for (int colidx = 0; colidx <= (int)map.mapSize.y/blkWidth; colidx++){
    line(0, colidx*blkWidth,  map.mapSize.x, colidx*blkWidth);
  }
  
  mainLoop();
}

void mainLoop(){
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
      test.resetFitness();
      time = 0;
    } else {
      test.update();
      if (progressingFitness){
        test.calculateFitness();
      }
      test.show();
      time++;
      MutMoveRate = baseMutMoveRate;
      //println("time: " + time);
    }
  }
}

void updateFitnessTable(int WpSeq){
  boolean isDistFound;
  AstarFitness = new double[mapH][mapW];
  for (int sampx = 0; sampx < mapH; sampx++){
    for (int sampy = 0; sampy < mapW; sampy++){
      open = new ArrayList<Grid>();
      closed = new ArrayList<Grid>();
       
      startGrid = new int[] {sampx, sampy};
      goalGrid = map.Wps[WpSeq].wpToGrid(map.mapSize);
      
      grids = new Grid[mapH][mapW];
      for (int i = 0; i < grids.length; i++) {
        for (int j = 0; j < grids[i].length; j++) {
          grids[i][j] = new Grid (i, j, gridObsTable);
          grids[i][j].heuristic();
        }
      }
      
      closed.add(grids[startGrid[0]][startGrid[1]]);
      grids[startGrid[0]][startGrid[1]].link();
      isDistFound = false;
      while (!isDistFound){
        Grid lowest = getMin();
        
        if (lowest == null) {
          AstarFitness[sampx][sampy] = 0.0;
          break;
        }
        closed.add(lowest);
        open.remove(lowest);
        lowest.link();
        
        for (int i = 0; i < open.size(); i++) {
          if (open.get(i) == grids[goalGrid[0]][goalGrid[1]]) {
            isDistFound = true;
            AstarFitness[sampx][sampy] = open.get(i).f;
          }
        }
      }
    }
  }
}

void updateObstacleTable(){
  gridObsTable = new int[mapW][mapH];
  for (int x = 0; x < floor(map.mapSize.x/blkWidth); x++){
    for (int y = 0; y < floor(map.mapSize.y/blkWidth); y++){
      for (int intobs = 0; intobs < map.Obss.length; intobs++){
        if ((x+0.5)*blkWidth >= map.Obss[intobs].pos.x && (x+0.5)*blkWidth < map.Obss[intobs].pos.x +  map.Obss[intobs].size.x &&
        (y+0.5)*blkWidth >= map.Obss[intobs].pos.y && (y+0.5)*blkWidth < map.Obss[intobs].pos.y +  map.Obss[intobs].size.y){
          gridObsTable[x][y] = 1;
        }
      }
    }
  }
}

float[] cmdDecipher(String s){
  float[] returnCmd = new float[3];
  switch(s){
    case "R":
      break;
    case "L":
      break;
    case "B":
      break;
    case "F":
      break;
    case "S":
      break;
  }
  return returnCmd;
}
