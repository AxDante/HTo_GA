Population test;
MapDB mapDB;
Map map; 

int mapID = 10;
int maxTime = 100;

// Begin of Adjustable Booleans 
// -----------------------------------------------
boolean forceRemove = true;            // Force remove unreasonalbe sequences
boolean debugMode = false;             // For debug text display
boolean dispText = false;              // Toggle text display on Processing

boolean mutTransInitialze = false;     // Toggle transformation mutate during initialzation phase
boolean mutTransProcess = true;        // Toggle transformation mutate dueing navigation phase

boolean progressingFitness = true;     // Calculate fitness at each time instance if set to true
boolean noRepeatingGrids = true;       // Prevent robot from revisiting previous grid

boolean snapShift = true;              // Instantly snap to the correct angle during rotation/reconfiguration
boolean colorfulRobot = true;          // A COLORFUL ELITE ROBOT!!!
// End of Adjustable Booleans
//*************************************************
//*************************************************


// Begin of Adjustable Variables 
// -----------------------------------------------
float rotAngVel = PI/2.0;              // Rotation angular velocity
float rotThreshold = PI/22.0;          // Rotation threshold angle for the robot to stop 

float baseMutMoveRate = 0.03;          //0.05; // Mutation rate of robot moving direction
float baseMutRemoveDirRate = 0.002;
float baseMutRemoveShapeRate = 0.3;
float MutMoveRate;
float baseMutTransRate = 0.02;         //0.005; // Mutation rate of robt transformation
float baseCrossoverRate = 0.0;
float moveTransRatio = 100.0;

float bestPercentage = 0.1;

int frameRefreshRate = 1000;           // Processing simulation frame refresh rate (default:1000)
int totPopulation = 100;               // Total robot population size (default:100)
float blkWidth = 25;                   // Robot block size (default:25)

// Robot Perception Setup
boolean robotPerception = true;
int rbtPcepPattern = 1;
int rbtSearchDist = 2;
int rptGridTraceMax = 7;               // Maximum number of grids to trace back to prevent grid repetition
float Wobs = 0;
float WgeneDir = 300;
// End of Adjustable Variables
//*************************************************
//*************************************************


// Begin of Fixed Variables (Do not make changes)
// -----------------------------------------------

int currentWpID = 0;                  // Current Waypoint ID
int time = 0;                         // Current Time

int mapW, mapH;                       // waypoint map grid width (mapW) and height (mapW) 
float diag = (float)Math.sqrt(2);     // diag = 1.4142...
int morphNum;                         // Total number of morphologies 

Grid[][] grids;                      
int[] startGrid;                      // Starting grid for GA
int[] goalGrid;                       // Goal grid for GA
ArrayList<Grid> open = new ArrayList<Grid>();      // Open array for A star search
ArrayList<Grid> closed = new ArrayList<Grid>();    // Closed array for A star search
double[][] AstarFitness;              // A star fitness matrix for the map
ArrayList<double[][]> AstarFitnessList; // Stores several fitness matrices for the map
int[][] gridObs;                      // Grid obstacle matrix
ArrayList<int[][]> gridObsList;       // Stores several grid obstacle matrices for the map
//int dyTime;                           // dynamic time for Astar fitness table loading
int dyTimeMax = 10;                        // Maximum dynamic time

                      
PFont dispFont;                       // Grid Font display 
 
ArrayList<int[]> fourDirGridArray = new ArrayList<int[]>();  // Grid movement array for four directions : (0,1), (0, -1), (1,0), (-1,0)

PVector[] fourDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth,  0), new PVector(0, blkWidth), new PVector(-blkWidth,0)};
PVector[] eightDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth, -blkWidth), new PVector(blkWidth, 0), new PVector(blkWidth, blkWidth), 
                                         new PVector(0, blkWidth), new PVector(-blkWidth, blkWidth), new PVector(-blkWidth,0), new PVector(-blkWidth, -blkWidth)};
String[] fourDirString = new String[]{"F", "R", "B", "L"};
String[] eightDirString = new String[]{"F", "FR", "R", "BR", "B", "BL", "L", "FL"};

Table popTable;                       // Population table for data logging
Table astarTable;                     // Astar distance table for data logging
Table gridObsTable;                   // Grid obstacle table for data logging

boolean terminate = false;            // checking whether loop meets terminatation criteria

void settings() {
  
  fourDirGridArray.add(new int[]{0, -1});
  fourDirGridArray.add(new int[]{1, 0});
  fourDirGridArray.add(new int[]{0, 1});
  fourDirGridArray.add(new int[]{-1, 0});
  
  mapDB = new MapDB();
  map = mapDB.Maps[mapID];
  morphNum = new Morphology().RelAng.length+1;
  
  // Size the window first before setup and display
  size((int)map.mapSize.x, (int)map.mapSize.y); 
  
}

void setup(){
  /*
  ArrayList<double[][]> kkk = new ArrayList<double[][]>();
  double[][] ss = new double[1][2];
  ss[0][0] = 10;
  ss[0][1] = 100;
  kkk.add(ss);
  double[][] ss2 = new double[1][2];
  ss2[0][0] = 20;
  ss2[0][1] = 200;
  kkk.add(ss2);
  */
 
  // Preperation of data logging
  popTable = new Table();
  popTable.addColumn("genID");
  popTable.addColumn("bestTime");
  popTable.addColumn("reachGoal");
  popTable.addColumn("bestFitness");
  popTable.addColumn("bestGene");
  popTable.addColumn("bestID");
  popTable.addColumn("ID 0 Fitness");
  
  // Display font
  dispFont = createFont("Arial",8,true); 
  
  // Load starting population
  test = new Population(totPopulation, maxTime , map.Obss);
  frameRate(frameRefreshRate);
  
  // Calculate map grid width and height based on map size and block width
  mapW = floor(map.mapSize.x/blkWidth);
  mapH = floor(map.mapSize.y/blkWidth);
  
  // Data logging for A star table and grid obstacle table
  astarTable = new Table();
  gridObsTable = new Table();
  for (int sampy = 0; sampy < mapW; sampy++){
    astarTable.addColumn(str(sampy));
    gridObsTable.addColumn(str(sampy));
  }
  updateFitnessList(1);
  saveTable(gridObsTable, "data/gridObsTable.csv");
  saveTable(astarTable, "data/astarTable.csv");
  
  
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
  
  
  // Draw obstacle(s)
  fill(0, 0, 255);
  for (int intobs = 0; intobs < map.Obss.length; intobs++){
    rect(map.Obss[intobs].pos.x, map.Obss[intobs].pos.y, map.Obss[intobs].size.x, map.Obss[intobs].size.y);
  }
  for (int intDyobs = 0; intDyobs < map.DyObss.length; intDyobs++){
    rect(map.DyObss[intDyobs].pos.x, map.DyObss[intDyobs].pos.y, map.DyObss[intDyobs].size.x, map.DyObss[intDyobs].size.y);
  }
 
 
  // Draw grids
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
      
      time = 0;
      test.calculateFitness();      // Calculate Population Fitness
      boolean overTime = test.naturalSelection();      // Perform Natural Selection
      
      if (overTime){
        println("GA maximum time used. Waypoint navigation process terminating...");
        saveTable(popTable, "data/GAresult.csv");
        terminate = true;
        
      }else{
        test.GAMutation();            // Perform GA Mutation
        test.GACrossover();           // Perform GA Crossover
        test.GARemoveTwoDir();        // Remove two opposite directions in a gene
        test.GARemoveExtraShapes();   // Randomly remove shapes
        test.clearReachHistory();
        
        if (dispText){
          println("==================================");
          println("Navigation " + test.gen + " result:" );
        }
        if (test.isConverged()){
          if (currentWpID < map.Wps.length-2){
            currentWpID += 1;
            println("GA converged! Now navigating from wp " + currentWpID + " to wp " + str(currentWpID+1)); 
            updateFitnessList(currentWpID+1);  
            test = new Population(totPopulation, maxTime, map.Obss);
          }else{
            println("GA converged! Waypoint navigation process terminates.");
            saveTable(popTable, "data/GAresult.csv");
            terminate = true;
          }
        }
        test.resetFitness();
      }
      
    } else {
      test.update();
      if (progressingFitness){
        test.calculateFitness();
      }
      test.show();
      time++;
      MutMoveRate = baseMutMoveRate;
      updateDynamicObstaclePos();
    }
  }
}
