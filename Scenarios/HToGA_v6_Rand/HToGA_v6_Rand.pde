import java.util.Collections;
import java.util.*; 

Population test;
MapDB mapDB;
Map map; 

int mapID = 16;
int maxTime = 100;
int maxStep = 5000;

// Begin of Adjustable Booleans 
// -----------------------------------------------

boolean dispSim = false;               // Toggle simulation screen display on Processing
boolean dispText = false;              // Toggle text display on Processing
boolean dispPopResult = false;
boolean debugMode = false;             // For debug text display
boolean multiTrials = true;            // Multiple trials for GA algorithm

boolean mutTransInitialze = false;     // Toggle transformation mutate during initialzation phase
boolean mutTransProcess = true;        // Toggle transformation mutate during navigation phase

boolean forceRemove = true;            // Force remove unreasonalbe sequences
boolean progressingFitness = true;     // Calculate fitness at each time instance if set to true
boolean noRepeatingGrids = true;       // Prevent robot from revisiting previous grid

boolean singleAstarFitnessList = true;
boolean snapShift = true;              // Instantly snap to the correct angle during rotation/reconfiguration
boolean colorfulRobot = true;          // A COLORFUL ELITE ROBOT!!!
boolean singleFitnessList = true;      // single or multiple astar fitness list table for dynamic obstacle 
// End of Adjustable Booleans
//*************************************************
//*************************************************


// Begin of Adjustable Variables 
// -----------------------------------------------
float rotAngVel = PI/2.0;              // Rotation angular velocity
float rotThreshold = PI/22.0;          // Rotation threshold angle for the robot to stop 

float baseMutMoveRate = 0.1;          //0.05; // Mutation rate of robot moving direction
float baseMutRemoveDirRate = 0.4;
float baseMutRemoveShapeRate = 0.2;
float MutMoveRate;
float baseMutTransRate = 0.02;         //0.005; // Mutation rate of robt transformation
float baseCrossoverRate = 0.0;
float moveTransRatio = 100.0;  

float bestPercentage = 0.1;
int frameRefreshRate = 1000;           // Processing simulation frame refresh rate (default:1000)
int totPopulation = 25;               // Total robot population size (default:100)
int totTrials = 25;                    // Total number of trials (default: 50)
float blkWidth = 25;                   // Robot block size (default:25)

// Robot Perception Setup
boolean robotPerception = true;
int rbtPcepPattern = 1;
int rbtSearchDist = 2;
int rptGridTraceMax = 7;               // Maximum number of grids to trace back to prevent grid repetition
float Wobs = 0.2;
float WgeneDir = 10;

// End of Adjustable Variables
//*************************************************
//*************************************************


// Begin of Fixed Variables (Do not make changes)
// -----------------------------------------------

double startTime;
double reachTime;
double finalTime;
double[] trialTermTime;
double[] trialReachTime;
int[] trialReachGen;
int[] trialTermGen;
float[] trialC;
float[] trialT;
float[] trialSM;
float[] trialSF;

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
//double[][] AstarFitness;              // A star fitness matrix for the map
ArrayList<double[][]> AstarFitnessList; // Stores several fitness matrices for the map
int[][] gridObs;                      // Grid obstacle matrix
ArrayList<int[][]> gridObsList;       // Stores several grid obstacle matrices for the map
//int dyTime;                           // dynamic time for Astar fitness table loading
int dyTimeMax = 1;                     // Maximum dynamic time

                      
PFont dispFont;                       // Grid Font display 
 
ArrayList<int[]> fourDirGridArray = new ArrayList<int[]>();  // Grid movement array for four directions : (0,1), (0, -1), (1,0), (-1,0)

PVector[] fourDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth,  0), new PVector(0, blkWidth), new PVector(-blkWidth,0)};
PVector[] eightDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth, -blkWidth), new PVector(blkWidth, 0), new PVector(blkWidth, blkWidth), 
                                         new PVector(0, blkWidth), new PVector(-blkWidth, blkWidth), new PVector(-blkWidth,0), new PVector(-blkWidth, -blkWidth)};
                                         
PVector[] doubleDirArray = new PVector[]{new PVector(0,-blkWidth), new PVector(blkWidth,  0), new PVector(0, blkWidth), new PVector(-blkWidth,0),
                                         new PVector(0,-2*blkWidth), new PVector(2*blkWidth,  0), new PVector(0, 2*blkWidth), new PVector(-2*blkWidth,0),};

String[] fourDirString = new String[]{"F", "R", "B", "L"};
String[] eightDirString = new String[]{"F", "FR", "R", "BR", "B", "BL", "L", "FL"};

Table popTable;                       // Population table for data logging
Table astarTable;                     // Astar distance table for data logging
Table gridObsTable;                   // Grid obstacle table for data logging
Table fitnessTable;                   // Fitness table for data logging         
TableRow fitnessTableRow;             // Fitness tablerow for data logging
Table bestTimeTable;                  // Best time table for data logging  
TableRow bestTimeTableRow;            // Best time tablerow for data logging  

boolean terminateTrial = false;       // checking whether loop meets terminatation criteria
boolean trialComplete = false;    // checking whether loop meets terminatation criteria
int intTrials = 0;

void settings() {
  
  fourDirGridArray.add(new int[]{0, -1});
  fourDirGridArray.add(new int[]{1, 0});
  fourDirGridArray.add(new int[]{0, 1});
  fourDirGridArray.add(new int[]{-1, 0});
  
  mapDB = new MapDB();
  map = mapDB.Maps[mapID];
  morphNum = new Morphology().RelAng.length+1;
  
  // Size the window first before setup and display
  if (dispSim){
    size((int)map.mapSize.x, (int)map.mapSize.y); 
  }
      
}

void setup(){
  
  
  trialTermTime = new double[totTrials];
  trialReachTime = new double[totTrials];
  trialReachGen = new int[totTrials];
  trialTermGen = new int[totTrials];
  trialC = new float[totTrials];
  trialT = new float[totTrials];
  trialSM = new float[totTrials];
  trialSF = new float[totTrials];
  
  startTime = millis();
  popTableInitialize();
  
  // Display font
  dispFont = createFont("Arial",8,true); 
  frameRate(frameRefreshRate);
  
  // Calculate map grid width and height based on map size and block width
  mapW = floor(map.mapSize.x/blkWidth);
  mapH = floor(map.mapSize.y/blkWidth);
  
  // Load starting population
  
  test = new Population(totPopulation, map.Obss);
    
  // Data logging for Astar table, grid obstacle table, fitness table, and bestTime table
  astarTable = new Table();
  gridObsTable = new Table();
  fitnessTable = new Table();
  bestTimeTable = new Table();
  
  // Assign column indices for the tables 
  for (int sampy = 0; sampy < mapW; sampy++){
    astarTable.addColumn(str(sampy));
    gridObsTable.addColumn(str(sampy));
  }
  fitnessTable.addColumn("Trial No.");
  bestTimeTable.addColumn("Trial No.");
  for (int intTime = 0; intTime <= maxTime; intTime++){
    fitnessTable.addColumn(str(intTime));
    bestTimeTable.addColumn(str(intTime));
  }
  
  // Update Astar fitness
  updateFitnessList(1);
  
  // Save astar and grid obstacle tables
  saveTable(gridObsTable, "data/gridObsTable.csv");
  saveTable(astarTable, "data/astarTable.csv");
  
  // Trial 0 begins! start recording trial fitness and bestTime data
  fitnessTableRow = fitnessTable.addRow();
  fitnessTableRow.setInt("Trial No.", 0);
  bestTimeTableRow = bestTimeTable.addRow();
  bestTimeTableRow.setInt("Trial No.", 0);
  println("-- Trial 0.");
}

// Main draw function for Processing
void draw() { 
  
  // Toggle dispSim to show/hide display
  if (dispSim){   
    background(255);
    fill(0);
    
    // Draw grid text for debugging
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
    for (int intDyobs = 0; intDyobs < map.DyObssList.size(); intDyobs++){
      rect(map.DyObssList.get(intDyobs).pos.x, map.DyObssList.get(intDyobs).pos.y, map.DyObssList.get(intDyobs).size.x, map.DyObssList.get(intDyobs).size.y);
    }
   
    // Draw grids
    stroke(125);
    for (int rowidx = 0; rowidx <= (int)map.mapSize.x/blkWidth; rowidx++){
      line(rowidx*blkWidth, 0, rowidx*blkWidth, map.mapSize.y);
    }
    for (int colidx = 0; colidx <= (int)map.mapSize.y/blkWidth; colidx++){
      line(0, colidx*blkWidth,  map.mapSize.x, colidx*blkWidth);
    }
  }
  // Perform main Loop
  mainLoop();
}

void mainLoop(){
  
  trialComplete = false;
  
  if (intTrials < totTrials){
    terminateTrial = false;
    if (!terminateTrial){
      if (test.allRobotsDead()) {
        time = 0;
        test.calculateFinalFitness();      // Calculate Population Fitness
        test.nsga2();
        test.nsga2RecordData();
        boolean overTime = test.naturalSelection();      // Perform Natural Selection
        
        if (overTime){
          println("T"+ intTrials + " Wp" + currentWpID + " : GA maximum time used.");
          trialComplete = true;
        }else{
          test.GANonreached();
          test.GAMutation();            // Perform GA Mutation
          //test.GACrossover();           // Perform GA Crossover
          test.GARemoveTwoDir();        // Remove two opposite directions in a gene
          test.GARemoveExtraShapes();   // Randomly remove shapes
          test.GASwap();
          test.MergeRobots();
          test.clearReachHistory();
          
          if (dispText){
            println("==================================");
            println("Navigation " + test.gen + " result:" );
          }
          if (test.isConverged()){
            if (currentWpID < map.Wps.length-2){
              currentWpID += 1;
              if (dispText) println("T"+ intTrials + " Wp" + currentWpID + " : GA converged! Now navigating from wp " + currentWpID + " to wp " + str(currentWpID+1)); 
              updateFitnessList(currentWpID+1); 
              test = new Population(totPopulation, map.Obss);
            }else{
              if (dispText) println("T"+ intTrials + " Wp" + currentWpID + " : GA converged! Waypoint navigation process terminates.");
              trialComplete = true;
            }
          }
          test.resetFitness();
        }
      } else {
        test.update();
        if (progressingFitness){
          test.calculateFitness();
        }
        if (dispSim) test.show();
        time++;
        MutMoveRate = baseMutMoveRate;
        updateDynamicObstaclePos();
      }
    }
    
    if (trialComplete){
      // GA result logging
      saveTable(popTable, "data/GAresult" + intTrials + ".csv");
      terminateTrial = true;
      finalTime = millis();
      trialTermTime[intTrials] = finalTime - startTime;
      trialTermGen[intTrials] = test.gen;
      trialC[intTrials] = 1;
      trialT[intTrials] = 1-(float)test.unsortRobotListCopy.get(test.bestSumCDRbtID).brain.Cmds.size()/maxTime;
      trialSM[intTrials] = test.unsortRobotListCopy.get(test.bestSumCDRbtID).smoothScore*2-1;
      trialSF[intTrials] = test.unsortRobotListCopy.get(test.bestSumCDRbtID).safeScore;
      intTrials = intTrials+1;
      
      if (intTrials < totTrials){
        // Create new population if more trials remain
        startTime = millis();
        popTableInitialize();
        test = new Population(totPopulation, map.Obss);
        currentWpID = 0; 
        time = 0;
        updateFitnessList(1);
        println("-- Trial " + intTrials + ".");
        fitnessTableRow = fitnessTable.addRow();
        fitnessTableRow.setInt("Trial No.", intTrials);
        bestTimeTableRow = bestTimeTable.addRow();
        bestTimeTableRow.setInt("Trial No.", intTrials);
      }else{
        
        saveTable(fitnessTable, "data/GAFitnessResult.csv");
        saveTable(bestTimeTable, "data/GABestTimeResult.csv");
        
        println("=================================");
        println("All trials completed.");
        println("Population Size " + totPopulation);
        
        float sumRT = 0, sumTT = 0;
        float sumRG = 0, sumTG = 0, sumT = 0, sumSM = 0, sumSF = 0;
        
        float avgRT = 0, avgTT = 0;
        float avgRG = 0, avgTG = 0, avgT = 0, avgSM = 0, avgSF = 0;
        
        float sumDevRT = 0, sumDevTT = 0;
        float sumDevRG = 0, sumDevTG = 0, sumDevT = 0, sumDevSM = 0, sumDevSF = 0;
        
        float stdRT = 0, stdTT = 0;
        float stdRG = 0, stdTG = 0, stdT = 0, stdSM = 0, stdSF = 0;

        for(int inttr = 0; inttr < totTrials; inttr++){
          sumRG += (float)trialReachGen[inttr];
          sumTG += (float)trialTermGen[inttr];
          sumRT += (float)trialReachTime[inttr];
          sumTT += (float)trialTermTime[inttr];
          sumT += (float)trialT[inttr];
          sumSM += (float)trialSM[inttr];
          sumSF += (float)trialSF[inttr];
        }
        avgRG = sumRG/totTrials;
        avgTG = sumTG/totTrials;
        avgRT = sumRT/totTrials;
        avgTT = sumTT/totTrials;
        avgT =  sumT/totTrials;
        avgSM = sumSM/totTrials;
        avgSF = sumSF/totTrials;
        
        for(int inttr = 0; inttr < totTrials; inttr++){
          sumDevRG += pow((float)trialReachGen[inttr] - avgRG, 2);
          sumDevTG += pow((float)trialTermGen[inttr]- avgTG, 2);
          sumDevRT += pow((float)trialReachTime[inttr]- avgRT, 2);
          sumDevTT += pow((float)trialTermTime[inttr]- avgTT, 2);
          sumDevT += pow((float)trialT[inttr]- avgT, 2);
          sumDevSM += pow((float)trialSM[inttr]- avgSM, 2);
          sumDevSF += pow((float)trialSF[inttr]- avgSF, 2);
        }
        
        stdRG = pow(sumDevRG/totTrials,0.5);
        stdTG = pow(sumDevTG/totTrials,0.5);
        stdRT = pow(sumDevRT/totTrials,0.5);
        stdTT = pow(sumDevTT/totTrials,0.5);
        stdT = pow(sumDevT/totTrials,0.5);
        stdSM = pow(sumDevSM/totTrials,0.5);
        stdSF = pow(sumDevSF/totTrials,0.5);
        
        for(int inttr = 0; inttr < totTrials; inttr++){
          println();
          println("---- Trial "+ inttr + " results ----");
          println("RG: " + trialReachGen[inttr] + " | TG: " + trialTermGen[inttr] + " | RT: "+ trialReachTime[inttr] + " | TT: "+ trialTermTime[inttr]);
          println("T: " + trialT[inttr] + " | SM: " + trialSM[inttr] + " | SF: "+ trialSF[inttr] );
        }
        println("avgT: " + avgT + " | avgSM: " + avgSM + " | avgSF: "+ avgSF );
        println("avgRG: " + avgRG + " | avgTG: " + avgTG + " | avgRT: "+ avgRT + " | avgTT: "+ avgTT);
        println("stdT: " + stdT + " | stdSM: " + stdSM + " | stdSF: "+ stdSF );
        println("stdRG: " + stdRG + " | stdTG: " + stdTG + " | stdRT: "+ stdRT + " | stdTT: "+ stdTT);
      }
    }
  }
}
