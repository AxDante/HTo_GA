void updateDynamicObstaclePos(){
  // update dynamic obstacle position
  for (int intDyObs = 0; intDyObs < map.DyObss.length; intDyObs++){
    
    int dyObsTime = time % (map.DyObss[intDyObs].patrolTime*2);
    if (dyObsTime == map.DyObss[intDyObs].patrolTime+1){
      map.DyObss[intDyObs].dirX *= -1;
      map.DyObss[intDyObs].dirY *= -1;
    }
    if (dyObsTime == 0){
      map.DyObss[intDyObs].pos = map.DyObss[intDyObs].startPos.copy();
    }
    else{
      map.DyObss[intDyObs].pos.add(new PVector(map.DyObss[intDyObs].dirX, map.DyObss[intDyObs].dirY).mult(map.DyObss[intDyObs].speed));
    }  
  }
}

float getGridFitnessValue(int posX, int posY, int scale){
  int dyObsTime = time % (dyTimeMax*2);
  //println("pos" + posX + " " + posY + " " + dyObsTime);
  return pow((float)AstarFitnessList.get(dyObsTime)[posX][posY], scale);
}

void updateFitnessList(int WpSeq){
  
  // Initialize list table
  AstarFitnessList = new ArrayList<double[][]>();
  gridObsList = new ArrayList<int[][]>();
  
  // Loop through different dynamic time
  for (int dyTimeCount = 0; dyTimeCount < 2*dyTimeMax; dyTimeCount++){
    // update grid obstacle informaton and store in list
    gridObs = new int[mapW][mapH];
    for (int x = 0; x < mapW; x++){
      TableRow newRow = gridObsTable.addRow();
      for (int y = 0; y < mapH; y++){
        for (int intObs = 0; intObs < map.Obss.length; intObs++){
          if ((x+0.5)*blkWidth >= map.Obss[intObs].pos.x && (x+0.5)*blkWidth < map.Obss[intObs].pos.x +  map.Obss[intObs].size.x &&
          (y+0.5)*blkWidth >= map.Obss[intObs].pos.y && (y+0.5)*blkWidth < map.Obss[intObs].pos.y +  map.Obss[intObs].size.y){
            gridObs[x][y] = 1;
          }
        }
        for (int intDyObs = 0; intDyObs < map.DyObss.length; intDyObs++){
          if ((x+0.5)*blkWidth >= map.DyObss[intDyObs].pos.x && (x+0.5)*blkWidth < map.DyObss[intDyObs].pos.x +  map.DyObss[intDyObs].size.x &&
          (y+0.5)*blkWidth >= map.DyObss[intDyObs].pos.y && (y+0.5)*blkWidth < map.DyObss[intDyObs].pos.y +  map.DyObss[intDyObs].size.y){
            gridObs[x][y] = 1;
          }
        }
        newRow.setInt(str(y), gridObs[x][y]);
      }
    }
    gridObsList.add(gridObs);
    
    // update grid astar score information and store in list
    double[][] AstarFitness = new double[mapW][mapH];
    boolean isDistFound;
    for (int sampx = 0; sampx < mapW; sampx++){
      TableRow newRow = astarTable.addRow();
      for (int sampy = 0; sampy < mapH; sampy++){
        
        open = new ArrayList<Grid>();
        closed = new ArrayList<Grid>();
         
        startGrid = new int[] {sampx, sampy};
        goalGrid = map.Wps[WpSeq].wpToGrid(map.mapSize);
        
        grids = new Grid[mapW][mapH];
        for (int i = 0; i < grids.length; i++) {
          for (int j = 0; j < grids[i].length; j++) {
            grids[i][j] = new Grid (i, j, gridObs);
            grids[i][j].heuristic();
          }
        }
        
        closed.add(grids[startGrid[0]][startGrid[1]]);
        grids[startGrid[0]][startGrid[1]].link();
        isDistFound = false;
        while (!isDistFound){
          Grid lowest = getMin();
          
          if (lowest == null) {
            AstarFitness[sampx][sampy] = 1.0;
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
          if (grids[sampx][sampy].isWall) {
            AstarFitness[sampx][sampy] = 1.0;
          }
        }
        AstarFitness[goalGrid[0]][goalGrid[1]] = 0.0;
        newRow.setFloat(str(sampy), (float)AstarFitness[sampx][sampy]); //AstarFitness[sampx][sampy]);
      }
    }
    AstarFitnessList.add(AstarFitness);
  }
}


float[] cmdDecipher(String inString){
  float[] returnCmd = new float[3];

  // Morphology Input
  int inMorph = int(inString);

  // Movement input
  if (inMorph == 0){
    for (int diridx = 0; diridx < fourDirString.length; diridx++){
      if (fourDirString[diridx] == inString){
        returnCmd[0] = fourDirArray[diridx].x;
        returnCmd[1] = fourDirArray[diridx].y;
        returnCmd[2] = -1;
      }
    }
  }
  return returnCmd;
}


boolean isCollideRbt(Robot rbt) {
  int[] blkGrid;
  for (int blkidx = 0; blkidx < rbt.Blks.length; blkidx++){
    blkGrid = rbt.Blks[blkidx].blkGridPos(map.mapSize);
    if (!isValidGrid(blkGrid)){
      return true;
    };
  }
  return false;
}

boolean isValidGrid(int[] blkGrid) {
  if (gridObs[blkGrid[0]][blkGrid[1]] == 1){
    return false;
  }else if (blkGrid[0] > mapW || blkGrid[1] > mapH || blkGrid[0] < 0 || blkGrid[1] < 0){
    return false;
  }else{
    return true;
  }
}

int PortionSelect(float[] arr){
  
  //println("IN(" + arr[0] + ","+ arr[1] + ","+ arr[2] + ","+ arr[3] + ")");
  float sums = 0;
  for (int arridx = 0; arridx < arr.length; arridx++){
    sums += arr[arridx];
  }
  if (sums == 0){
    return floor(random(4));
  }
  float rand = random(sums);
  float runningSum = 0;
  for (int arridx = 0; arridx < arr.length; arridx++) {
    runningSum += arr[arridx];
    if (runningSum > rand) {
      //println("OUT(" + arridx + ")");
      return arridx;
    }
  }
  return -1;
}

int[] getGridID(PVector pos){
  int[] gridPos = new int[2];
  gridPos[0] = floor(pos.x/blkWidth);
  gridPos[1] = floor(pos.y/blkWidth);
  if (gridPos[0] >= mapW){
    gridPos[0] = mapW -1;
  }
  if (gridPos[1] >= mapH){
    gridPos[1] = mapH -1;
  }
  if (gridPos[0] < 0 ){
    gridPos[0] = 0;
  }
  if (gridPos[1] < 0){
    gridPos[1] = 0;
  }
  return gridPos;
}

void popTableInitialize(){
  // Preperation of data logging
  popTable = new Table();
  popTable.addColumn("genID");
  popTable.addColumn("bestTime");
  popTable.addColumn("reachGoal");
  popTable.addColumn("bestFitness");
  popTable.addColumn("bestGene");
  popTable.addColumn("bestID");
  popTable.addColumn("ID 0 Fitness");
}
