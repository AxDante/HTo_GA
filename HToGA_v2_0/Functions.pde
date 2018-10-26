
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
  if (gridObsTable[blkGrid[0]][blkGrid[1]] == 1){
    return false;
  }else if (blkGrid[0] >= mapW || blkGrid[1] >= mapH || blkGrid[0] < 0 || blkGrid[1] < 0){
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
