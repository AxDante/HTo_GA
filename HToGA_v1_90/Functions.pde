
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

boolean isCollideGrid(Robot rbt) {
  int[] blkGrid;
  for (int blkidx = 0; blkidx < rbt.Blks.length; blkidx++){
    blkGrid = rbt.Blks[blkidx].blkGridPos(map.mapSize);
    if (gridObsTable[blkGrid[0]][blkGrid[1]] == 1){
      return true;
    };
  }
  return false;
}

int PortionSelect(float[] arr){
  
  float sums = 0;
  for (int arridx = 0; arridx < arr.length; arridx++){
    sums += arr[arridx];
  }
  float rand = random(sums);
  float runningSum = 0;
  for (int arridx = 0; arridx < arr.length; arridx++) {
    runningSum += arr[arridx];
    if (runningSum > rand) {
      return arridx;
    }
  }
  return -1;
}
