 class Block{
  
  int id;
  PVector pos;
  float heading;
  float desHeading;
  float blkWidth;
  
  Block(int id_, float blkWidth_, PVector pos_, float heading_){
    id = id_;
    pos = pos_;
    heading = heading_;
    desHeading = heading_;
    blkWidth = blkWidth_;
    
    if (debugMode){
      println("Block " + id + " created at position (x,y) = (" + pos.x + ", " + pos.y + ").");
    }
  }
  
  
  int[] blkGridPos(PVector mapSize){
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
  
  
  void Move(PVector vel){
    pos.add(vel);
  }
  
  PVector getCorner(int cornerId){
    PVector[] retCorner = new PVector[4];
    // Upper-right corner
    retCorner[0] = pos.copy().add(new PVector(-blkWidth * sqrt(2) / 2.0, 0).rotate(-PI/4+heading));
    // Upper-left corner
    retCorner[1] = pos.copy().add(new PVector(-blkWidth * sqrt(2) / 2.0, 0).rotate(PI/4+heading));
    // Bottom-left corner
    retCorner[2] = pos.copy().add(new PVector(-blkWidth * sqrt(2) / 2.0, 0).rotate(3*PI/4+heading));
    // Bottom-right corner
    retCorner[3] = pos.copy().add(new PVector(-blkWidth * sqrt(2) / 2.0, 0).rotate(-3*PI/4+heading));
    return retCorner[cornerId];
  }
}
