 class Block{
  
  int id;
  PVector pos;
  float heading;
  float desHeading;
  float blkW;
  
  Block(int id_, float blkW_, PVector pos_, float heading_){
    id = id_;
    pos = pos_;
    heading = heading_;
    desHeading = heading_;
    blkW = blkW_;
    
    if (debugMode){
      println("Block " + id + " created at position (x,y) = (" + pos.x + ", " + pos.y + ").");
    }
  }
  
  // Return current grid position of the block
  int[] getBlkGridPos(){
    int[] blkGridPos = new int[2];
    blkGridPos[0] = floor(pos.x/blkW);
    blkGridPos[1] = floor(pos.y/blkW);
    if (blkGridPos[0] >= mapW || blkGridPos[0] < 0){
      blkGridPos[0] = -1;
    }
    if (blkGridPos[1] >= mapH || blkGridPos[1] < 0){
      blkGridPos[1] = -1;
    }
    return blkGridPos;
  }
  
  
  void Move(PVector vel){
    pos.add(vel);
  }
  
  PVector getCorner(int cornerId){
    PVector[] retCorner = new PVector[4];
    // Upper-right corner
    retCorner[0] = pos.copy().add(new PVector(-blkW * sqrt(2) / 2.0, 0).rotate(-PI/4+heading));
    // Upper-left corner
    retCorner[1] = pos.copy().add(new PVector(-blkW * sqrt(2) / 2.0, 0).rotate(PI/4+heading));
    // Bottom-left corner
    retCorner[2] = pos.copy().add(new PVector(-blkW * sqrt(2) / 2.0, 0).rotate(3*PI/4+heading));
    // Bottom-right corner
    retCorner[3] = pos.copy().add(new PVector(-blkW * sqrt(2) / 2.0, 0).rotate(-3*PI/4+heading));
    return retCorner[cornerId];
  }
}
