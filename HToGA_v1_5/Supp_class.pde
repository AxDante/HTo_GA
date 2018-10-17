class Waypoint{
  PVector pos;
  int morph;
  
  Waypoint(PVector pos_, int morph_){
    pos = pos_;
    morph = morph_;
  }
  
  int[] wpToGrid(PVector mapSize){
    int[] gridPos = new int[2];
    gridPos[0] = floor((pos.x % mapSize.x)/blkWidth);
    gridPos[1] = floor( pos.y / blkWidth );
    return gridPos;
  }
  
}

class Map{
  
  boolean gridBasedMode;
  Waypoint[] Wps;
  Obstacle[] Obss;
  PVector mapSize;
  
  Map(PVector mapSize_, boolean gridBasedMode_, Waypoint[] Wps_, Obstacle[] Obss_){
    mapSize = mapSize_;
    gridBasedMode = gridBasedMode_;
    Wps = Wps_;
    Obss = Obss_;
  }
}


class Command{

  PVector moveDir = new PVector(0, 0);
  int transMorph = -1;
  
  Command(PVector moveDir_, int transMorph_){
    moveDir = moveDir_;
    transMorph = transMorph_;
  }
}

class Obstacle{
  PVector pos;
  PVector size;
  float cornerAng;
  
  Obstacle(PVector pos_, PVector size_){
    pos = pos_;
    size = size_;
    cornerAng = atan((float)size.x/size.y);
  }
  
  PVector getCorner(int cornerId){
    PVector[] retCorner = new PVector[4];
    // Upper-right corner
    retCorner[0] = pos.copy().add(new PVector(-size.x / sin(cornerAng) / 2.0, 0).rotate(-cornerAng));
    // Upper-left corner
    retCorner[1] = pos.copy().add(new PVector(-size.x / sin(cornerAng) / 2.0, 0).rotate(cornerAng));
    // Bottom-left corner
    retCorner[2] = pos.copy().add(new PVector(-size.x / sin(cornerAng) / 2.0, 0).rotate(PI - cornerAng));
    // Bottom-right corner
    retCorner[3] = pos.copy().add(new PVector(-size.x / sin(cornerAng) / 2.0, 0).rotate(-PI + cornerAng));
    return retCorner[cornerId];
  }
  
}
