class Waypoint{
  PVector pos;
  int morph;
  
  Waypoint(PVector pos_, int morph_){
    pos = pos_;
    morph = morph_;
  }
  
  int[] wpToGrid(PVector mapSize){
    int[] gridPos = new int[2];
    gridPos[0] = floor(pos.x/blkW);
    gridPos[1] = floor(pos.y/blkW);
    if (gridPos[0] >= floor(mapSize.x/blkW)){
      gridPos[0] = floor(mapSize.x/blkW) -1;
    }
    if (gridPos[1] >= floor(mapSize.y/blkW)){
      gridPos[1] = floor(mapSize.y/blkW) -1;
    }
    return gridPos;
  }
  
}

class Map{
  
  boolean gridBasedMode;
  ArrayList<DynamicObstacle> DyObssList;
  ArrayList<Obstacle> ObsList;
  ArrayList<Waypoint> WpList;
  
  PVector mapSize;
  
  Map(PVector mapSize_, boolean gridBasedMode_, ArrayList<Waypoint> WpList_, ArrayList<Obstacle> ObsList_, ArrayList<DynamicObstacle> DyObssList_){
    mapSize = mapSize_;
    gridBasedMode = gridBasedMode_;
    WpList = WpList_;
    ObsList = ObsList_;
    DyObssList = DyObssList_;
    //DyObssList = (ArrayList<DynamicObstacle>) DyObssList_.clone();
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

class DynamicObstacle{
  
  PVector startPos;
  PVector pos;
  PVector size;
  float cornerAng;
  int dirX;
  int dirY;
  int speed;
  int patrolTime;
  
  DynamicObstacle(PVector startPos_, PVector size_, int dirX_, int dirY_, int speed_, int patrolTime_){
    startPos = startPos_;
    size = size_;
    cornerAng = atan((float)size.x/size.y);
    dirX = dirX_;
    dirY = dirY_;
    speed = speed_;
    patrolTime = patrolTime_;
    pos = startPos.copy();
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


class sortByCost implements Comparator<Robot> { 
  public int compare(Robot r1, Robot r2) { 
    return Float.compare(r1.costScore,r2.costScore); 
  } 
} 
class sortBySmooth implements Comparator<Robot> { 
  public int compare(Robot r1, Robot r2) { 
    return Float.compare(r1.smoothScore, r2.smoothScore); 
  } 
} 
class sortBySafe implements Comparator<Robot> { 
  public int compare(Robot r1, Robot r2) 
  { 
    return Float.compare(r1.safeScore, r2.safeScore); 
  } 
} 
