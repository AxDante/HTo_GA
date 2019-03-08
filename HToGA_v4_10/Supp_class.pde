class Waypoint{
  PVector pos;
  int morph;
  
  Waypoint(PVector pos_, int morph_){
    pos = pos_;
    morph = morph_;
  }
  
  int[] wpToGrid(PVector mapSize){
    int[] gridPos = new int[2];
    gridPos[0] = floor(pos.x/blkWidth);
    gridPos[1] = floor(pos.y/blkWidth);
    if (gridPos[0] >= floor(mapSize.x/blkWidth)){
      gridPos[0] = floor(mapSize.x/blkWidth) -1;
    }
    if (gridPos[1] >= floor(mapSize.y/blkWidth)){
      gridPos[1] = floor(mapSize.y/blkWidth) -1;
    }
    return gridPos;
  }
  
}

class Map{
  
  boolean gridBasedMode;
  Waypoint[] Wps;
  Obstacle[] Obss;
  DynamicObstacle[] DyObss;
  ArrayList<DynamicObstacle> DyObssList;
  
  PVector mapSize;
  
  Map(PVector mapSize_, boolean gridBasedMode_, Waypoint[] Wps_, Obstacle[] Obss_, ArrayList<DynamicObstacle> DyObssList_){
    mapSize = mapSize_;
    gridBasedMode = gridBasedMode_;
    Wps = Wps_;
    Obss = Obss_;
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


class sortByCost implements Comparator<Robot> 
{ 
    public int compare(Robot r1, Robot r2) 
    { 
        return Float.compare(r1.costScore,r2.costScore); 
    } 
} 
class sortBySmooth implements Comparator<Robot> 
{ 
    public int compare(Robot r1, Robot r2) 
    { 
        return Float.compare(r1.smoothScore, r2.smoothScore); 
    } 
} 
class sortBySafe implements Comparator<Robot> 
{ 
    public int compare(Robot r1, Robot r2) 
    { 
        return Float.compare(r1.safeScore, r2.safeScore); 
    } 
} 


/*
class CrowdingDistance{
  float costMax;
  float costMin;
  float smoothMax;
  float smoothMin;
  float safeMax;
  float safeMin;
  
  CrowdingDistance(){
    costMax = 99999;
    costMin = 0;
    smoothMax = 99999;
    smoothMin = 0;
    safeMax = 99999;
    safeMin = 0;
  }
}
*/
/*
class GARobot{
  int ID;
  float costScore;
  float smoothScore;
  float safeScore;
  
  GARobot(int ID_, float costScore_, float smoothScore_, float safeScore_){
    ID = ID_;
    costScore = costScore_;
    smoothScore = smoothScore_;
    safeScore = safeScore_;

  }
}
*/
