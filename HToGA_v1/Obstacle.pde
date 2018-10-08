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
