class Block{
  
  int id;
  PVector pos;
  float heading;
  
  Block(int id_, PVector pos_, float heading_){
    id = id_;
    pos = pos_;
    heading = heading_;
  }
  
  void Rotate(float angle, PVector posCtr){
    heading = heading + angle;
    if (heading >= 2*PI){
      heading = 0;
    }
    PVector posToCenter = pos.sub(posCtr);
    pos = pos.add(posToCenter.rotate(angle).sub(posToCenter));
  }
  
  void Move(PVector vel){
    pos.add(vel);
  }
}
