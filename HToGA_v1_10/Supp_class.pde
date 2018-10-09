class Waypoint{
  PVector pos;
  int morph;
  
  Waypoint(PVector pos_, int morph_){
    pos = pos_;
    morph = morph_;
  }
  
}

class Map{
  
  boolean gridBasedMode;
  Waypoint[] Wps;
  Obstacle[] Obss;
  
  Map(boolean gridBasedMode_, Waypoint[] Wps_, Obstacle[] Obss_){
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
