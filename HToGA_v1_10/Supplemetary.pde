class Morphology{
  
  float[][] RelAng;
  
  Morphology(){
    RelAng = new float[][]{{0, 0, 0, 0}, 
                             {0, 0, -PI, -PI},
                             {PI, 0, -PI/2.0, PI/2.0},
                             {PI, 0, 0, 0},
                             {PI, 0, 0, PI},
                             {PI/2.0, 0, -PI, 0},
                             {PI, 0, -PI/2.0, -PI/2.0}};
  }
                             
}

class Waypoint{
  PVector pos;
  int morph;
}

class Command{

  PVector moveDir = new PVector(0, 0);
  int transMorph = -1;
  
  Command(PVector moveDir_, int transMorph_){
    moveDir = moveDir_;
    transMorph = transMorph_;
  }
}
