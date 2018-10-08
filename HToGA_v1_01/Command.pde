class Command{

  PVector moveDir = new PVector(0, 0);
  int transMorph = -1;
  
  Command(PVector moveDir_, int transMorph_){
    moveDir = moveDir_;
    transMorph = transMorph_;
  }
}
