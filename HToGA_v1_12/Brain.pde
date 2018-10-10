class Brain {
  
  Command[] Cmds;
  PVector[] pastPos; 
  int bestTime = 0;
  float bestFitness = 0;
  
  Brain(int size) {
    Cmds = new Command[size];
    pastPos = new PVector[size];
    randomizeCmds(); 
  }

  //--------------------------------------------------------------------------------------------------------------------------------
  //sets all the vectors in directions to a random vector with length 1
  void randomizeCmds() {
    for (int idxcmd = 0; idxcmd < Cmds.length; idxcmd++) {
      float randomNum = random(1);
      if (randomNum < 1/moveTransRatio && mutTransInitialze){
        int randMorph = floor(random(7));
        Cmds[idxcmd] = new Command(new PVector(0, 0), randMorph);
      } else {
        if (!gridBasedMode){
          float randomAngle = random(2*PI);
          Cmds[idxcmd] = new Command(PVector.fromAngle(randomAngle), -1);
        }else{
          float randomAngle = ((int)random(4))*PI/2;
          Cmds[idxcmd] = new Command(PVector.fromAngle(randomAngle).mult(blkWidth), -1);
        }
      }
    }
  }



  //-------------------------------------------------------------------------------------------------------------------------------------
  //returns a perfect copy of this brain object
  Brain clone() {
    Brain clone = new Brain(Cmds.length);
    for (int i = 0; i < Cmds.length; i++) {
      clone.Cmds[i] = new Command(Cmds[i].moveDir, Cmds[i].transMorph);
    }
    return clone;
  } 
}
