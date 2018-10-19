class Brain {
  
  String[] Cmds;
  PVector[] pastPos; 
  int curTime = 0; 
  
  Brain(int size) {
    Cmds = new String[size];
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
          int randInt = (int)random(4);
          PVector[] randDirArray = new PVector[]{new PVector(0,blkWidth), new PVector(0,-blkWidth), new PVector(blkWidth,0), new PVector(-blkWidth,0)};
          PVector randDir = randDirArray[randInt];
          if(noRepeatingGrids){
            PVector sumDir = randDir.copy();
            boolean validDir = false;
            while (!validDir){
              validDir = true;
              if (idxcmd < 7){
                validDir = true;
              }else{
                for (int grididx = 1; grididx <= 7; grididx += 2){
                  sumDir.add(Cmds[idxcmd - grididx].moveDir);
                  if (sumDir.x == 0 && sumDir.y == 0){
                    validDir = false;
                    randInt = (int)random(4);
                    randDir = randDirArray[randInt];
                    sumDir = randDir.copy();
                    break;
                  }
                }
              }
            }
            Cmds[idxcmd] = new Command(randDir, -1);
            
            PVector testSum = new PVector(0,0);
            for (int nidx = 0; nidx < idxcmd; nidx++){
              testSum.add(Cmds[nidx].moveDir);
            }
          } else{
            Cmds[idxcmd] = new Command(randDir, -1);
          }
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
