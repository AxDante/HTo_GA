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
        //Cmds[idxcmd] = new Command(new PVector(0, 0), randMorph);
      } else {
        int randInt = (int)random(4);
        String randDirStr = fourDirString[randInt];
        PVector randDir = fourDirArray[randInt];
        if(noRepeatingGrids){
          PVector sumDir = randDir.copy();
          boolean validDir = false;
          while (!validDir){ 
            validDir = true;
            if (idxcmd < 7){
              validDir = true;
            }else{
              for (int grididx = 1; grididx <= 7; grididx += 2){
                if (sumDir.x == 0 && sumDir.y == 0){
                  validDir = false;
                  randInt = (int)random(4);
                  randDir = fourDirArray[randInt];
                  sumDir = randDir.copy();
                  break;
                }
              }
            }
          }
          Cmds[idxcmd] = fourDirString[randInt];
        } else{
          Cmds[idxcmd] = fourDirString[randInt];
        }
      }
    }
  }



  //-------------------------------------------------------------------------------------------------------------------------------------
  //returns a perfect copy of this brain object
  Brain clone() {
    Brain clone = new Brain(Cmds.length);
    for (int i = 0; i < Cmds.length; i++) {
      clone.Cmds[i] = Cmds[i];
    }
    return clone;
  } 
}
