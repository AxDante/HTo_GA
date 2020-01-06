class Brain {
  
  ArrayList<String> Cmds ;
  PVector[] pastPos; 
  
  int curTime = 0;
  
  float curCost = 0; 
  float curSafe = 0;
  float curSmooth = 0;
  
  Brain(int size) {
    Cmds = new ArrayList<String>();
    for (int cmdidx = 0; cmdidx < size; cmdidx++){
      Cmds.add("S");
    }
    pastPos = new PVector[size];
    randomizeCmds(); 
  }

  //--------------------------------------------------------------------------------------------------------------------------------
  //sets all the vectors in directions to a random vector with length 1
  void randomizeCmds() {
    
    // Start with different morphologies
    if (mutTransInitialze){ 
      for (int idxcmd = 0; idxcmd < Cmds.size(); idxcmd++) {
        float randomNum = random(1);
        if (randomNum < 1/moveTransRatio){
          int randMorph = floor(random(morphNum));
          Cmds.set(idxcmd,str(randMorph));
        }
      }
      return;
    }
      
      
    // Start with one morphology
    for (int idxcmd = 0; idxcmd < Cmds.size(); idxcmd++) {

      int randInt = (int)random(4);
      PVector randDir = fourDirArray[randInt];
      if(noRepeatingGrids){
        PVector sumDir = randDir.copy();
        boolean validDir = false;
        while (!validDir){ 
          validDir = true;
          if (idxcmd < rptGridTraceMax){
            validDir = true;
          }else{
            for (int grididx = 1; grididx <= rptGridTraceMax; grididx += 2){
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
        Cmds.set(idxcmd,fourDirString[randInt]);
      } else{
        Cmds.set(idxcmd,fourDirString[randInt]);
      }
    }
    
  }

  //returns a copy of brain 
  Brain clone() {
    Brain clone = new Brain(Cmds.size());
    for (int i = 0; i < Cmds.size(); i++) {
      clone.Cmds.set(i, Cmds.get(i));
    }
    return clone;
  } 
}
