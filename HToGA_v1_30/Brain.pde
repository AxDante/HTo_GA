class Brain {
  
  Command[] Cmds;
  PVector[] pastPos; 
  int curTime = 0; 
  
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
          int randInt = random(4);
          PVector[] randDirArray = new PVector[]{PVector(0,blkWidth), PVector(0,-blkWidth), PVector(blkWidth,0), PVector(-blkWidth,0)};
          PVector[] randDir = randDirArray[randInt];
        if (noRepeatingGrids){
         
          
          boolean reroll = false;
          float randomAngle = ((int)random(4))*PI/2;
          PVector nextPos = pos.copy().add(PVector.fromAngle(randomAngle).mult(blkWidth));
          if (time > 3){
            for (int grididx = 1; grididx <= 4; grididx++){
                //println("nextPosition (x, y) = (" + nextPos.x + ", " +nextPos.y + ").");
                //println("grididx " + grididx);
                //println("brain.step " + brain.step);
              if (nextPos.x == brain.pastPos[time - grididx].x && nextPos.y == brain.pastPos[time - grididx].y){
                reroll = true;
              }
            }
          }
          while (reroll){
            randomAngle = ((int)random(4))*PI/2;
            nextPos = Blks[1].pos.copy().add(PVector.fromAngle(randomAngle).mult(blkWidth));
            reroll = false;
            if (time > 3){
              for (int grididx = 1; grididx <= 4; grididx++){
                if (nextPos.x == brain.pastPos[time - grididx].x && nextPos.y == brain.pastPos[time - grididx].y){
                  reroll = true;
                }
              }
            }
          }
          vel = PVector.fromAngle(randomAngle).mult(blkWidth);
          pos = nextPos;
        }else{
          vel = brain.Cmds[time].moveDir;
          pos.add(vel);
        }*/
          
          
          
          
          
          
          
          
          
          
          
          
          
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
