class Robot {
  
  PVector pos;
  PVector vel;
  PVector acc;
  Brain brain;

  boolean dead = false;
  boolean reachedGoal = false;
  boolean isBest = false;
  float fitness = 0;

  Block[] Blks;
  int morph = 2;
  Morphology Morph;
  
  Robot() {
    
    Morph = new Morphology();
    brain = new Brain(1000);//new brain with 1000 instructions

    //start the dots at the bottom of the window with a no velocity or acceleration
    pos = new PVector(map.Wps[currentWpID].pos.x, map.Wps[currentWpID].pos.y);
    vel = new PVector(0, 0);
    acc = new PVector(0, 0);
    
    
    Blks = new Block[4];
    for(int blkidx = 0; blkidx < 4; blkidx++){
      //Blks[blkidx] = new Block(blkidx, pos, 0);
    }
    Blks[0] = new Block(0, blkWidth , pos.copy().add(new PVector(blkWidth,0)), 0);
    Blks[1] = new Block(1, blkWidth , pos.copy().add(new PVector(0,0)), 0);
    Blks[2] = new Block(2, blkWidth , pos.copy().add(new PVector(-blkWidth,0)), 0);
    Blks[3] = new Block(3, blkWidth , pos.copy().add(new PVector(-blkWidth*2,0)), 0);
    
    updateBlockDesHeading();
  }
  
  //-----------------------------------------------------------------------------------------------------------------
  // update desired heading to each robot blocks
  void updateBlockDesHeading(){
    int newMorph = brain.Cmds[time].transMorph;
    if (newMorph != -1 && newMorph != morph){
      morph = newMorph;
    }
    
    int modMorph = morph % 7;
    int divMorph = floor((morph-1)/7);
    for(int blkidx = 0; blkidx < 4; blkidx++){
      float newHeading = Morph.RelAng[modMorph][blkidx] - divMorph * PI/2;
      // Set desired heading of each block according to the array values
      Blks[blkidx].desHeading = newHeading;
    }
  }

  //-----------------------------------------------------------------------------------------------------------------
  // perform robot shape-shifting
  boolean shapeShift(){
    
    if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
      // Block 2 rotates with respect to bottom-right corner of block 1
      float rotateAng = (Blks[0].desHeading > Blks[0].heading)? rotAngVel : -rotAngVel;
      Rotate(0,rotateAng, Blks[1].getCorner(3));
      return true;
    }
    else if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
     // Blks[1].Rotate(Blks[1].desHeading-Blks[1].heading, Blks[1].getCorner(3));
     return true;
    }
    else if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
      // Block 2 rotates with respect to upper-right corner of block 1
      float rotateAng = (Blks[2].desHeading > Blks[2].heading)? rotAngVel : -rotAngVel;
      Rotate(2,rotateAng, Blks[1].getCorner(0));
      return true;
    }
    else if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
      // Block 3 rotates with respect to upper-left corner of block 2
      float rotateAng = (Blks[3].desHeading > Blks[3].heading)? rotAngVel : -rotAngVel;
      Rotate(3,rotateAng, Blks[2].getCorner(1));
      return true;
    }
    return false;
  }
  
  void Rotate(int blkId, float angle, PVector posCtr){
    Blks[blkId].heading = Blks[blkId].heading + angle;
    PVector posToCenter = Blks[blkId].pos.copy().sub(posCtr);
    PVector posShift = posToCenter.copy().rotate(angle).sub(posToCenter);
    Blks[blkId].pos.add(posShift);
    
    if (blkId == 2){
      Rotate(3, angle, posCtr);
    }
    if(debugMode){
      println("Block " + blkId + " Rotation:");
      println("posCtr (x,y) = (" + posCtr.x + ", " + posCtr.y +").");
      println("posToCenter (x,y) = (" + posToCenter.x + ", " + posToCenter.y +").");
      println("posToCenter (x,y).rotate(angle) = (" + posToCenter.rotate(angle).x + ", " + posToCenter.rotate(angle).y +").");
      println("posToCenter (x,y).rotate(angle).sub(posToCenter) = (" + posToCenter.rotate(angle).sub(posToCenter).x + ", " + posToCenter.rotate(angle).sub(posToCenter).y +").");
    }
  }
  

  //-----------------------------------------------------------------------------------------------------------------
  //draws the dot on the screen
  void show() {
    if (isBest) {
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 255, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
      }
    } else {//all other dots are just smaller black dots
      fill(0);
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 0, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
      }
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------
  //moves the dot according to the brains directions
  
  void move() {
    updateBlockDesHeading();
    if (!shapeShift()){
      if (!gridBasedMode){
        acc = brain.Cmds[time].moveDir;
        vel.add(acc);
        vel.limit(5);
        pos.add(vel);
      }else if (gridBasedMode){
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
        }
      }
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        Blks[blkidx].pos.add(vel);
      }
    }
    
  }

  
  //--------------------------------------------------------------------------------------------------------------------------------------
  // mutate
  void mutate(){
    for (int i = 0; i < brain.Cmds.length; i++) {
      float rand = random(1);
      if (rand < baseMutTransRate && mutTransProcess){
        int randMorph = floor(random(7));
        morph = randMorph;
        brain.Cmds[i].transMorph = randMorph;
      }else if (rand < baseMutMoveRate) {
        if (!gridBasedMode){
          float randomAngle = random(2*PI);
          brain.Cmds[i].moveDir = PVector.fromAngle(randomAngle);
        }else{
          float randomAngle = ((int)random(4))*PI/2;
          brain.Cmds[i].moveDir = PVector.fromAngle(randomAngle).mult(blkWidth);
        }
      }
    }
  }



  //--------------------------------------------------------------------------------------------------------------------------------------
  //calculates the fitness
  void calculateFitness() {
    if (reachedGoal) {
      brain.bestTime = time;
      fitness = 1.0/16.0 + 10000.0/(float)(time * time);
      brain.bestFitness = fitness;
    } else {
      float distanceToGoal = dist(Blks[1].pos.x, Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y);
      fitness = 1.0/(distanceToGoal * distanceToGoal);
      brain.bestFitness = fitness;
    }
  }

  //---------------------------------------------------------------------------------------------------------------------------------------
  //clone it 
  Robot gimmeBaby() {
    Robot baby = new Robot();
    baby.brain = brain.clone();//babies have the same brain as their parents
    return baby;
  }
}
