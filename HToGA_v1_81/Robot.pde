class Robot {
  
  PVector pos;
  PVector vel;
  PVector acc;
  Brain brain;

  boolean dead = false;
  boolean reachedGoal = false;
  boolean isBest = false;
  boolean shapeShifting = false;
  
  float fitness = 0;

  Block[] Blks;
  int morph = 0;
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
    int newMorph = int(brain.Cmds[time]);
    println(newMorph);
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
  void shapeShift(){
    
    shapeShifting = false;
    if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
      // Block 2 rotates with respect to bottom-right corner of block 1
      float rotateAng = (Blks[0].desHeading > Blks[0].heading)? rotAngVel : -rotAngVel;
      Rotate(0,rotateAng, Blks[1].getCorner(3));
      shapeShifting = true;
    }
    else if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
     // Blks[1].Rotate(Blks[1].desHeading-Blks[1].heading, Blks[1].getCorner(3));
       shapeShifting = true;
    }
    else if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
      // Block 2 rotates with respect to upper-right corner of block 1
      float rotateAng = (Blks[2].desHeading > Blks[2].heading)? rotAngVel : -rotAngVel;
      Rotate(2,rotateAng, Blks[1].getCorner(0));
      shapeShifting = true;
    }
    else if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
      // Block 3 rotates with respect to upper-left corner of block 2
      float rotateAng = (Blks[3].desHeading > Blks[3].heading)? rotAngVel : -rotAngVel;
      Rotate(3,rotateAng, Blks[2].getCorner(1));
      shapeShifting = true;
    }
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
    } else {
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
    shapeShift();
    if (!shapeShifting){
      
      float[] decipheredCmd = cmdDecipher(brain.Cmds[time]);
      vel = new PVector (decipheredCmd[0], decipheredCmd[1]);
      pos.add(vel);
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        Blks[blkidx].pos.add(vel);
      }
    }
    
  }

  //---------------------------------------------------------------------------------------------------------------------------------------
  //clone it 
  Robot gimmeBaby() {
    Robot baby = new Robot();
    baby.brain = brain.clone();//babies have the same brain as their parents
    baby.fitness = fitness;
    return baby;
  }
}
