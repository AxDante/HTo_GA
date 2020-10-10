class Robot {
  
  int popID;
  int frontRank;
  float costCD;
  float smoothCD;
  float safeCD;
  float sumCD;
  
  PVector pos;
  PVector vel;
  Brain brain;

  boolean dead = false;
  boolean reachedGoal = false;
  boolean isBest = false;
  boolean isPerc = false;
  boolean shapeShifting = false;
  
  float costScore = 0;
  float smoothScore = 0;
  float safeScore = 0;
  float sumScore = 0;
  
  int bestFitnessTime = 0;
  float fitness = 0;

  Block[] Blks;
  int morph = 2;
  Morphology Morph;
  
  Robot() {
    
    Morph = new Morphology();
    brain = new Brain(maxTime); //new brain with 1000 instructions

    //start the dots at the bottom of the window with a no velocity
    pos = new PVector(map.Wps[currentWpID].pos.x, map.Wps[currentWpID].pos.y);
    vel = new PVector(0, 0);
    
    Blks = new Block[4];
    Blks[0] = new Block(0, blkWidth , pos.copy().add(new PVector(blkWidth,0)), 0);
    Blks[1] = new Block(1, blkWidth , pos.copy().add(new PVector(0,0)), 0);
    Blks[2] = new Block(2, blkWidth , pos.copy().add(new PVector(-blkWidth,0)), 0);
    Blks[3] = new Block(3, blkWidth , pos.copy().add(new PVector(-blkWidth*2,0)), 0);
    
    updateBlockDesHeading();
    initializeBlockDesHeading();
  }
  
  //-----------------------------------------------------------------------------------------------------------------
  //Update desired heading to each robot blocks
  void updateBlockDesHeading(){
    //println(time);
    if (time < brain.Cmds.size()){
      int newMorph = int(brain.Cmds.get(time));
      
      //Moving -> No changes in morphology
      if (newMorph == 0){ 
        return;
      //Changes in morphology
      }else if (newMorph != morph){
        morph = newMorph;
      }
      
      //Perform shapeshifting
      int modMorph = morph % morphNum;
      int divMorph = floor((morph-1)/morphNum);
      for(int blkidx = 0; blkidx < 4; blkidx++){
        float newHeading = Morph.RelAng[modMorph-1][blkidx] - divMorph * PI/2;
        // Set desired heading of each block according to the array values
        Blks[blkidx].desHeading = newHeading;
      }
    }else{
      return;
    }
  }

  void initializeBlockDesHeading(){
    
    int modMorph = morph % morphNum;
    int divMorph = floor((morph-1)/morphNum);
    for(int blkidx = 0; blkidx < 4; blkidx++){
      float newHeading = Morph.RelAng[modMorph-1][blkidx] - divMorph * PI/2;
      // Set desired heading of each block according to the array values
      Blks[blkidx].desHeading = newHeading;
    }
  }


  //-----------------------------------------------------------------------------------------------------------------
  // perform robot shape-shifting
  void shapeShift(){
    
    if (snapShift){
      
      shapeShifting = false;
      if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
        float rotateAng = Blks[0].desHeading - Blks[0].heading;
        Rotate(0,rotateAng, Blks[1].getCorner(3));
        shapeShifting = true;
      }
      if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
        // Block 2 rotates with respect to upper-right corner of block 1
        float rotateAng = Blks[2].desHeading - Blks[2].heading;
        Rotate(2,rotateAng, Blks[1].getCorner(0));
        shapeShifting = true;
      }
      if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
        // Block 3 rotates with respect to upper-left corner of block 2
        float rotateAng = Blks[3].desHeading - Blks[3].heading;
        Rotate(3,rotateAng, Blks[2].getCorner(1));
        shapeShifting = true;
      }
      if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
        float rotateAng = Blks[1].desHeading - Blks[1].heading;
        RotateCenter(rotateAng);
        shapeShifting = true;
      }
    } else{
      shapeShifting = false;
      if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
        // Block 2 rotates with respect to bottom-right corner of block 1
        float rotateAng = (Blks[0].desHeading > Blks[0].heading)? rotAngVel : -rotAngVel;
        Rotate(0,rotateAng, Blks[1].getCorner(3));
        shapeShifting = true;
      }
      else if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
       float rotateAng = (Blks[1].desHeading > Blks[1].heading)? rotAngVel : -rotAngVel;
       RotateCenter(rotateAng);
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
  }
  
  void Rotate(int blkId, float angle, PVector posCtr){
    Blks[blkId].heading = Blks[blkId].heading + angle;
    PVector posToCenter = Blks[blkId].pos.copy().sub(posCtr);
    PVector posShift = posToCenter.copy().rotate(angle).sub(posToCenter);
    Blks[blkId].pos.add(posShift);
    
    if (blkId == 2){
      Rotate(3, angle, posCtr); // If block 2 rotates, rotate block 3 as well
    }
    if(debugMode){
      println("Block " + blkId + " Rotation:");
      println("posCtr (x,y) = (" + posCtr.x + ", " + posCtr.y +").");
      println("posToCenter (x,y) = (" + posToCenter.x + ", " + posToCenter.y +").");
      println("posToCenter (x,y).rotate(angle) = (" + posToCenter.rotate(angle).x + ", " + posToCenter.rotate(angle).y +").");
      println("posToCenter (x,y).rotate(angle).sub(posToCenter) = (" + posToCenter.rotate(angle).sub(posToCenter).x + ", " + posToCenter.rotate(angle).sub(posToCenter).y +").");
    }
  }
  
  void RotateCenter(float angle){
    
    PVector posCtr = Blks[1].pos.copy();
    Blks[1].heading = Blks[1].heading + angle;

    Rotate(0, angle, posCtr); 
    Rotate(2, angle, posCtr);
  }

  //-----------------------------------------------------------------------------------------------------------------
  //draws the dot on the screen
  void show() {
    if (isBest) {
      if (colorfulRobot){
        fill(0, 255, 0);
        rect(Blks[1].pos.x, Blks[1].pos.y, Blks[1].blkWidth, Blks[1].blkWidth);
        fill(0, 205, 100);
        rect(Blks[0].pos.x, Blks[0].pos.y, Blks[0].blkWidth, Blks[0].blkWidth);
        fill(100, 205, 0);
        rect(Blks[2].pos.x, Blks[2].pos.y, Blks[2].blkWidth, Blks[2].blkWidth);
        fill(150, 155, 0);
        rect(Blks[3].pos.x, Blks[3].pos.y, Blks[3].blkWidth, Blks[3].blkWidth);
      }else{
        for (int blkidx = 0; blkidx < Blks.length; blkidx++){
          fill(0, 255, 0);
          rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
        }
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
    
    if (brain.Cmds.size() <= time) return;
    
    if (!shapeShifting){
      
      float[] decipheredCmd = cmdDecipher(brain.Cmds.get(time));
      
      vel = new PVector (decipheredCmd[0], decipheredCmd[1]);

      if (robotPerception){
        vel = mutateMoveDis(new PVector (decipheredCmd[0], decipheredCmd[1]),rbtPcepPattern);
        for (int cmdidx = 0; cmdidx < fourDirArray.length; cmdidx++){
          if (vel.copy().dist(fourDirArray[cmdidx]) < 0.01){
            brain.Cmds.set(time,fourDirString[cmdidx]); 
          }
        }
      }
      
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
    baby.brain = brain.clone();
    
    baby.fitness = fitness;
    
    // TODO: Unsure whether this is needed
    baby.costScore = costScore;
    baby.smoothScore = smoothScore;
    baby.safeScore = 0;
    baby.sumScore = sumScore;
    return baby;
  }
  
  float fourDirFitness(int[] gridID, int sd){
    if (sd == 0){
      return (float)(10/(1+getGridFitnessValue(gridID[0],gridID[1], 1)));
    }else{
      int validGrids = 0;
      int[] nextGrid = new int[2];
      float[] nextGridScore = new float[] {1, 1, 1, 1};
      for (int grididx = 0; grididx < nextGridScore.length; grididx++){
        nextGrid = new int[] {gridID[0] + fourDirGridArray.get(grididx)[0],
                              gridID[1] + fourDirGridArray.get(grididx)[1]};
        if (!(nextGrid[0] >= mapW || nextGrid[1] >= mapH || nextGrid[0] < 0 || nextGrid[1] < 0)){
          validGrids++;
          nextGridScore[grididx] = fourDirFitness(nextGrid, sd-1);
          //println(nextGrid[0] + "," + nextGrid[1] + "  " + nextGridScore[grididx]);
        }else{
          nextGridScore[grididx] *= Wobs;
        }
      }

      return pow(nextGridScore[0]*nextGridScore[1]*nextGridScore[2]*nextGridScore[3],1.0/validGrids);
    }
  }
  
  PVector mutateMoveDis(PVector vel, int sd){
    
    PVector nvel = new PVector();
    PVector[] nextGrid = new PVector[8];
    float[] nextGridScore = new float[] {1, 1, 1, 1};
    
    for (int grididx = 0; grididx < 8; grididx++){
      for (int blkidx = 0; blkidx < 4; blkidx++){
        
        nextGrid[grididx] = Blks[blkidx].pos.copy().add(doubleDirArray[grididx]);
        int[] gridID = getGridID(nextGrid[grididx]);
        float outp = fourDirFitness(gridID, 1);
        if (grididx < 4) nextGridScore[grididx] *= outp;
        if (!isValidGrid(gridID)){
          if (grididx < 4) nextGridScore[grididx] *= Wobs;
          if (grididx < 4){
            safeScore += 1;
          }else{
            safeScore += 2;
          }
        }
      }
      if (vel.copy().dist(doubleDirArray[grididx]) == 0){
        if (grididx < 4) nextGridScore[grididx] *= WgeneDir;
      }
    }
    if (isPerc){
      int selectidx = PortionSelect(nextGridScore);
      nvel = fourDirArray[selectidx];
    }
    else{
      nvel = vel.copy();
    }
    return nvel;
  }
}
