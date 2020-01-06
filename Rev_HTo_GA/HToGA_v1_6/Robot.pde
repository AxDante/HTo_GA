class Robot {
  
  int popID;        // Population ID
  int frontRank;    // Front rank number

  String generateType; 

  float costCD;
  float smoothCD;
  float safeCD;
  float sumCD;
  
  PVector pos;      // Robot position (x,y)
  PVector vel;      // Robot velocity 
  Brain brain;      // Robot Brain

  boolean isDead = false;       
  boolean isReachGoal = false;
  boolean isBest = false;     // Not used #TODO
  boolean isPerc = false;
  boolean isShapeShifting = false;
  
  float costScore = 0;
  float smoothScore = 0;
  float safeScore = 0;
  float sumScore = 0;
  
  int bestFitnessTime = 0;
  float fitness = 0;

  Block[] Blks;
  int morph = 2;    // Robot morphology
  Morphology Morph;
  
  Robot() {
    
    Morph = new Morphology();
    brain = new Brain(maxTime); 

    //start the dots at the bottom of the window with a no velocity
    
    pos = map.WpList.get(currentWpID).pos.copy();
    vel = new PVector(0, 0);
    
    Blks = new Block[4];
    Blks[0] = new Block(0, blkW , pos.copy().add(new PVector(blkW,0)), 0);
    Blks[1] = new Block(1, blkW , pos.copy().add(new PVector(0,0)), 0);
    Blks[2] = new Block(2, blkW , pos.copy().add(new PVector(-blkW,0)), 0);
    Blks[3] = new Block(3, blkW , pos.copy().add(new PVector(-blkW*2,0)), 0);
    
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
      isShapeShifting = false;
      if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
        float rotateAng = Blks[0].desHeading - Blks[0].heading;
        Rotate(0,rotateAng, Blks[1].getCorner(3));
        isShapeShifting = true;
      }
      if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
        // Block 2 rotates with respect to upper-right corner of block 1
        float rotateAng = Blks[2].desHeading - Blks[2].heading;
        Rotate(2,rotateAng, Blks[1].getCorner(0));
        isShapeShifting = true;
      }
      if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
        // Block 3 rotates with respect to upper-left corner of block 2
        float rotateAng = Blks[3].desHeading - Blks[3].heading;
        Rotate(3,rotateAng, Blks[2].getCorner(1));
        isShapeShifting = true;
      }
      if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
        float rotateAng = Blks[1].desHeading - Blks[1].heading;
        RotateCenter(rotateAng);
        isShapeShifting = true;
      }
    } else{
      isShapeShifting = false;
      if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
        // Block 2 rotates with respect to bottom-right corner of block 1
        float rotateAng = (Blks[0].desHeading > Blks[0].heading)? rotAngVel : -rotAngVel;
        Rotate(0,rotateAng, Blks[1].getCorner(3));
        isShapeShifting = true;
      }
      else if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
       float rotateAng = (Blks[1].desHeading > Blks[1].heading)? rotAngVel : -rotAngVel;
       RotateCenter(rotateAng);
       isShapeShifting = true;
      }
      else if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
        // Block 2 rotates with respect to upper-right corner of block 1
        float rotateAng = (Blks[2].desHeading > Blks[2].heading)? rotAngVel : -rotAngVel;
        Rotate(2,rotateAng, Blks[1].getCorner(0));
        isShapeShifting = true;
      }
      else if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
        // Block 3 rotates with respect to upper-left corner of block 2
        float rotateAng = (Blks[3].desHeading > Blks[3].heading)? rotAngVel : -rotAngVel;
        Rotate(3,rotateAng, Blks[2].getCorner(1));
        isShapeShifting = true;
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
  
  // ***** CHECKED 2020
  //draws the dot on the screen 
  void show() {
    if (isBest) {      // TODO: Redundent right now, need to adjust to MOGA front 
      if (colorfulRobot){
        fill(0, 255, 0);
        rect(Blks[1].pos.x, Blks[1].pos.y, Blks[1].blkW, Blks[1].blkW);
        fill(0, 205, 100);
        rect(Blks[0].pos.x, Blks[0].pos.y, Blks[0].blkW, Blks[0].blkW);
        fill(100, 205, 0);
        rect(Blks[2].pos.x, Blks[2].pos.y, Blks[2].blkW, Blks[2].blkW);
        fill(150, 155, 0);
        rect(Blks[3].pos.x, Blks[3].pos.y, Blks[3].blkW, Blks[3].blkW);
      }else{
        for (int blkidx = 0; blkidx < Blks.length; blkidx++){
          fill(0, 255, 0);
          rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkW, Blks[blkidx].blkW);
        }
      }
    } else {
      fill(0);
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 0, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkW, Blks[blkidx].blkW);
      }
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------
  //moves the dot according to the brains directions
  void move() {
   
    updateBlockDesHeading();  
    shapeShift();
    
    if (brain.Cmds.size() <= time) return;  // TODO: Keep or not?
    
    // Perform movment if not in the middle of shapeshifting
    if (!isShapeShifting){
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
  Robot generateChild() {
    Robot child = new Robot();
    child.brain = brain.clone();
    
    child.fitness = fitness;
    
    // TODO: Unsure whether this is needed
    child.costScore = costScore;
    child.smoothScore = smoothScore;
    child.safeScore = 0;
    child.sumScore = sumScore;
    return child;
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
        if (!isValidGrid(gridID, pos)){
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
