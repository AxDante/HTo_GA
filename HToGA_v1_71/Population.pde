class Population {
  
  Robot[] prevGoodRbts;
  Robot[] Rbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;

  int bestRobot = 0;//the index of the best dot in the dots[]

  int[] StepData;
  int bestTime;
  float bestFitness;
  
  
  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    bestTime = minStep_;
    Rbts = new Robot[size];
    for (int i = 0; i < size; i++) {
      Rbts[i] = new Robot();
    }
    StepData = new int[minStep_];
    for (int i = 0; i < StepData.length; i++) {
      StepData[i] = -1;
    }
  }


  //------------------------------------------------------------------------------------------------------------------------------
  //show all dots
  void show() {
    for (int i = 1; i< Rbts.length; i++) {
      Rbts[i].show();
    }
    Rbts[0].show();
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  //update all Robots
  void update() {
    Robot curRbt;
    for (int rbtidx = 0; rbtidx< Rbts.length; rbtidx++) {
      curRbt = Rbts[rbtidx];
      if (!curRbt.dead){
        if (curRbt.brain.curTime > bestTime){
          //println("died because of timeout");
          curRbt.dead = true;
        }
        if (isCollide(Rbts[rbtidx])){
          //println("died because obstacle collision");
          //curRbt.dead = true;
        } 
        if (isCollideGrid(Rbts[rbtidx])){
          //println("died because obstacle collision");
          curRbt.dead = true;
        } 
        if (curRbt.Blks[1].pos.x < blkWidth/2.0|| curRbt.Blks[1].pos.y < blkWidth/2.0 || curRbt.Blks[1].pos.x > map.mapSize.x - blkWidth/2.0 || curRbt.Blks[1].pos.y > map.mapSize.y - blkWidth/2.0) { //if near the edges of the window then kill it 
          //println("died because of wall collision");
          curRbt.dead = true;
        } 
        if (noRepeatingGrids && !curRbt.shapeShifting){
          if (time > 10){
            for (int grididx = 2; grididx <= 10; grididx += 2){
              if (curRbt.Blks[1].pos.x == curRbt.brain.pastPos[time - grididx].x && curRbt.Blks[1].pos.y == curRbt.brain.pastPos[time - grididx].y){
                //println("died because of repeating grids at position " + curRbt.Blks[1].pos.x + "," + curRbt.Blks[1].pos.y);
                
                //curRbt.dead = true; 
              }
            }
          }
        }
      }
      if (!curRbt.dead){   
        if (!curRbt.reachedGoal){
          if (dist(curRbt.Blks[1].pos.x, curRbt.Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y) < 15) {
            curRbt.reachedGoal = true;
          }else{
            curRbt.move();
            curRbt.brain.curTime += 1;
          }
        }
        if (gridBasedMode && noRepeatingGrids){
          curRbt.brain.pastPos[time] = Rbts[rbtidx].pos;
        }
        if (!curRbt.reachedGoal && curRbt.fitness == 0){
        //  print(" bestFitness " + curRbt.fitness);
        }
      }
    }
  }
  
  //-------------------------------------------------------------------------------------------------------------------------------
  // check for collisions
  boolean isCollideGrid(Robot rbt) {
    int[] blkGrid;
    for (int blkidx = 0; blkidx < rbt.Blks.length; blkidx++){
      blkGrid = rbt.Blks[blkidx].blkGridPos(map.mapSize);
      if (gridObsTable[blkGrid[0]][blkGrid[1]] == 1){
        return true;
      };
    }
    return false;
  }
  
  boolean isCollide(Robot rbt) {
    for (int obsidx = 0; obsidx < Obss.length; obsidx++) {
      for (int blkidx = 0; blkidx < rbt.Blks.length; blkidx++){
        if (isPointInRect(rbt.Blks[blkidx].pos, Obss[obsidx].pos, Obss[obsidx].size)){
          return true;
        }
        //for (int coridx = 0; coridx < 4; coridx++){
        //  if (isPointInRect(rbt.Blks[blkidx].getCorner(coridx), Obss[obsidx].pos, Obss[obsidx].size)){
        //    return true;
        //  }
        
      }
      //if (isRectIntersect(rbt.pos, new PVector (30, 30), 0, Obss[obsidx].pos, Obss[obsidx].size, 0)){
      //  return true;
      //}
    }
    return false;
  }

  boolean isRectIntersect(PVector a_c, PVector a_s, float a_deg, PVector b_c, PVector b_s, float b_deg){
    if (isPointInRect(a_c, b_c, b_s) || isPointInRect(new PVector(a_c.x + a_s.x, a_c.y), b_c, b_s) 
        || isPointInRect(new PVector(a_c.x, a_c.y + a_s.y), b_c, b_s)  || isPointInRect(new PVector(a_c.x + a_s.x, a_c.y + a_s.y), b_c, b_s)){
      return true;
    }else{
      return false;
    }
  }

  boolean isPointInRect(PVector pt, PVector b_c, PVector b_s){
    float area = b_s.x * b_s.y;
    PVector corner1 = b_c;
    PVector corner2 = new PVector(b_c.x + b_s.x, b_c.y);
    PVector corner3 = new PVector(b_c.x , b_c.y + b_s.y);
    PVector corner4 = new PVector(b_c.x + b_s.x, b_c.y + b_s.y);
    float area1 = area(pt, corner1, corner2);
    float area2 = area(pt, corner2, corner3);
    float area3 = area(pt, corner3, corner4);
    float area4 = area(pt, corner4, corner1);
    if (area >= area1 + area2 + area3 + area4 - 10){
      return true;
    }else{
      return false;
    }
  }
  
  float area(PVector pt1, PVector pt2, PVector pt3) { 
    return (float)Math.abs((pt1.x * (pt2.y - pt3.y) + pt2.x * (pt3.y - pt1.y) + pt3.x * (pt1.y - pt2.y)) / 2.0); 
  }


  //-----------------------------------------------------------------------------------------------------------------------------------
  //calculate all the fitnesses
  void calculateFitness() {
    for (int i = 0; i< Rbts.length; i++) {
      if (Rbts[i].reachedGoal) {
        Rbts[i].fitness = 10 + 10000.0/(float)(Rbts[i].brain.curTime * Rbts[i].brain.curTime);
        //Rbts[i].fitness = 1.0/16.0 + 10000.0/(float)(Rbts[i].brain.curTime * Rbts[i].brain.curTime);
      } else {
        //println("posx posy" + Rbts[i].Blks[1].pos.x + "," + Rbts[i].Blks[1].pos.y);
        int[] gridPos = Rbts[i].Blks[1].blkGridPos(map.mapSize);
        print("finalPos  " + gridPos[0] + "," + gridPos[1] + "   ");
        //println(" fitH " + (AstarFitness[gridPos[0]][gridPos[1]]));
        
        Rbts[i].fitness = (float)(1/(1+(AstarFitness[gridPos[0]][gridPos[1]])));
        //float distanceToGoal = dist(Rbts[i].Blks[1].pos.x, Rbts[i].Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y);
        //Rbts[i].fitness = 1.0/(distanceToGoal * distanceToGoal);
      }
      println("Robot " + nf(i,4) + " | ReachGoal: " + Rbts[i].reachedGoal + "  curTime: " + nf(Rbts[i].brain.curTime,3) + "  Fitness: " + nf(Rbts[i].fitness,1,7));
    }
  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //returns whether all the dots are either dead or have reached the goal
  boolean allRobotsDead() {
    for (int i = 0; i< Rbts.length; i++) {
      if (!Rbts[i].dead && !Rbts[i].reachedGoal) { 
        return false;
      }
    }
    return true;
  }
  
  boolean isConverged() {
    int minConvergeSample = 4;
    int maxConvergeDiff = 0;
    if (gen-1 > minConvergeSample) {
      int genDiff = (StepData[gen-1-minConvergeSample] - StepData[gen-1]);
      println("gen resutls difference:  " + genDiff);
      if (StepData[gen-1-minConvergeSample] != -1 && (StepData[gen-1-minConvergeSample] - StepData[gen-1]) <= maxConvergeDiff){
        return true;
      }
    }
    return false;
  }



  //-------------------------------------------------------------------------------------------------------------------------------------


  void updateStepFitness(){
    float max = 0;
    int maxIndex = 0;

    for (int i = 0; i< Rbts.length; i++) {
      if (Rbts[i].fitness > max) {
        max = Rbts[i].fitness;
        maxIndex = i;
      }
    }
    bestRobot = maxIndex;
  }



  //gets the next generation of dots
  void naturalSelection() {
    Robot[] newRbts = new Robot[Rbts.length]; //next gen
    setbestRobot();
    calculateFitnessSum();
    
    //the champion lives on 
    newRbts[0] = Rbts[bestRobot].gimmeBaby();
    newRbts[0].isBest = true;
    for (int i = 1; i< newRbts.length; i++) {
      Robot parent = selectParent();
      newRbts[i] = parent.gimmeBaby();
    }
    Rbts = newRbts.clone();
    gen ++;
  }


  //--------------------------------------------------------------------------------------------------------------------------------------
  //you get it
  void calculateFitnessSum() {
    fitnessSum = 0;
    for (int i = 0; i< Rbts.length; i++) {
      fitnessSum += Rbts[i].fitness;
    }
   
  }

  //-------------------------------------------------------------------------------------------------------------------------------------

  //chooses dot from the population to return randomly(considering fitness)
  Robot selectParent() {
    float rand = random(fitnessSum);
    float runningSum = 0;
    for (int i = 0; i< Rbts.length; i++) {
      runningSum += Rbts[i].fitness;
      if (runningSum > rand) {
        return Rbts[i];
      }
    }
    return null;
  }

  Robot[] tournamentSelect(){
    
    
    Robot[] winRbts = new Robot[2];
    int[] randRbtID = new int[4];
    for (int rbtidx = 0; rbtidx < 4; rbtidx++){
      randRbtID[rbtidx] = floor(random(Rbts.length));
    }
    winRbts[0] = (Rbts[randRbtID[0]].fitness > Rbts[randRbtID[1]].fitness)? Rbts[randRbtID[0]] : Rbts[randRbtID[1]];
    winRbts[1] = (Rbts[randRbtID[2]].fitness > Rbts[randRbtID[3]].fitness)? Rbts[randRbtID[2]] : Rbts[randRbtID[3]];
    return winRbts;
  }


  //------------------------------------------------------------------------------------------------------------------------------------------
  //mutate
  void GAMutation() {
    for (int i = 1; i< Rbts.length; i++) {
      for (int cmdidx = 0; cmdidx < Rbts[i].brain.Cmds.length; cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess){
          int randMorph = floor(random(7));
          //Rbts[i].morph = randMorph;
          Rbts[i].brain.Cmds[cmdidx].transMorph = randMorph;
        }else if (rand < baseMutMoveRate) {
          if (!gridBasedMode){
            float randomAngle = random(2*PI);
            Rbts[i].brain.Cmds[cmdidx].moveDir = PVector.fromAngle(randomAngle);
          }else{
            float randomAngle = ((int)random(4))*PI/2;
            Rbts[i].brain.Cmds[cmdidx].moveDir = PVector.fromAngle(randomAngle).mult(blkWidth);
          }
        }
      }
    }
  }
  
  void GACrossover() {
    for (int i = 1; i< Rbts.length; i++) {
      if (random(1) < baseCrossoverRate){
        Robot[] parents = tournamentSelect();
        Robot childRbt = parents[0];
        int cutPoint = floor(random(bestTime));
        for (int cmdidx = 0; cmdidx < cutPoint; cmdidx++){
          //childRbt.brain.Cmds[cmdidx] = parents[0].brain.Cmds[cmdidx];
          childRbt.brain.Cmds[cmdidx] = Rbts[bestRobot].brain.Cmds[cmdidx];
        }
        for (int cmdidx = cutPoint; cmdidx < childRbt.brain.Cmds.length; cmdidx++){
          childRbt.brain.Cmds[cmdidx] = parents[1].brain.Cmds[cmdidx];
        }
        //Robot parent = selectParent();
        //Robot anotherRbt  = parent.gimmeBaby();
        
        //Command[] newCmd = Rbts[i].brain.Cmds;
        //for (int cmdidx = cutPoint; cmdidx < newCmd.length; cmdidx++){
        //  newCmd[cmdidx] = newCmd[cmdidx];
        //  childRbt.brain.Cmds[cmdidx] = childRbt.brain.Cmds[cmdidx];
        //}
        Rbts[i].brain = childRbt.brain.clone();
      }
    }
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------
  //finds the dot with the highest fitness and sets it as the best dot
  void setbestRobot() {
    
    if (Rbts[bestRobot].reachedGoal) {
      bestTime = Rbts[bestRobot].brain.curTime;
      StepData[gen] = bestTime;
      println("bestTime:", bestTime);
    }else{
      StepData[gen] = -1;
    }
  }
}
