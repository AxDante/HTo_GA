class Population {
  
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
          curRbt.dead = true;
        }
        if (isCollide(Rbts[rbtidx])){
          curRbt.dead = true;
        } 
        if (curRbt.Blks[1].pos.x < blkWidth/2.0|| curRbt.Blks[1].pos.y < blkWidth/2.0 || curRbt.Blks[1].pos.x > width - blkWidth/2.0 || curRbt.Blks[1].pos.y > height - blkWidth/2.0) { //if near the edges of the window then kill it 
          curRbt.dead = true;
        } 
        if (noRepeatingGrids){
          if (time > 3){
            for (int grididx = 1; grididx <= 4; grididx++){
              if (curRbt.Blks[1].pos.x == curRbt.brain.pastPos[time - grididx].x && curRbt.Blks[1].pos.y == curRbt.brain.pastPos[time - grididx].y){
                curRbt.dead = true;
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
    //println("Calculating fitness");
    for (int i = 0; i< Rbts.length; i++) {
      if (Rbts[i].reachedGoal) {
        Rbts[i].fitness = 1.0/16.0 + 10000.0/(float)(Rbts[i].brain.curTime * Rbts[i].brain.curTime);
      } else {
        float distanceToGoal = dist(Rbts[i].Blks[1].pos.x, Rbts[i].Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y);
        Rbts[i].fitness = 1.0/(distanceToGoal * distanceToGoal);
      }
      //println("Robot " + nf(i,4) + " | ReachGoal: " + Rbts[i].reachedGoal + "  curTime: " + nf(Rbts[i].brain.curTime,3) + "  Fitness: " + nf(Rbts[i].fitness,1,7));
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
    int minConvergeSample = 2;
    int maxConvergeDiff = 3;
    if (time > minConvergeSample) { 
      if (StepData[time-minConvergeSample] != -1 && (StepData[time-minConvergeSample] - StepData[time]) <= maxConvergeDiff){
        return true;
      }
    }
    return false;
  }



  //-------------------------------------------------------------------------------------------------------------------------------------

  //gets the next generation of dots
  void naturalSelection() {
    Robot[] newRbts = new Robot[Rbts.length]; //next gen
    setbestRobot();
    calculateFitnessSum();

    //the champion lives on 
    newRbts[0] = Rbts[bestRobot].gimmeBaby();
    newRbts[0].isBest = true;
    for (int i = 1; i< newRbts.length; i++) {
      //select parent based on fitness
      Robot parent = selectParent();

      //get baby from them
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
      runningSum+= Rbts[i].fitness;
      if (runningSum > rand) {
        return Rbts[i];
      }
    }
    return null;
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  //mutates all the brains of the babies
  void mutateDemBabies() {
    for (int i = 1; i< Rbts.length; i++) {
      Rbts[i].mutate();
    }
  }

  //---------------------------------------------------------------------------------------------------------------------------------------------
  //finds the dot with the highest fitness and sets it as the best dot
  void setbestRobot() {
    float max = 0;
    int maxIndex = 0;

    for (int i = 0; i< Rbts.length; i++) {
      if (Rbts[i].fitness > max) {
        max = Rbts[i].fitness;
        maxIndex = i;
      }
    }
    
    bestRobot = maxIndex;
    if (Rbts[bestRobot].reachedGoal) {
      bestTime = Rbts[bestRobot].brain.curTime;
      println("step:", bestTime);
    }else{
      StepData[time] = -1;
    }
  }
}
