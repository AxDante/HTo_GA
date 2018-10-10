class Population {
  
  Robot[] Rbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;
  

  int bestRobot = 0;//the index of the best dot in the dots[]

  int[] StepData;
  int minStep;
  
  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    minStep = minStep_;
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
  //update all dots 
  void update() {
    for (int rbtidx = 0; rbtidx< Rbts.length; rbtidx++) {
      if (Rbts[rbtidx].brain.bestTime > minStep) {
        Rbts[rbtidx].dead = true;
      } else if (isCollide(Rbts[rbtidx])){
        Rbts[rbtidx].dead = true;
      } else{
        if (gridBasedMode && noRepeatingGrids){
          Rbts[rbtidx].brain.pastPos[time] = Rbts[rbtidx].pos;
        }
        Rbts[rbtidx].update();
      }
    }
    //println(StepData.length + ",  " + time);
    //println(Rbts[0].pos.x);
    //println("Time: " + time + ", current position (x,y) = (" + Rbts[0].Blks[1].pos.x + ", " + Rbts[0].Blks[1].pos.y + " StepData: " +  StepData[time]);
    time++;
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
    for (int i = 0; i< Rbts.length; i++) {
      Rbts[i].calculateFitness();
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
      //float k = time-minConvergeSample;
      //println("time-minConvergeSample " + k);
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
    println("=======================");
    
    for (int i = 0; i< Rbts.length; i++) {
      println("Robot No. " + i );
      println("Reached Goal: " + Rbts[i].reachedGoal + "; bestTime: " + Rbts[i].brain.bestTime + " ; bestFitness: " + Rbts[i].brain.bestFitness) ;
      if (Rbts[i].fitness > max) {
        max = Rbts[i].fitness;
        maxIndex = i;
      }
    }
    println("Best Robot this gen is robot " + maxIndex);
    
    bestRobot = maxIndex;
    //if this dot reached the goal then reset the minimum number of steps it takes to get to the goal
    if (Rbts[bestRobot].reachedGoal) {
      minStep = time;
      //minStep = Rbts[bestRobot].brain.step;
      //StepData[time] = minStep;
      println("step:", minStep);
    }else{
      StepData[time] = -1;
    }
  }
}
