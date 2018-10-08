class Population {
  
  Robot[] Rbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 1;

  int bestRobot = 0;//the index of the best dot in the dots[]

  int minStep;

  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    minStep = minStep_;
    Rbts = new Robot[size];
    for (int i = 0; i< size; i++) {
      Rbts[i] = new Robot();
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
      if (Rbts[rbtidx].brain.step > minStep) {//if the dot has already taken more steps than the best dot has taken to reach the goal
        Rbts[rbtidx].dead = true;
      } else if (isCollide(Rbts[rbtidx])){
        Rbts[rbtidx].dead = true;
      } else{
        Rbts[rbtidx].update();
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
    for (int i = 0; i< Rbts.length; i++) {
      Rbts[i].calculateFitness();
    }
  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //returns whether all the dots are either dead or have reached the goal
  boolean allDotsDead() {
    for (int i = 0; i< Rbts.length; i++) {
      if (!Rbts[i].dead && !Rbts[i].reachedGoal) { 
        return false;
      }
    }
    return true;
  }



  //-------------------------------------------------------------------------------------------------------------------------------------

  //gets the next generation of dots
  void naturalSelection() {
    Robot[] newRbts = new Robot[Rbts.length];//next gen
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

  //this function works by randomly choosing a value between 0 and the sum of all the fitnesses
  //then go through all the dots and add their fitness to a running sum and if that sum is greater than the random value generated that dot is chosen
  //since dots with a higher fitness function add more to the running sum then they have a higher chance of being chosen
  Robot selectParent() {
    float rand = random(fitnessSum);


    float runningSum = 0;

    for (int i = 0; i< Rbts.length; i++) {
      runningSum+= Rbts[i].fitness;
      if (runningSum > rand) {
        return Rbts[i];
      }
    }

    //should never get to this point

    return null;
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  //mutates all the brains of the babies
  void mutateDemBabies() {
    for (int i = 1; i< Rbts.length; i++) {
      Rbts[i].brain.mutate();
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

    //if this dot reached the goal then reset the minimum number of steps it takes to get to the goal
    if (Rbts[bestRobot].reachedGoal) {
      minStep = Rbts[bestRobot].brain.step;
      println("step:", minStep);
    }
  }
}
