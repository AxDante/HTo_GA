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
        if (curRbt.brain.curTime > bestTime || curRbt.brain.curTime >= maxTime){
          curRbt.dead = true;
        } 
        if (isCollideRbt(Rbts[rbtidx])){
          curRbt.dead = true;
        } 
        if (curRbt.brain.Cmds.size() <= time){
          curRbt.dead = true;
        }
        //if (curRbt.Blks[1].pos.x < blkWidth/2.0|| curRbt.Blks[1].pos.y < blkWidth/2.0 || curRbt.Blks[1].pos.x > map.mapSize.x - blkWidth/2.0 || curRbt.Blks[1].pos.y > map.mapSize.y - blkWidth/2.0) { //if near the edges of the window then kill it 
        //  curRbt.dead = true;
        //} 
      }
      
      //println("t-" + time);
      //println("curt-" + curRbt.brain.curTime);
      //println("best-" + bestTime);
      //println("-" + maxTime);
      if (!curRbt.dead){   
        if (!curRbt.reachedGoal){
          if (dist(curRbt.Blks[1].pos.x, curRbt.Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y) < 25) {
            curRbt.reachedGoal = true;
            //for (int cmdidx = bestTime+1; cmdidx < curRbt.brain.Cmds.size(); cmdidx++){
            //  curRbt.brain.Cmds.set(cmdidx,"S");
            //}
            for(int cmdidx = curRbt.brain.Cmds.size()-1; cmdidx > bestTime; cmdidx --){
              curRbt.brain.Cmds.remove(cmdidx);
            }
            println( curRbt.brain.Cmds.size());
          }else{
            curRbt.move();
            curRbt.brain.curTime += 1;
          }
        }
        if (noRepeatingGrids && time < curRbt.brain.pastPos.length){
          curRbt.brain.pastPos[time] = Rbts[rbtidx].pos;
        }
      }
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------------------
  //calculate all the fitnesses
  void calculateFitness() {
    for (int i = 0; i< Rbts.length; i++) {
      float tempFitness = 0;
      if (Rbts[i].reachedGoal) {
        tempFitness = 10 + 10000.0/(float)(Rbts[i].brain.curTime * Rbts[i].brain.curTime);
      } else {
        int[] gridPos = Rbts[i].Blks[1].blkGridPos(map.mapSize);
        tempFitness = (float)(10/(1+pow((float)AstarFitness[gridPos[0]][gridPos[1]],4)));
      }
      if (progressingFitness){
        if (tempFitness > Rbts[i].fitness){
          Rbts[i].fitness = tempFitness;
        }
      }else{
        Rbts[i].fitness = tempFitness;
      }
      //println("Robot " + nf(i,4) + " | ReachGoal: " + Rbts[i].reachedGoal + "  curTime: " + nf(Rbts[i].brain.curTime,3) + "  Fitness: " + nf(Rbts[i].fitness,1,7));
    }
  }
  
  void resetFitness() {
    for (int i = 0; i< Rbts.length; i++) {
      Rbts[i].fitness = 0;
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
    int minConvergeSample = 10;
    int maxConvergeDiff = 0;
    if (gen-1 > minConvergeSample) {
      int genDiff = (StepData[gen-1-minConvergeSample] - StepData[gen-1]);
      println("generation resutls difference:  " + genDiff);
      if (StepData[gen-1-minConvergeSample] != -1 && (StepData[gen-1-minConvergeSample] - StepData[gen-1]) <= maxConvergeDiff){
        return true;
      }
    }
    return false;
  }

  //-------------------------------------------------------------------------------------------------------------------------------------

  void findBestFitness(){
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

  void naturalSelection() {
    
    Robot[] newRbts = new Robot[Rbts.length]; 
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
      StepData[gen] = bestTime;
      println("bestTime:", bestTime);
      println("Best Sequence:");
      boolean reached = false;
      ArrayList<String> cmds = Rbts[bestRobot].brain.Cmds;
      int cmdidx = 0;
      while(!reached && cmdidx < cmds.size()){
        print (cmds.get(cmdidx) + " ");
        if (cmds.get(cmdidx) == "S"){
          reached = true;
        }
        if ((cmdidx+1)%50 == 0){
          print("\n");
        }
        cmdidx++;
      }
      println();
    }else{
      StepData[gen] = -1;
    }
   
    fitnessSum = 0;
    for (int i = 0; i< Rbts.length; i++) {
      fitnessSum += Rbts[i].fitness;
    }
   
    newRbts[0] = Rbts[bestRobot].gimmeBaby();
    newRbts[0].isBest = true;
    for (int i = 1; i< newRbts.length; i++) {
      Robot parent = selectParent();
      newRbts[i] = parent.gimmeBaby();
    }
    Rbts = newRbts.clone();
    gen ++;
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
      for (int cmdidx = 0; cmdidx < bestTime; cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess){
          int randMorph = floor(random(7));
          Rbts[i].morph = randMorph;
          Rbts[i].brain.Cmds.set(cmdidx,str(randMorph));
        }else if (rand < MutMoveRate) {
          int randomDist = (int)random(2);
          int randInt = (int)random(4);
          while (randomDist >= 0 && cmdidx+randomDist < Rbts[i].brain.Cmds.size()){
            Rbts[i].brain.Cmds.set(cmdidx+randomDist, fourDirString[randInt]);
            randomDist -= 1;
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
          childRbt.brain.Cmds.set(cmdidx, Rbts[bestRobot].brain.Cmds.get(cmdidx));
        }
        for (int cmdidx = cutPoint; cmdidx < childRbt.brain.Cmds.size(); cmdidx++){
          childRbt.brain.Cmds.set(cmdidx, parents[1].brain.Cmds.get(cmdidx));
        }
        Rbts[i].brain = childRbt.brain.clone();
      }
    }
  }
  
  void GARemoveTwoDir() {
    for (int i = 1; i< Rbts.length; i++) {
      if (random(1) < baseMutRemoveDirRate){
        ArrayList<String> Cmds = Rbts[i].brain.Cmds;
        int cmdSize = Cmds.size();
        boolean validSelect = false;
        int searchCount = 0;
        while (!validSelect && searchCount < 1000){
          searchCount++;
          int idx1 = floor(random(cmdSize));
          int idx2 = floor(random(cmdSize));
          
          String str1 = Cmds.get(idx1);
          String str2 = Cmds.get(idx2);
          
          if (idx1 != idx2){
            if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||
                (str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
                if(idx1 > idx2){
                  Cmds.remove(idx1);
                  Cmds.remove(idx2);
                  validSelect = true;
                }else{
                  Cmds.remove(idx2);
                  Cmds.remove(idx1);
                  validSelect = true;
                }
            }
          }
        }
      }
    }
  }
  
  
  void GARemoveExtraShapes() {
    for (int i = 1; i< Rbts.length; i++) {
      if (random(1) < baseMutRemoveShapeRate){
        ArrayList<String> Cmds = Rbts[i].brain.Cmds;
        int cmdSize = Cmds.size();
        boolean validSelect = false;
        int searchCount = 0;
        while (!validSelect && searchCount < 1000){
          searchCount++;
          int idx = floor(random(cmdSize));
          int inMorph = int(Cmds.get(idx));
          if (inMorph != 0){
            Cmds.remove(idx);
          }
        }
      }
    }
  }
}