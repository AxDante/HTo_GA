class Population {
  
  Robot[] prevGoodRbts;
  Robot[] Rbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;

  //GARobot bestRobot;//the index of the best dot in the dots[]
  int bestRobot = 0;

  int[] StepData;
  int bestTime;
  float bestFitness;
  
  ArrayList<Robot> parentRobotList;
  ArrayList<Robot> childRobotList;
  ArrayList<Robot> unsortRobotList;
  ArrayList<ArrayList<Robot>> Front; 
  ArrayList<ArrayList<Robot>> cdFront;
  int maxFront = 2;
  
  
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
    Front = new ArrayList<ArrayList<Robot>>();
    cdFront = new ArrayList<ArrayList<Robot>>();
    parentRobotList = new ArrayList<Robot>();
    childRobotList = new ArrayList<Robot>();
    unsortRobotList = new ArrayList<Robot>();
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
        
        if (curRbt.brain.curTime >= maxTime){
          curRbt.dead = true;
        } 
        if (isCollideRbt(Rbts[rbtidx])){
          curRbt.dead = true;
        } 
        //What is this?
        if (curRbt.brain.Cmds.size() <= time){
          curRbt.dead = true;
        }
        
      }
      
      if (!curRbt.dead){   
        if (!curRbt.reachedGoal){
          if (dist(curRbt.Blks[1].pos.x, curRbt.Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y) < 25) {
            curRbt.reachedGoal = true;
            for(int cmdidx = curRbt.brain.Cmds.size()-1; cmdidx > bestTime; cmdidx --){
              curRbt.brain.Cmds.remove(cmdidx);
            }
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
      float tempCostScore = 0;
      float tempSmoothScore = 0;
      float tempSafeScore = 0;
      
      if (Rbts[i].reachedGoal) {
        
        tempFitness = 10 + 10000.0/(float)(Rbts[i].brain.curTime * Rbts[i].brain.curTime); //REMOVE?
        tempCostScore = (float)Rbts[i].brain.curTime;
        tempSafeScore = (float)Rbts[i].brain.curSafe + 0.1;
        tempSmoothScore = (float)Rbts[i].brain.curSmooth + 0.1;
        
        Rbts[i].fitness = tempFitness;
        Rbts[i].brain.curCost = tempCostScore;
        Rbts[i].brain.curSmooth = tempSmoothScore;
        Rbts[i].brain.curSafe = tempSafeScore;
        
        Rbts[i].costScore = tempCostScore;
        Rbts[i].smoothScore = tempSmoothScore;
        Rbts[i].safeScore = tempSafeScore;
        
      } else {
        int[] gridPos = Rbts[i].Blks[1].blkGridPos(map.mapSize);
        tempFitness = (float)(10/(1+getGridFitnessValue(gridPos[0],gridPos[1], 4)));
        if (progressingFitness){
          if (tempFitness > Rbts[i].fitness){
            Rbts[i].fitness = tempFitness;
          }
        }else{
          Rbts[i].fitness = tempFitness;
        }
        Rbts[i].smoothScore = tempFitness;  // remove this!
      }
      //println("Robot " + nf(i,4) + " | ReachGoal: " + Rbts[i].reachedGoal + "  curTime: " + nf(Rbts[i].brain.curTime,3) + "  Fitness: " + nf(Rbts[i].fitness,1,7));
    }
  }
  
  void resetFitness() {
    for (int i = 0; i< Rbts.length; i++) {
      Rbts[i].fitness = 0;
      Rbts[i].costScore = 0;
      Rbts[i].smoothScore = 0;
      Rbts[i].safeScore = 0;
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
      if (dispText) println("generation resutls difference:  " + genDiff);
      if (StepData[gen-1-minConvergeSample] != -1 && (StepData[gen-1-minConvergeSample] - StepData[gen-1]) <= maxConvergeDiff){
        return true;
      }
    }
    return false;
  }

  void getPopInfo(){
    
    childRobotList = new ArrayList<Robot>();
    for (int i = 0; i< Rbts.length; i++) {
      childRobotList.add(Rbts[i]);
    }
    println(childRobotList.size());
  }

  void nsga2(){

    int requiredLength = (int)Rbts.length;
    int countLength = 0;
    // Merging the two generations
    
    unsortRobotList = parentRobotList;
    unsortRobotList.addAll(childRobotList);
    
    Front = new ArrayList<ArrayList<Robot>>(); 
    cdFront = new ArrayList<ArrayList<Robot>>(); 
    for (int frontidx = 0; frontidx < maxFront; frontidx++){
      Front.add(new ArrayList<Robot>());
      cdFront.add(new ArrayList<Robot>());
    }
    
    println("REQ LENGTH" + requiredLength);
    int frontCount = 0;
    while(countLength < requiredLength){
      ArrayList<Integer> removeList = new ArrayList<Integer>();
      for (int rbtidx = 0; rbtidx < unsortRobotList.size(); rbtidx++){
        Robot curGARobot = unsortRobotList.get(rbtidx);
        boolean paretoBest = true;
        for (int rbtidj = 0; rbtidj < unsortRobotList.size(); rbtidj++){
          Robot cmpGARobot = unsortRobotList.get(rbtidj);
          if (curGARobot.costScore < cmpGARobot.costScore
           && curGARobot.smoothScore < cmpGARobot.smoothScore
           && curGARobot.safeScore < cmpGARobot.safeScore){
            paretoBest = false;
          }
        }
        if (paretoBest){
          Front.get(frontCount).add(curGARobot);
          countLength++;
          removeList.add(rbtidx);
        }
        if (countLength == requiredLength){
          break;
        }
      }
      //displayList();
      for (int remidx = removeList.size()-1; remidx >= 0; remidx--){   
        int toRemove = removeList.get(remidx);
        unsortRobotList.remove(toRemove);
      }
      //displayList();
      frontCount++;
    }
    
    // For DEBUG purpose
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      println("FRONT " + frontidx);
      for (int cmpidx = 0; cmpidx < Front.get(frontidx).size(); cmpidx++){
        Robot theBot = Front.get(frontidx).get(cmpidx);
        print("(" + nf(theBot.costScore,2,4) + "," + nf(theBot.smoothScore,2,4) + "," + nf(theBot.safeScore,2,4) + ") ");
      }
      println();
      println("-----------------------------------");
    }
    
    // Update parent robot list
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      parentRobotList = new ArrayList<Robot>();
      parentRobotList.addAll(Front.get(frontidx));
    }
    
    ArrayList<Robot> ar1 = new ArrayList<Robot>();
    ArrayList<Robot> ar2 = new ArrayList<Robot>();
    ArrayList<Robot> ar3 = new ArrayList<Robot>();
    Robot r0 = new Robot();
    r0.setScore(0, 2, 8, 1);
    Robot r1 = new Robot();
    r1.setScore(1, 2, 6, 3);
    Robot r2 = new Robot();
    r2.setScore(2, 5, 5, 1);
    Robot r3 = new Robot();
    r3.setScore(3, 6, 3, 4);
    Robot r4 = new Robot();
    r4.setScore(4, 10, 2, 1);
    Robot r5 = new Robot();
    r5.setScore(5, 8, 2, 8);
    Robot r6 = new Robot();
    r6.setScore(6, 1, 1, 7);   
    
    ar1.add(r0);
    ar1.add(r1);
    ar1.add(r2);
    ar1.add(r3);
    ar1.add(r4);
    ar1.add(r5);
    ar1.add(r6);
    
    ar2.add(r0);
    ar2.add(r1);
    ar2.add(r2);
    ar2.add(r3);
    ar2.add(r4);
    ar2.add(r5);
    ar2.add(r6);
    
    ar3.add(r0);
    ar3.add(r1);
    ar3.add(r2);
    ar3.add(r3);
    ar3.add(r4);
    ar3.add(r5);
    ar3.add(r6);
    
    float costDiff = 0;
    float smoothDiff = 0;
    float safeDiff = 0;
    
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      Collections.sort(ar1, new sortByCost()); 
      Collections.sort(ar2, new sortBySmooth()); 
      Collections.sort(ar3, new sortBySafe()); 
      costDiff = ar1.get((int)(ar1.size()-1)).costScore - ar1.get(0).costScore; 
      smoothDiff = ar2.get((int)(ar1.size()-1)).smoothScore - ar2.get(0).smoothScore; 
      safeDiff = ar3.get((int)(ar1.size()-1)).safeScore - ar3.get(0).safeScore; 
      println("(costDiff, smoothDiff, safeDiff) = " + costDiff + ", " + smoothDiff +", " + safeDiff);
    }
    
    ArrayList<ArrayList<Integer>> costCrowdDist = new ArrayList<ArrayList<Integer>>();
    ArrayList<ArrayList<Integer>> smoothcrowdDist = new ArrayList<ArrayList<Integer>>();
    ArrayList<ArrayList<Integer>> safeCrowdDist = new ArrayList<ArrayList<Integer>>();
    
    ArrayList<Float> frontCostCrowdDist = new ArrayList<Float>();
    ArrayList<Float> frontSmoothCrowdDist = new ArrayList<Float>();
    ArrayList<Float> frontSafeCrowdDist = new ArrayList<Float>();
    ArrayList<Float> sumCrowdDist = new ArrayList<Float>();
    
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      sumCrowdDist.add(0.0);
    }
    
    
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      if (rbtidx == 0 || rbtidx == ar1.size()-1){
        frontCostCrowdDist.add(99999.0f);
        frontSmoothCrowdDist.add(99999.0f);
        frontSafeCrowdDist.add(99999.0f);
      }
      else{
        frontCostCrowdDist.add((float)(ar1.get(rbtidx+1).costScore-ar1.get(rbtidx-1).costScore)/costDiff);
        frontSmoothCrowdDist.add((float)(ar2.get(rbtidx+1).smoothScore-ar2.get(rbtidx-1).smoothScore)/smoothDiff);
        frontSafeCrowdDist.add((float)(ar3.get(rbtidx+1).safeScore-ar3.get(rbtidx-1).safeScore)/safeDiff);
      }
      
      sumCrowdDist.set(ar1.get(rbtidx).scoreID, sumCrowdDist.get(ar1.get(rbtidx).scoreID)+frontCostCrowdDist.get(rbtidx));
      sumCrowdDist.set(ar2.get(rbtidx).scoreID, sumCrowdDist.get(ar2.get(rbtidx).scoreID)+frontSmoothCrowdDist.get(rbtidx));
      sumCrowdDist.set(ar3.get(rbtidx).scoreID, sumCrowdDist.get(ar3.get(rbtidx).scoreID)+frontSafeCrowdDist.get(rbtidx));

      //sumCostCrowdDist.add(frontCostCrowdDist.get(rbtidx) + smoothCostCrowdDist.get(rbtidx) + safeCostCrowdDist.get(rbtidx));
    }
    
    print("SumScores: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(sumCrowdDist.get(rbtidx) + ", ");
    }
    
    print("costScores: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(frontCostCrowdDist.get(rbtidx) + ", ");
    }
    println();
    
    print("smoothScores: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(frontSmoothCrowdDist.get(rbtidx) + ", ");
    }
    println();
    
    print("safeScores: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(frontSafeCrowdDist.get(rbtidx) + ", ");
    }
    println();
    /*
    println("frontcount: " + frontCount);
    print("Cost: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(ar1.get(rbtidx).scoreID + ", ");
    }
    println();
    
    print("Smooth: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(ar2.get(rbtidx).scoreID + ", ");
    }
    println();
    
    print("Safe: ");
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
      print(ar3.get(rbtidx).scoreID + ", ");
    }
    println();
    */
    
    
    for (int rbtidx = 0; rbtidx < ar1.size(); rbtidx++){
    }
    
    
    
    // Sorting Fronts through crowding distances
    /*
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      
      ArrayList<Robot> costSortFront =  (ArrayList<Robot>) Front.get(frontidx).clone();
      ArrayList<Robot> smoothSortFront =  (ArrayList<Robot>) Front.get(frontidx).clone();
      ArrayList<Robot> safeSortFront =  (ArrayList<Robot>) Front.get(frontidx).clone();
      
      Collections.sort(costSortFront, new sortByCost()); 
      Collections.sort(smoothSortFront, new sortBySmooth()); 
      Collections.sort(safeSortFront, new sortBySafe()); 
    }
    */
  }
  
  
  void displayList(){
    for (int frontidx = 0; frontidx < unsortRobotList.size(); frontidx++){
      Robot theBot = unsortRobotList.get(frontidx);
      print("(" +theBot.costScore + "," + theBot.smoothScore + "," + theBot.safeScore + ")  ");
    }
    println();
  }


  boolean naturalSelection() {
    
    Robot[] newRbts = new Robot[Rbts.length]; 
    
    getPopInfo();
    nsga2();
    

    //bestRobot = maxIndex;
    //if (dispText)println("BestRobot  " + bestRobot  + "  rg: " + Rbts[bestRobot].reachedGoal + " fns: " + Rbts[bestRobot].fitness );
    
    // Data Table Recording!!
    TableRow newRow = popTable.addRow();
    newRow.setInt("genID", gen);
    newRow.setInt("bestTime", bestTime);
    newRow.setFloat("fitness", Rbts[bestRobot].fitness);
    newRow.setFloat("costScore", Rbts[bestRobot].costScore);
    newRow.setFloat("smoothScore", Rbts[bestRobot].smoothScore);
    newRow.setFloat("safeScore", Rbts[bestRobot].safeScore);
    newRow.setInt("bestID", bestRobot);
    
    String toWrite = "";
    for (int frontidx = 0; frontidx < Front.size(); frontidx ++){
      for (int arridx = 0; arridx < Front.get(frontidx).size(); arridx++){
        toWrite += Front.get(frontidx).get(arridx) + " ";
      }
      newRow.setString("F" + frontidx, toWrite);
    }
    
    fitnessTableRow.setFloat(str(gen), Rbts[bestRobot].fitness);
    bestTimeTableRow.setInt(str(gen), bestTime);
   
    
    
    if (Rbts[bestRobot].reachedGoal) {
      bestTime = Rbts[bestRobot].brain.curTime;
      StepData[gen] = bestTime;          ///>>>>>???
      
      if (dispText) println("bestTime:", bestTime);
      if (dispText) println("Best Sequence:");
      boolean reached = false;
      ArrayList<String> cmds = Rbts[bestRobot].brain.Cmds;
      
      int cmdidx = 0;
      String seqStr = "";
      while(!reached && cmdidx < cmds.size()){
        seqStr = seqStr + cmds.get(cmdidx);
        if (dispText) print (cmds.get(cmdidx) + " ");
        if (cmds.get(cmdidx) == "S"){
          reached = true;
          }
        if ((cmdidx+1)%50 == 0){
          if (dispText) print("\n");
        }
        cmdidx++;
      }
      if (dispText) println();
      
      newRow.setString("reachGoal", "Y");
      newRow.setString("bestGene", seqStr);
      
    }else{
      newRow.setString("reachGoal", "F");
      newRow.setString("bestGene", "n/a");
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
    if (gen < StepData.length){
      gen++;
      return false;
    }
    return true;
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
    for (int i = 1; i < Rbts.length; i++) {
      for (int cmdidx = 0; cmdidx < Rbts[i].brain.Cmds.size(); cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess && cmdidx!= 0 && cmdidx != Rbts[i].brain.Cmds.size()-1){
          int randMorph = floor(random(morphNum));
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
      int cmdSize = Rbts[i].brain.Cmds.size();
      int maxToRemove = floor(baseMutRemoveDirRate*cmdSize);
      int numToRemove = floor(random(maxToRemove));
      int count = 0;
      while(numToRemove!= 0 && count < 200){
        cmdSize = Rbts[i].brain.Cmds.size();
        count++;
        int idx1 = floor(random(cmdSize));
        int idx2 = floor(random(cmdSize));
        String str1 = Rbts[i].brain.Cmds.get(idx1);
        String str2 = Rbts[i].brain.Cmds.get(idx2);
        if (idx1 != idx2){
          if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||(str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
            if(idx1 > idx2){
              Rbts[i].brain.Cmds.remove(idx1);
              Rbts[i].brain.Cmds.remove(idx2);
              numToRemove--;
            }else{
              Rbts[i].brain.Cmds.remove(idx2);
              Rbts[i].brain.Cmds.remove(idx1);
              numToRemove--;
            }
          }
        }
      }
    }
    if (forceRemove){
      boolean noAdjust = false;
      while (!noAdjust){
        noAdjust = true;
        int cmdSize = Rbts[0].brain.Cmds.size();
        for (int cmdidx = cmdSize-1; cmdidx >= 1; cmdidx--){
          String str1 = Rbts[0].brain.Cmds.get(cmdidx);
          String str2 = Rbts[0].brain.Cmds.get(cmdidx-1);
          if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||(str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
            Rbts[0].brain.Cmds.remove(cmdidx);
            Rbts[0].brain.Cmds.remove(cmdidx-1);
            noAdjust = false;
            break;
          }
        }
      }
    }
  }
  
  void GARemoveExtraShapes() {
    for (int i = 1; i< Rbts.length; i++) {
      ArrayList<String> Cmds = Rbts[i].brain.Cmds;
      int cmdSize = Cmds.size();
      for (int cmdidx = cmdSize-1; cmdidx >= 0; cmdidx--){
         int inMorph = int(Cmds.get(cmdidx));

         if (inMorph != 0){
           if (random(1) < baseMutRemoveShapeRate && Rbts[i].fitness > 10){
             Rbts[i].brain.Cmds.remove(cmdidx);
           } 
         }
      }
    }
  }
  
  void clearReachHistory(){
    for (int i = 1; i< Rbts.length; i++) {
      Rbts[i].reachedGoal = false; 
    }
  }
}
