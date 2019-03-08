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
  ArrayList<Robot> unsortRobotListCopy;
  ArrayList<ArrayList<Robot>> Front; 
  ArrayList<ArrayList<Robot>> cdFront;
  int maxFront = 2;
  
  int bestCostRbtID;
  int bestSmoothRbtID;
  int bestSafeRbtID;
  int bestSumRbtID;
  float bestCost;
  float bestSmooth;
  float bestSafe;
  float bestSum;
  float bigNum = 99999;
  
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
    unsortRobotListCopy = new ArrayList<Robot>();
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
        //Rbts[i].brain.curCost = tempCostScore;
        //Rbts[i].brain.curSmooth = tempSmoothScore;
        //Rbts[i].brain.curSafe = tempSafeScore;
        
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
        Rbts[i].costScore = tempFitness;
        Rbts[i].smoothScore =  1/bigNum;
        Rbts[i].safeScore = 1/bigNum;
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
  }

  void nsga2(){

    int requiredLength = (int)Rbts.length;
    int countLength = 0;
    // Merging the two generations
    
    unsortRobotList = parentRobotList;
    unsortRobotList.addAll(childRobotList);
    
    for (int rbtidx = 0; rbtidx < unsortRobotList.size(); rbtidx++){
      unsortRobotList.get(rbtidx).popID = rbtidx;
    }
    
    unsortRobotListCopy = (ArrayList) unsortRobotList.clone();
    
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
          curGARobot.frontRank = frontCount;
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
    /*
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      println("FRONT " + frontidx);
      for (int cmpidx = 0; cmpidx < Front.get(frontidx).size(); cmpidx++){
        Robot theBot = Front.get(frontidx).get(cmpidx);
        print("(" + nf(theBot.costScore,2,4) + "," + nf(theBot.smoothScore,2,4) + "," + nf(theBot.safeScore,2,4) + ") ");
      }
      println();
      println("-----------------------------------");
    }
    */
    
    // Update parent robot list
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      parentRobotList = new ArrayList<Robot>();
      parentRobotList.addAll(Front.get(frontidx));
    }
    
    float costDiff;
    float smoothDiff;
    float safeDiff;
    
    ArrayList<Robot> ar1 = new ArrayList<Robot>();
    ArrayList<Robot> ar2 = new ArrayList<Robot>();
    ArrayList<Robot> ar3 = new ArrayList<Robot>();
    
    bestCostRbtID = 0;
    bestSmoothRbtID = 0;
    bestSafeRbtID = 0;
    bestSumRbtID = 0;
    bestCost = bigNum;
    bestSmooth = bigNum;
    bestSafe = bigNum;
    bestSum = bigNum;
      
    for (int frontidx = 0; frontidx < frontCount; frontidx++){
      
      costDiff = 0;
      smoothDiff = 0;
      safeDiff = 0;
      
      ar1 = (ArrayList)Front.get(frontidx).clone();
      ar2 = (ArrayList)Front.get(frontidx).clone();
      ar3 = (ArrayList)Front.get(frontidx).clone();
      
      Collections.sort(ar1, new sortByCost()); 
      Collections.sort(ar2, new sortBySmooth()); 
      Collections.sort(ar3, new sortBySafe()); 
      
      costDiff = ar1.get((int)(ar1.size()-1)).costScore - ar1.get(0).costScore; 
      smoothDiff = ar2.get((int)(ar1.size()-1)).smoothScore - ar2.get(0).smoothScore; 
      safeDiff = ar3.get((int)(ar1.size()-1)).safeScore - ar3.get(0).safeScore; 
      println("(costDiff, smoothDiff, safeDiff) = " + costDiff + ", " + smoothDiff +", " + safeDiff);
      
      costDiff = (costDiff == 0)? 1/bigNum : costDiff;
      smoothDiff = (smoothDiff == 0)? 1/bigNum : smoothDiff;
      safeDiff = (safeDiff == 0)? 1/bigNum : safeDiff;
      
      for (int rbtidx = 0; rbtidx < Front.get(frontidx).size(); rbtidx++){
        //println("arisize " +  ar1.size() + "  popid" +   Front.get(frontidx).get(rbtidx).popID);
        for (int aridx = 0; aridx < ar1.size(); aridx++){
          //println("ar1.get(aridx).popID  " + ar1.get(aridx).popID);
          if (aridx == 0 || aridx == ar1.size()-1){
            if (ar1.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).costCrowdDist = bigNum;
            }
          }else{
            if (ar1.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).costCrowdDist = (float)(ar1.get(aridx+1).costScore-ar1.get(aridx-1).costScore)/costDiff;
            }
          }
        }
        
        for (int aridx = 0; aridx < ar2.size(); aridx++){
          if (aridx == 0 || aridx == ar2.size()-1){
            if (ar2.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).smoothCrowdDist = bigNum;
            }
          }else{
            if (ar2.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).smoothCrowdDist = (float)(ar2.get(aridx+1).smoothScore-ar2.get(aridx-1).smoothScore)/smoothDiff;
            }
          }
        }
        
        for (int aridx = 0; aridx < ar3.size(); aridx++){
          if (aridx == 0 || aridx == ar3.size()-1){
            if (ar3.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).safeCrowdDist = bigNum;
            }
          }else{
            if (ar3.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
              Front.get(frontidx).get(rbtidx).safeCrowdDist = (float)(ar3.get(aridx+1).safeScore-ar3.get(aridx-1).safeScore)/safeDiff;
            }
          }
        }
        
        Front.get(frontidx).get(rbtidx).sumCrowdDist = Front.get(frontidx).get(rbtidx).costCrowdDist + Front.get(frontidx).get(rbtidx).smoothCrowdDist + 
                                                       Front.get(frontidx).get(rbtidx).safeCrowdDist;
        
        /*
        println("Front rbt " + rbtidx + " (ID,Rk,c,sm,sf,sum)  = (" + Front.get(frontidx).get(rbtidx).popID + ", " +
                                                              Front.get(frontidx).get(rbtidx).frontRank + ", " +
                                                              Front.get(frontidx).get(rbtidx).costCrowdDist + ", " + 
                                                              Front.get(frontidx).get(rbtidx).smoothCrowdDist + ", "+
                                                              Front.get(frontidx).get(rbtidx).safeCrowdDist + ", " +
                                                              Front.get(frontidx).get(rbtidx).sumCrowdDist + ") ("+
                                                              Front.get(frontidx).get(rbtidx).costScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).smoothScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).safeScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).sumScore + ") ");                                               
        */
         println("Front rbt " + rbtidx + " (c,sm,sf,sum)  = (" + Front.get(frontidx).get(rbtidx).popID + ", " +
                                                              Front.get(frontidx).get(rbtidx).frontRank + ")( " +
                                                              Front.get(frontidx).get(rbtidx).costScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).smoothScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).safeScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).sumScore + ") "); 
        
        if (frontidx == 0){
          if (Front.get(frontidx).get(rbtidx).costCrowdDist < bestCost && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestCostRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).smoothCrowdDist < bestSmooth && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSmoothRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).safeCrowdDist < bestSafe && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSafeRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).sumCrowdDist < bestSum  && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSumRbtID = Front.get(frontidx).get(rbtidx).popID;
          }

        }
      }
    }
    println("=====================");
    println("bestSumID: " + bestSumRbtID + "; bestSafeRbtID: " + bestSafeRbtID +  "; bestSmoothRbtID: " + bestSmoothRbtID + "; bestCostRbtID: " + bestCostRbtID );
    println("=====================");

  }
  
  
  void displayList(){
    //for (int frontidx = 0; frontidx < unsortRobotList.size(); frontidx++){
    //  Robot theBot = unsortRobotList.get(frontidx);
    //  print("(" +theBot.costScore + "," + theBot.smoothScore + "," + theBot.safeScore + ")  ");
    //}
    //println();
  }


  boolean naturalSelection() {
    
    Robot[] newRbts = new Robot[Rbts.length]; 
    
    getPopInfo();
    nsga2();
    /*
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
    */
    
        // Data Table Recording!!
    TableRow newRow = popTable.addRow();
    newRow.setInt("genID", gen);
    newRow.setInt("reachGoal", 0);
    
    newRow.setFloat("bC-ID", bestCostRbtID);
    if (unsortRobotListCopy.get(bestCostRbtID).reachedGoal){
      newRow.setFloat("bC-C", unsortRobotListCopy.get(bestCostRbtID).costCrowdDist);
      newRow.setFloat("bC-Sm", unsortRobotListCopy.get(bestCostRbtID).smoothCrowdDist);
      newRow.setFloat("bC-Sf", unsortRobotListCopy.get(bestCostRbtID).safeCrowdDist);
      newRow.setFloat("bC-A", unsortRobotListCopy.get(bestCostRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bSm-ID", bestSmoothRbtID);
    if (unsortRobotListCopy.get(bestSmoothRbtID).reachedGoal){
      newRow.setFloat("bSm-C", unsortRobotListCopy.get(bestSmoothRbtID).costCrowdDist);
      newRow.setFloat("bSm-Sm", unsortRobotListCopy.get(bestSmoothRbtID).smoothCrowdDist);
      newRow.setFloat("bSm-Sf", unsortRobotListCopy.get(bestSmoothRbtID).safeCrowdDist);
      newRow.setFloat("bSm-A", unsortRobotListCopy.get(bestSmoothRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bSf-ID", bestSafeRbtID);
    if (unsortRobotListCopy.get(bestSafeRbtID).reachedGoal){
      newRow.setFloat("bSf-C", unsortRobotListCopy.get(bestSafeRbtID).costCrowdDist);
      newRow.setFloat("bSf-Sm", unsortRobotListCopy.get(bestSafeRbtID).smoothCrowdDist);
      newRow.setFloat("bSf-Sf", unsortRobotListCopy.get(bestSafeRbtID).safeCrowdDist);
      newRow.setFloat("bSf-A", unsortRobotListCopy.get(bestSafeRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bA-ID", bestSumRbtID);
    if (unsortRobotListCopy.get(bestSumRbtID).reachedGoal){
      newRow.setFloat("bA-C", unsortRobotListCopy.get(bestSumRbtID).costCrowdDist);
      newRow.setFloat("bA-Sm", unsortRobotListCopy.get(bestSumRbtID).smoothCrowdDist);
      newRow.setFloat("bA-Sf", unsortRobotListCopy.get(bestSumRbtID).safeCrowdDist); 
      newRow.setFloat("bA-A", unsortRobotListCopy.get(bestSumRbtID).sumCrowdDist); 
    }
    
    
    if (Rbts[bestRobot].reachedGoal) {
      bestTime = Rbts[bestRobot].brain.curTime;
      StepData[gen] = bestTime;          ///>>>>>???
      
      boolean reached = false;
      ArrayList<String> cmds = Rbts[bestRobot].brain.Cmds;
      
      int cmdidx = 0;
      while(!reached && cmdidx < cmds.size()){
        if (cmds.get(cmdidx) == "S"){
          reached = true;
        }
        cmdidx++;
      }
    }
   
    fitnessSum = 0;
    for (int i = 0; i< Rbts.length; i++) {
      fitnessSum += Rbts[i].fitness;
    }
   
   
    //for (int i = 0; i< newRbts.length; i++) {
    //  newRbts[i] = Rbts[i].gimmeBaby();
    //}
    //Rbts = newRbts.clone();
    
    
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

  Robot selectParentNsga() {
    int randRbt1 = floor(random(Rbts.length));
    int randRbt2 = floor(random(Rbts.length));
    float randRbt1Score = Rbts[randRbt1].sumCrowdDist;
    float randRbt2Score = Rbts[randRbt2].sumCrowdDist;
    
    //println("randRbt1Score " + randRbt1Score + " randRbt2Score " + randRbt2Score);
    if (Rbts[randRbt1].frontRank < Rbts[randRbt2].frontRank){
      
      return Rbts[randRbt1];
    }else if (Rbts[randRbt1].frontRank > Rbts[randRbt2].frontRank){
      return Rbts[randRbt2];
    }else{
      return randRbt1Score < randRbt2Score? Rbts[randRbt1]: Rbts[randRbt2];
    }
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
