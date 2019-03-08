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
  int maxFront = 15;
  
  int bestCostCDRbtID;
  int bestSmoothCDRbtID;
  int bestSafeCDRbtID;
  int bestSumCDRbtID;
  
  int bestCostRbtID;
  int bestSmoothRbtID;
  int bestSafeRbtID;
  int bestSumRbtID;
  
  float bestCostCD;
  float bestSmoothCD;
  float bestSafeCD;
  float bestSumCD;
  
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
        //if (curRbt.brain.Cmds.size() <= time){
        //  curRbt.dead = true;
        //}
        
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
        
        int disSim = 0;
        for (int cmdidx = 0; cmdidx < Rbts[i].brain.Cmds.size()-1; cmdidx++){
          if (Rbts[i].brain.Cmds.get(cmdidx) == Rbts[i].brain.Cmds.get(cmdidx+1)){
            disSim ++;
          }
        }
        tempSmoothScore = disSim/(float)Rbts[i].brain.Cmds.size(); //(float)Rbts[i].brain.Cmds.size();
        //tempSafeScore = 1;
        
        Rbts[i].fitness = tempFitness;
        //Rbts[i].brain.curCost = tempCostScore;
        //Rbts[i].brain.curSmooth = tempSmoothScore;
        //Rbts[i].brain.curSafe = tempSafeScore;
        
        Rbts[i].costScore = tempFitness;
        Rbts[i].smoothScore = tempSmoothScore;
        //Rbts[i].safeScore = 1; //tempFitness*tempSmoothScore;
        
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
        //Rbts[i].safeScore = 1/bigNum;
      }
      Rbts[i].sumScore = Rbts[i].costScore + Rbts[i].smoothScore + Rbts[i].safeScore;
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
    
    bestCostCDRbtID = 0;
    bestSmoothCDRbtID = 0;
    bestSafeCDRbtID = 0;
    bestSumCDRbtID = 0;
    
    bestCostCD = bigNum;
    bestSmoothCD = bigNum;
    bestSafeCD = bigNum;
    bestSumCD = bigNum;
    
    bestCost = 0;
    bestSmooth = 0;
    bestSafe = 0;
    bestSum = 0;
    
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
        /*
         println("Front rbt " + rbtidx + " (c,sm,sf,sum)  = (" + Front.get(frontidx).get(rbtidx).popID + ", " +
                                                              Front.get(frontidx).get(rbtidx).frontRank + ")( " +
                                                              Front.get(frontidx).get(rbtidx).costScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).smoothScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).safeScore + ", " +
                                                              Front.get(frontidx).get(rbtidx).sumScore + ") "); 
        */
        
        if (frontidx == 0){
          if (Front.get(frontidx).get(rbtidx).costCrowdDist < bestCostCD && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestCostCD = Front.get(frontidx).get(rbtidx).costCrowdDist;
            bestCostCDRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).smoothCrowdDist < bestSmoothCD && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSmoothCD = Front.get(frontidx).get(rbtidx).smoothCrowdDist;
            bestSmoothCDRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).safeCrowdDist < bestSafeCD && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSafeCD = Front.get(frontidx).get(rbtidx).safeCrowdDist;
            bestSafeCDRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).sumCrowdDist < bestSumCD  && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSumCD = Front.get(frontidx).get(rbtidx).sumCrowdDist;
            bestSumCDRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          
          if (Front.get(frontidx).get(rbtidx).costScore > bestCost && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestCost = Front.get(frontidx).get(rbtidx).costScore;
            bestCostRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).smoothScore > bestSmooth && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSmooth = Front.get(frontidx).get(rbtidx).smoothScore;
            bestSmoothRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).safeScore > bestSafe && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSafe = Front.get(frontidx).get(rbtidx).safeScore;
            bestSafeRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
          if (Front.get(frontidx).get(rbtidx).sumScore > bestSum  && Front.get(frontidx).get(rbtidx).sumCrowdDist < bigNum){
            bestSum = Front.get(frontidx).get(rbtidx).sumScore;
            bestSumRbtID = Front.get(frontidx).get(rbtidx).popID;
          }
        }
      }
    }
    println("=====================");
    println("bestSumCDRbtID: " + bestSumCDRbtID + "; bestSafeCDRbtID: " + bestSafeCDRbtID +  "; bestSmoothCDRbtID: " + bestSmoothCDRbtID + "; bestCostCDRbtID: " + bestCostCDRbtID );
    println("bestSumRbtID: " + bestSumRbtID + "; bestSafeRbtID: " + bestSafeRbtID +  "; bestSmoothRbtID: " + bestSmoothRbtID + "; bestCostRbtID: " + bestCostRbtID );
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
    
    // Data Table Recording!!
    TableRow newRow = popTable.addRow();
    newRow.setInt("genID", gen);
    newRow.setInt("reachGoal", 0);
    
    newRow.setFloat("bC-ID", bestCostCDRbtID);
    if (unsortRobotListCopy.get(bestCostCDRbtID).reachedGoal){
      newRow.setFloat("bC-C", unsortRobotListCopy.get(bestCostCDRbtID).costCrowdDist);
      newRow.setFloat("bC-Sm", unsortRobotListCopy.get(bestCostCDRbtID).smoothCrowdDist);
      newRow.setFloat("bC-Sf", unsortRobotListCopy.get(bestCostCDRbtID).safeCrowdDist);
      newRow.setFloat("bC-A", unsortRobotListCopy.get(bestCostCDRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bSm-ID", bestSmoothCDRbtID);
    if (unsortRobotListCopy.get(bestSmoothCDRbtID).reachedGoal){
      newRow.setFloat("bSm-C", unsortRobotListCopy.get(bestSmoothCDRbtID).costCrowdDist);
      newRow.setFloat("bSm-Sm", unsortRobotListCopy.get(bestSmoothCDRbtID).smoothCrowdDist);
      newRow.setFloat("bSm-Sf", unsortRobotListCopy.get(bestSmoothCDRbtID).safeCrowdDist);
      newRow.setFloat("bSm-A", unsortRobotListCopy.get(bestSmoothCDRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bSf-ID", bestSafeCDRbtID);
    if (unsortRobotListCopy.get(bestSafeCDRbtID).reachedGoal){
      newRow.setFloat("bSf-C", unsortRobotListCopy.get(bestSafeCDRbtID).costCrowdDist);
      newRow.setFloat("bSf-Sm", unsortRobotListCopy.get(bestSafeCDRbtID).smoothCrowdDist);
      newRow.setFloat("bSf-Sf", unsortRobotListCopy.get(bestSafeCDRbtID).safeCrowdDist);
      newRow.setFloat("bSf-A", unsortRobotListCopy.get(bestSafeCDRbtID).sumCrowdDist);
    }
    
    newRow.setFloat("bA-ID", bestSumCDRbtID);
    if (unsortRobotListCopy.get(bestSumCDRbtID).reachedGoal){
      newRow.setFloat("bA-C", unsortRobotListCopy.get(bestSumCDRbtID).costCrowdDist);
      newRow.setFloat("bA-Sm", unsortRobotListCopy.get(bestSumCDRbtID).smoothCrowdDist);
      newRow.setFloat("bA-Sf", unsortRobotListCopy.get(bestSumCDRbtID).safeCrowdDist); 
      newRow.setFloat("bA-A", unsortRobotListCopy.get(bestSumCDRbtID).sumCrowdDist); 
    }
    
    
    String toWrite;
    
    toWrite = "BestCostRbt: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestCostRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestCostRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    toWrite = "BestSmoothRbt: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSmoothRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestSmoothRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    toWrite = "BestSafeRbt: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSafeRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestSafeRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);

    toWrite = "BestSumRbt: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSumRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestSumRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    

    
    toWrite = "BestSumCDRbt: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSumCDRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestSumCDRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    
    
    
    
    
    
    
    //if (Rbts[bestRobot].reachedGoal) {
    if (unsortRobotListCopy.get(bestSumCDRbtID).reachedGoal) {
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
    
    newRbts[0] = unsortRobotListCopy.get(bestSumCDRbtID).gimmeBaby();
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
