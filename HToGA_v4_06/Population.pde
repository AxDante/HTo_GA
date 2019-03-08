class Population {
  
  //Robot[] prevGoodRbts;
  //Robot[] Rbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;

  //GARobot bestRobot;//the index of the best dot in the dots[]
  int bestRobot = 0;
  float bigNum = 99999;
  
  int populationSize;

  int[] StepData;
  int bestTime;
  float bestFitness;
  
  ArrayList<Robot> parentRobotList;
  ArrayList<Robot> childRobotList;
  ArrayList<Robot> unsortRobotList;
  ArrayList<Robot> unsortRobotListCopy;
  ArrayList<Robot> RobotList;
  ArrayList<ArrayList<Robot>> Front; 
  ArrayList<ArrayList<Robot>> cdFront;
  int maxFront = 3;
  
  int bestCostRbtID;
  int bestSmoothRbtID;
  int bestSafeRbtID;
  int bestSumRbtID;
  float bestCost;
  float bestSmooth;
  float bestSafe;
  float bestSum;
  
  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    bestTime = minStep_;
    StepData = new int[minStep_];
    populationSize = size;
    for (int i = 0; i < StepData.length; i++) {
      StepData[i] = -1;
    }
    RobotList = new ArrayList<Robot>();
    parentRobotList = new ArrayList<Robot>();
    for (int i = 0; i < size; i++) {
      RobotList.add(new Robot());
    }
    parentRobotList = (ArrayList) RobotList.clone();
    
    Front = new ArrayList<ArrayList<Robot>>();
    cdFront = new ArrayList<ArrayList<Robot>>();
    childRobotList = new ArrayList<Robot>();
    unsortRobotList = new ArrayList<Robot>();
    unsortRobotListCopy = new ArrayList<Robot>();
  } 


  //------------------------------------------------------------------------------------------------------------------------------
  //show all dots
  void show() {
    for (int i = 1; i< RobotList.size(); i++) {
      RobotList.get(i).show();
    }
    //show();
  }

  //-------------------------------------------------------------------------------------------------------------------------------
  //update all Robots
  void update() {
    Robot curRbt;
    for (int rbtidx = 0; rbtidx < RobotList.size(); rbtidx++) {
      curRbt = RobotList.get(rbtidx);
      if (!curRbt.dead){
        if (curRbt.brain.curTime >= maxTime){
          curRbt.dead = true;
        } 
        if (isCollideRbt(curRbt)){
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
          curRbt.brain.pastPos[time] = curRbt.pos;
        }
      }
    }
  }
  

  //-----------------------------------------------------------------------------------------------------------------------------------
  //calculate all the fitnesses
  void calculateFitness() {
    
    for (int i = 0; i < RobotList.size(); i++) {
      
      float tempFitness = 0;
      float tempCostScore = 0;
      float tempSmoothScore = 0;
      float tempSafeScore = 0;
      
      if (RobotList.get(i).reachedGoal) {
        
        tempFitness = 10 + 10000.0/(float)(RobotList.get(i).brain.curTime * RobotList.get(i).brain.curTime); //REMOVE?
        tempCostScore = (float)RobotList.get(i).brain.curTime;
        int disSim = 0;
        for (int cmdidx = 0; cmdidx < RobotList.get(i).brain.Cmds.size()-1; cmdidx++){
          if (RobotList.get(i).brain.Cmds.get(cmdidx) == RobotList.get(i).brain.Cmds.get(cmdidx+1)){
            disSim ++;
          }
        }
        tempSmoothScore = disSim/(float)RobotList.get(i).brain.Cmds.size();
        tempSafeScore = 1;
        //tempSmoothScore = (float)Rbts[i].brain.curSmooth + 0.1;
        //tempSafeScore = (float)Rbts[i].brain.curSafe + 0.1;
        
        
        RobotList.get(i).fitness = tempFitness;
        //Rbts[i].brain.curCost = tempCostScore;
        //Rbts[i].brain.curSmooth = tempSmoothScore;
        //Rbts[i].brain.curSafe = tempSafeScore;
        
        RobotList.get(i).costScore = tempCostScore;
        RobotList.get(i).smoothScore = tempSmoothScore;
        RobotList.get(i).safeScore = tempSafeScore;
        
      } else {
        int[] gridPos = RobotList.get(i).Blks[1].blkGridPos(map.mapSize);
        tempFitness = (float)(10/(1+getGridFitnessValue(gridPos[0],gridPos[1], 4)));
        if (progressingFitness){
          if (tempFitness > RobotList.get(i).fitness){
            RobotList.get(i).fitness = tempFitness;
          }
        }else{
          RobotList.get(i).fitness = tempFitness;
        }
        RobotList.get(i).costScore = tempFitness;
        RobotList.get(i).smoothScore =  1/bigNum;
        RobotList.get(i).safeScore = 1/bigNum;
      }
      RobotList.get(i).sumScore = RobotList.get(i).costScore + RobotList.get(i).smoothScore + RobotList.get(i).safeScore;
    }
  }
  
  void resetFitness() {
    for (int i = 0; i < RobotList.size(); i++) {
      RobotList.get(i).fitness = 0;
      RobotList.get(i).costScore = 0;
      RobotList.get(i).smoothScore = 0;
      RobotList.get(i).safeScore = 0;
    }
  }


  //------------------------------------------------------------------------------------------------------------------------------------
  //returns whether all the dots are either dead or have reached the goal
  boolean allRobotsDead() {
    for (int i = 0; i< RobotList.size(); i++) {
      if (!RobotList.get(i).dead && !RobotList.get(i).reachedGoal) { 
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


  void nsga2(){

    
    
    int requiredLength = populationSize;
    int countLength = 0;
    
    // Unsorted robot list is the combination of parent and child generation
    unsortRobotList = (ArrayList)parentRobotList.clone();
    unsortRobotList.addAll((ArrayList)RobotList.clone());
    
    // Assign robot ID in the list 
    for (int rbtidx = 0; rbtidx < unsortRobotList.size(); rbtidx++){
      unsortRobotList.get(rbtidx).popID = rbtidx;
    }
    
    // Create a robot list copy
    unsortRobotListCopy = (ArrayList) unsortRobotList.clone();
    RobotList = new ArrayList<Robot>();
    
    // Create fronts
    Front = new ArrayList<ArrayList<Robot>>(); 
    cdFront = new ArrayList<ArrayList<Robot>>(); 
    for (int frontidx = 0; frontidx < maxFront; frontidx++){
      Front.add(new ArrayList<Robot>());
      cdFront.add(new ArrayList<Robot>());
    }
    
    // Peform sorting of the robot until required number of robots are found
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
          //curGARobot.popID = countLength;
          Front.get(frontCount).add(curGARobot);
          RobotList.add(curGARobot);
          countLength++;
          removeList.add(rbtidx);
        }
        if (countLength == requiredLength){
          break;
        }
      }
      for (int remidx = removeList.size()-1; remidx >= 0; remidx--){   
        int toRemove = removeList.get(remidx);
        unsortRobotList.remove(toRemove);
      }
      frontCount++;
    }
    
    // Update parent robot list
    //for (int frontidx = 0; frontidx < frontCount; frontidx++){
    //  parentRobotList = new ArrayList<Robot>();
    //  parentRobotList.addAll(Front.get(frontidx));
    //}
    
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
  
  boolean naturalSelection() {
    
    nsga2();
   
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
    
    
    String toWrite = "";
    for (int frontidx = 0; frontidx < Front.size(); frontidx ++){
      for (int arridx = 0; arridx < Front.get(frontidx).size(); arridx++){
        toWrite += Front.get(frontidx).get(arridx) + " ";
      }
      newRow.setString("F" + frontidx, toWrite);
    }
    /*
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
   */
    /*
    ArrayList<Robot> newRbts = new ArrayList<Robot>(); 

    newRbts.add(RobotList.get(bestSumRbtID).gimmeBaby());
    newRbts.get(0).isBest = true;
    for (int i = 1; i< newRbts.size(); i++) {
      Robot parent = nsga2SelectParent();
      newRbts.add(parent.gimmeBaby());
    }
    RobotList = (ArrayList) newRbts.clone();
    parentRobotList = (ArrayList) newRbts.clone();
    */
    parentRobotList = (ArrayList) RobotList.clone();
    
    /*
    String toWrite2;
    for (int rbtidx = 0; rbtidx < RobotList.size(); rbtidx ++){
      toWrite2 = "rbtID: " + rbtidx + "; cmd:  ";
      for (int cmdidx = 0; cmdidx < RobotList.get(rbtidx).brain.Cmds.size(); cmdidx++){
        toWrite2 += RobotList.get(rbtidx).brain.Cmds.get(cmdidx) ;
      }
      println(toWrite2);
    }
    */
    
    String toWrite2;
    for (int rbtidx = 0; rbtidx < unsortRobotListCopy.size(); rbtidx ++){
      toWrite2 = "rbtID: " + rbtidx + "; cmd:  ";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(rbtidx).brain.Cmds.size(); cmdidx++){
        toWrite2 += unsortRobotListCopy.get(rbtidx).brain.Cmds.get(cmdidx) ;
      }
      println(toWrite2);
    }
    
    
    if (gen < StepData.length){
      gen++;
      return false;
    }
    return true;
  }

  //-------------------------------------------------------------------------------------------------------------------------------------

  Robot nsga2SelectParent() {

    int randRbt1 = floor(random(RobotList.size()));
    int randRbt2 = floor(random(RobotList.size()));
    float randRbt1Score = RobotList.get(randRbt1).sumCrowdDist;
    float randRbt2Score = RobotList.get(randRbt2).sumCrowdDist;
    
    //println("randRbt1Score " + randRbt1Score + " randRbt2Score " + randRbt2Score);
    if (RobotList.get(randRbt1).frontRank < RobotList.get(randRbt2).frontRank){
      
      return RobotList.get(randRbt1);
    }else if (RobotList.get(randRbt1).frontRank > RobotList.get(randRbt2).frontRank){
      return RobotList.get(randRbt2);
    }else{
      return randRbt1Score < randRbt2Score? RobotList.get(randRbt1): RobotList.get(randRbt2);
    }
  }

  /*
  Robot[] tournamentSelect(){
    Robot[] winRbts = new Robot[2];
    int[] randRbtID = new int[4];
    for (int rbtidx = 0; rbtidx < 4; rbtidx++){
      randRbtID[rbtidx] = floor(random(length));
    }
    winRbts[0] = (Rbts[randRbtID[0]].fitness > Rbts[randRbtID[1]].fitness)? Rbts[randRbtID[0]] : Rbts[randRbtID[1]];
    winRbts[1] = (Rbts[randRbtID[2]].fitness > Rbts[randRbtID[3]].fitness)? Rbts[randRbtID[2]] : Rbts[randRbtID[3]];
    return winRbts;
  }
*/

  //------------------------------------------------------------------------------------------------------------------------------------------
  //mutate
  void GAMutation() {
    for (int i = 1; i < RobotList.size(); i++) {
      for (int cmdidx = 0; cmdidx < RobotList.get(i).brain.Cmds.size(); cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess && cmdidx!= 0 && cmdidx != RobotList.get(i).brain.Cmds.size()-1){
          int randMorph = floor(random(morphNum));
          RobotList.get(i).morph = randMorph;
          RobotList.get(i).brain.Cmds.set(cmdidx,str(randMorph));
          //println("changing ID: " + i + " cmdidx: " + cmdidx);
        }else if (rand < MutMoveRate) {
          int randomDist = (int)random(2);
          int randInt = (int)random(4);
          while (randomDist >= 0 && cmdidx+randomDist <RobotList.get(i).brain.Cmds.size()){
            RobotList.get(i).brain.Cmds.set(cmdidx+randomDist, fourDirString[randInt]);
            randomDist -= 1;
          }
        }
      }
    }
  }
  /*
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
  */
  
  void clearReachHistory(){
    for (int i = 1; i< RobotList.size(); i++) {
      RobotList.get(i).reachedGoal = false; 
      RobotList.get(i).dead = false; 
      RobotList.get(i).brain.curTime = 0; 
    }
  }
}
