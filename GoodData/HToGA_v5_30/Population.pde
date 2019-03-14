class Population {
  
  Robot[] prevGoodRbts;
  Robot[] Rbts;
  Robot[] newRbts;
  Robot[] newMutRbts;
  Robot[] newSwapRbts;
  Robot[] newPercRbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;
  int genSize;

  //GARobot bestRobot;//the index of the best dot in the dots[]
  int bestRobot = 0;

  float[] StepDataCD;
  float[] StepData;
  int bestTime;
  float bestFitness;
  
  ArrayList<Robot> parentRobotList;
  ArrayList<Robot> childRobotList;
  ArrayList<Robot> unsortRobotList;
  ArrayList<Robot> unsortRobotListCopy;
  ArrayList<ArrayList<Robot>> Front; 
  ArrayList<ArrayList<Robot>> cdFront;
  int maxFront = 100;
  
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
  
  boolean nonReached;
  float nrBestCost;
  int nrBestCostRbtID;
  
  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    bestTime = minStep_;
    genSize = size;
    Rbts = new Robot[genSize*5];
    for (int i = 0; i < genSize*5; i++) {
      Rbts[i] = new Robot();
    }
    StepData = new float[minStep_];
    StepDataCD = new float[minStep_];
    for (int i = 0; i < StepData.length; i++) {
      StepData[i] = -1;
      StepDataCD[i] = -1;
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
      }
      if (!curRbt.dead){   
        if (!curRbt.reachedGoal){
          if (dist(curRbt.Blks[1].pos.x, curRbt.Blks[1].pos.y, map.Wps[currentWpID+1].pos.x, map.Wps[currentWpID+1].pos.y) < 20) {
            curRbt.reachedGoal = true;
            for(int cmdidx = curRbt.brain.Cmds.size()-1; cmdidx > curRbt.brain.curTime; cmdidx --){
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
      float tempSmoothScore = 0;
      float rbtCmdSize = Rbts[i].brain.Cmds.size();
      
      if (Rbts[i].reachedGoal) {
        tempFitness = 1+100.0/(float)(rbtCmdSize); //REMOVE?
        
      } else {
        int[] gridPos = Rbts[i].Blks[1].blkGridPos(map.mapSize);
        tempFitness = (float)(1/(1+getGridFitnessValue(gridPos[0],gridPos[1], 2)));
        if (progressingFitness){
          if (tempFitness > Rbts[i].fitness){
            Rbts[i].fitness = tempFitness;
          }
        }else{
          Rbts[i].fitness = tempFitness;
        }
      }
      
      int disSim = 0;
      for (int cmdidx = 0; cmdidx < rbtCmdSize-1; cmdidx++){
        if (Rbts[i].brain.Cmds.get(cmdidx) != Rbts[i].brain.Cmds.get(cmdidx+1)){
          disSim ++;
        }
      }
      tempSmoothScore = rbtCmdSize/(float)(rbtCmdSize+disSim); 
      Rbts[i].costScore = Rbts[i].fitness;
      Rbts[i].smoothScore = tempSmoothScore;
      Rbts[i].sumScore = Rbts[i].costScore + Rbts[i].smoothScore + Rbts[i].safeScore;
    }
  }
  
  void calculateFinalFitness() {
    for (int i = 0; i< Rbts.length; i++) {
      float tempFitness = 0;
      float tempSmoothScore = 0;
      float rbtCmdSize = Rbts[i].brain.Cmds.size();
      
      if (Rbts[i].reachedGoal) {
        tempFitness = 100.0/(float)(rbtCmdSize); //REMOVE?
        
      } else {
        int[] gridPos = Rbts[i].Blks[1].blkGridPos(map.mapSize);
        tempFitness = (float)(1/(1+getGridFitnessValue(gridPos[0],gridPos[1], 2)));
        if (progressingFitness){
          if (tempFitness > Rbts[i].fitness){
            Rbts[i].fitness = tempFitness;
          }
        }else{
          Rbts[i].fitness = tempFitness;
        }
      }
      
      int disSim = 0;
      for (int cmdidx = 0; cmdidx < rbtCmdSize-1; cmdidx++){
        if (Rbts[i].brain.Cmds.get(cmdidx) != Rbts[i].brain.Cmds.get(cmdidx+1)){
          disSim ++;
        }
      }
      tempSmoothScore = rbtCmdSize/(float)(rbtCmdSize+disSim); 
      Rbts[i].costScore = tempFitness;
      Rbts[i].smoothScore = tempSmoothScore;
      Rbts[i].safeScore = rbtCmdSize/(float)(rbtCmdSize+Rbts[i].safeScore);
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
      //float genDiff = (StepData[gen-1-minConvergeSample] - StepData[gen-1]);
      //if (dispText) println("generation resutls difference:  " + genDiff);
      //if (StepData[gen-1-minConvergeSample] != -1 && abs(StepData[gen-1-minConvergeSample] - StepData[gen-1]) <= maxConvergeDiff){
      //  return true;
      //}
      if (StepData[gen-1-minConvergeSample] != -1 &&
          StepData[gen-1] == StepData[gen-4] && StepData[gen-1] == StepData[gen-7] && StepData[gen-1] == StepData[gen-10]){
        return true;
      }
    }
    return false;
  }


  void nsga2(){

    // copy Rbt array to nsga arraylist
    unsortRobotList = new ArrayList<Robot>();
    for (int i = 0; i< genSize*5; i++) {
      Rbts[i].popID = i;
      unsortRobotList.add(Rbts[i]);
    }
    unsortRobotListCopy = (ArrayList) unsortRobotList.clone();
    
    Front = new ArrayList<ArrayList<Robot>>(); 
    cdFront = new ArrayList<ArrayList<Robot>>(); 
    for (int frontidx = 0; frontidx < maxFront; frontidx++){
      Front.add(new ArrayList<Robot>());
      cdFront.add(new ArrayList<Robot>());
    }

    nonReached = true;
    nrBestCost = 0;
    nrBestCostRbtID = 0;
    for (int rbtidx = 0; rbtidx < unsortRobotList.size(); rbtidx++){
      Robot curGARobot = unsortRobotList.get(rbtidx);
      if (curGARobot.reachedGoal){
        nonReached = false; 
      }
      if (curGARobot.costScore > nrBestCost){
        nrBestCost = curGARobot.costScore;
        nrBestCostRbtID = curGARobot.popID;
      }
    }
    
    int requiredLength = genSize;
    // None of the robots reaches the goal, compare A* score instead
    if (nonReached){
      println("=================");
      println("NO robot reach");
      println("nrBestCostRbtID = " + nrBestCostRbtID  + ";  nrBestCost = " + nrBestCost );
      ArrayList<Robot> arCost = new ArrayList<Robot>();
      arCost = (ArrayList) unsortRobotList.clone();
      Collections.sort(arCost, new sortByCost()); 
      
      parentRobotList = new ArrayList<Robot>();
      for (int costidx = arCost.size()-1; costidx >= genSize; costidx--){
        parentRobotList.add(arCost.get(costidx));
      }
    // At least a robot reached goal, compare through multi-objective function  
    }else{
      int countLength = 0;
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
          
          if (paretoBest && curGARobot.reachedGoal){
            boolean isRepeat = false;
            if (frontCount == 0){
              for (int frontidx = 0; frontidx < Front.get(frontCount).size(); frontidx++){
                if (Front.get(frontCount).get(frontidx).costScore == curGARobot.costScore &&
                    Front.get(frontCount).get(frontidx).smoothScore == curGARobot.smoothScore &&
                    Front.get(frontCount).get(frontidx).safeScore == curGARobot.safeScore ){
                  isRepeat = true;
                  break;
                }
              }
            }
            if (!isRepeat){
              curGARobot.frontRank = frontCount;
              Front.get(frontCount).add(curGARobot);
              countLength++;
              removeList.add(rbtidx);
            }
          }
          if (countLength == requiredLength){
            break;
          }
        }
        for (int remidx = removeList.size()-1; remidx >= 0; remidx--){   
          int toRemove = removeList.get(remidx);
          unsortRobotList.remove(toRemove);
        }
        if (Front.get(frontCount).size() == 0){
          break;
        }
        frontCount++;
      }
      
      
      //Update parent robot list
      parentRobotList = new ArrayList<Robot>();
      for (int frontidx = 0; frontidx < frontCount; frontidx++){
        for (int rbtidx = 0; rbtidx < Front.get(frontidx).size(); rbtidx++){
          boolean isRepeat = false;
          for (int listidx = 0; listidx < parentRobotList.size(); listidx++){
            if (parentRobotList.get(listidx).costScore == Front.get(frontidx).get(rbtidx).costScore){
              isRepeat = true;
            }
          }
          if (!isRepeat){
            parentRobotList.add(Front.get(frontidx).get(rbtidx));
          }
        }
      }
      
      float costDiff;
      float smoothDiff;
      float safeDiff;
      
      ArrayList<Robot> ar1 = new ArrayList<Robot>();
      ArrayList<Robot> ar2 = new ArrayList<Robot>();
      ArrayList<Robot> ar3 = new ArrayList<Robot>();
      
      bestCostCD = bigNum;
      bestCostCDRbtID = 0;
      bestSmoothCD = bigNum;
      bestSmoothCDRbtID = 0;            
      bestSafeCD = bigNum;
      bestSafeCDRbtID = 0;           
      bestSumCD = bigNum;
      bestSumCDRbtID = 0;
      bestCost = 0;
      bestCostRbtID = 0;
      bestSmooth = 0;
      bestSmoothRbtID = 0;            
      bestSafe = 0;
      bestSafeRbtID = 0;           
      bestSum = 0;
      bestSumRbtID = 0;
      
      boolean notInit = true;
      
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
        //println("(costDiff, smoothDiff, safeDiff) = " + costDiff + ", " + smoothDiff +", " + safeDiff);
        
        costDiff = (costDiff == 0)? 1/bigNum : costDiff;
        smoothDiff = (smoothDiff == 0)? 1/bigNum : smoothDiff;
        safeDiff = (safeDiff == 0)? 1/bigNum : safeDiff;
        
        for (int rbtidx = 0; rbtidx < Front.get(frontidx).size(); rbtidx++){
          for (int aridx = 0; aridx < ar1.size(); aridx++){
            if (aridx == 0 || aridx == ar1.size()-1){
              if (ar1.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).costCD = bigNum;
              }
            }else{
              if (ar1.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).costCD = (float)(ar1.get(aridx+1).costScore-ar1.get(aridx-1).costScore)/costDiff;
              }
            }
          }
          
          for (int aridx = 0; aridx < ar2.size(); aridx++){
            if (aridx == 0 || aridx == ar2.size()-1){
              if (ar2.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).smoothCD = bigNum;
              }
            }else{
              if (ar2.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).smoothCD = (float)(ar2.get(aridx+1).smoothScore-ar2.get(aridx-1).smoothScore)/smoothDiff;
              }
            }
          }
          
          for (int aridx = 0; aridx < ar3.size(); aridx++){
            if (aridx == 0 || aridx == ar3.size()-1){
              if (ar3.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).safeCD = bigNum;
              }
            }else{
              if (ar3.get(aridx).popID == Front.get(frontidx).get(rbtidx).popID){
                Front.get(frontidx).get(rbtidx).safeCD = (float)(ar3.get(aridx+1).safeScore-ar3.get(aridx-1).safeScore)/safeDiff;
              }
            }
          }
          
          Front.get(frontidx).get(rbtidx).sumCD = Front.get(frontidx).get(rbtidx).costCD + Front.get(frontidx).get(rbtidx).smoothCD + 
                                                         Front.get(frontidx).get(rbtidx).safeCD;
          

         
          if (frontidx == 0){
            if (notInit){
              notInit = false;
              bestCostCD = Front.get(0).get(0).costCD;
              bestCostCDRbtID = Front.get(0).get(0).popID;
              bestSmoothCD = Front.get(0).get(0).smoothCD;
              bestSmoothCDRbtID = Front.get(0).get(0).popID;            
              bestSafeCD = Front.get(0).get(0).safeCD;
              bestSafeCDRbtID = Front.get(0).get(0).popID;           
              bestSumCD = Front.get(0).get(0).sumCD;
              bestSumCDRbtID = Front.get(0).get(0).popID;
              bestCost = Front.get(0).get(0).costScore;
              bestCostRbtID = Front.get(0).get(0).popID;
              bestSmooth = Front.get(0).get(0).smoothScore;
              bestSmoothRbtID = Front.get(0).get(0).popID;            
              bestSafe = Front.get(0).get(0).safeScore;
              bestSafeRbtID = Front.get(0).get(0).popID;           
              bestSum = Front.get(0).get(0).sumScore;
              bestSumRbtID = Front.get(0).get(0).popID;
            }
            
            println("Front rbt " + rbtidx + " (ID,Rk,c,sm,sf,sum)  = (" + Front.get(frontidx).get(rbtidx).popID + ", " +
                                                      Front.get(frontidx).get(rbtidx).frontRank + ", " +
                                                      Front.get(frontidx).get(rbtidx).costCD + ", " + 
                                                      Front.get(frontidx).get(rbtidx).smoothCD + ", "+
                                                      Front.get(frontidx).get(rbtidx).safeCD + ", " +
                                                      Front.get(frontidx).get(rbtidx).sumCD + ") ("+
                                                      Front.get(frontidx).get(rbtidx).costScore + ", " +
                                                      Front.get(frontidx).get(rbtidx).smoothScore + ", " +
                                                      Front.get(frontidx).get(rbtidx).safeScore + ", " +
                                                      Front.get(frontidx).get(rbtidx).sumScore + ") ");    
            print("RobotID " + nf(Front.get(frontidx).get(rbtidx).popID,3) + " score: " + nf(Front.get(frontidx).get(rbtidx).costScore,2,2) + " cmds: " );
            String toWrite;
            toWrite = "";
            for (int cmdidx = 0; cmdidx < Front.get(frontidx).get(rbtidx).brain.Cmds.size(); cmdidx++){
              toWrite += Front.get(frontidx).get(rbtidx).brain.Cmds.get(cmdidx);
            }
            println(toWrite);
            
            
            println("(" + bestCostCD + "," + bestSmoothCD + "," + bestSafeCD + "," + bestSumCD + ")");
            if (Front.get(0).get(rbtidx).costCD < bestCostCD){ // && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestCostCD = Front.get(0).get(rbtidx).costCD;
              bestCostCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).smoothCD < bestSmoothCD){ // && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSmoothCD = Front.get(0).get(rbtidx).smoothCD;
              bestSmoothCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).safeCD < bestSafeCD){ // && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSafeCD = Front.get(0).get(rbtidx).safeCD;
              bestSafeCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).sumCD < bestSumCD ){ //  && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSumCD = Front.get(0).get(rbtidx).sumCD;
              bestSumCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            
            if (Front.get(0).get(rbtidx).costScore > bestCost){
              bestCost = Front.get(0).get(rbtidx).costScore;
              bestCostRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).smoothScore > bestSmooth){
              bestSmooth = Front.get(frontidx).get(rbtidx).smoothScore;
              bestSmoothRbtID = Front.get(frontidx).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).safeScore > bestSafe){
              bestSafe = Front.get(frontidx).get(rbtidx).safeScore;
              bestSafeRbtID = Front.get(frontidx).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).sumScore > bestSum){
              bestSum = Front.get(0).get(rbtidx).sumScore;
              bestSumRbtID = Front.get(0).get(rbtidx).popID;
            }
          }
        }
      }
      
      println("=====================");
      println("bestSumCDRbtID: " + bestSumCDRbtID + "; bestSafeCDRbtID: " + bestSafeCDRbtID +  "; bestSmoothCDRbtID: " + bestSmoothCDRbtID + "; bestCostCDRbtID: " + bestCostCDRbtID );
      println("bestSumRbtID: " + bestSumRbtID + "; bestSafeRbtID: " + bestSafeRbtID +  "; bestSmoothRbtID: " + bestSmoothRbtID + "; bestCostRbtID: " + bestCostRbtID );
      println("bestSumCDRbtScore: " + unsortRobotListCopy.get(bestSumCDRbtID).sumScore + " ; bestSumRbtScore:" + unsortRobotListCopy.get(bestSumRbtID).sumScore);        
      println("=====================");
      
      ArrayList<Robot> arCost = new ArrayList<Robot>();
      arCost = (ArrayList) unsortRobotListCopy.clone();
      Collections.sort(arCost, new sortByCost()); 
      
      //parentRobotList = new ArrayList<Robot>();
      while (parentRobotList.size() < genSize){
        Robot parentRbt = selectParent(unsortRobotListCopy);
        parentRobotList.add(parentRbt);
      }
      
    }
  }
  
  boolean naturalSelection() {
    
    if (!nonReached){
      if (unsortRobotListCopy.get(bestSumCDRbtID).reachedGoal) {
        StepData[gen] = unsortRobotListCopy.get(bestSumRbtID).sumScore;        
      }
    }
    
    newRbts = new Robot[genSize];
    newMutRbts = new Robot[genSize];
    newSwapRbts = new Robot[genSize];
    newPercRbts = new Robot[genSize];
    //newRbts[0] = unsortRobotList.get(nrBestCostRbtID).gimmeBaby();
    for  (int i = 0; i < newRbts.length; i++) {
      Robot parent = selectParent(parentRobotList);
      newRbts[i] = (Robot) parent.gimmeBaby();
      newMutRbts[i] = (Robot) parent.gimmeBaby();
      newSwapRbts[i] = (Robot) parent.gimmeBaby();
      newPercRbts[i] = (Robot) parent.gimmeBaby();
      
    }  
      
    if (gen < StepData.length){
      gen++;
      return false;
    }
    return true;
  }

  void MergeRobots(){
    Rbts = new Robot[genSize*5];
    for (int i = 0; i < genSize; i++) {
      Rbts[i] = (Robot) parentRobotList.get(i).gimmeBaby();
    }
    for (int i = genSize; i < genSize*2; i++) {
      Rbts[i] = (Robot) newRbts[i-genSize].gimmeBaby();
    }  
    for (int i = genSize*2; i < genSize*3; i++) {
      Rbts[i] = (Robot) newMutRbts[i-genSize*2].gimmeBaby();
    }  
    for (int i = genSize*3; i < genSize*4; i++) {
      Rbts[i] = (Robot) newSwapRbts[i-genSize*3].gimmeBaby();
    }  
    for (int i = genSize*4; i < genSize*5; i++) {
      Rbts[i] = (Robot) newPercRbts[i-genSize*4].gimmeBaby();
      Rbts[i].isPerc = true;
    }  
  }

  //-------------------------------------------------------------------------------------------------------------------------------------

  //chooses dot from the population to return randomly(considering fitness)
  Robot selectParent(ArrayList<Robot> parentList) {
    fitnessSum = 0;
    for (int i = 0; i< parentList.size(); i++) {
      fitnessSum += parentList.get(i).fitness;
    }
    float rand = random(fitnessSum);
    float runningSum = 0;
    for (int i = 0; i< parentList.size(); i++) {
      runningSum += parentList.get(i).fitness;
      if (runningSum > rand) {
        return parentList.get(i);
      }
    }
    return null;
  }

  Robot selectParentNsga() {
    println("parent size" + parentRobotList.size());
    int randRbt1 = floor(random(parentRobotList.size()));
    int randRbt2 = floor(random(parentRobotList.size()));
    float randRbt1Score = parentRobotList.get(randRbt1).sumCD;
    float randRbt2Score = parentRobotList.get(randRbt2).sumCD;
    
    //println("randRbt1Score " + randRbt1Score + " randRbt2Score " + randRbt2Score);
    if (parentRobotList.get(randRbt1).frontRank < Rbts[randRbt2].frontRank){
      
      return parentRobotList.get(randRbt1);
    }else if (parentRobotList.get(randRbt1).frontRank > parentRobotList.get(randRbt2).frontRank){
      return parentRobotList.get(randRbt2);
    }else{
      return randRbt1Score < randRbt2Score? parentRobotList.get(randRbt1): parentRobotList.get(randRbt2);
    }
  }
  
  Robot[] tournamentSelect(){
    Robot[] winRbts = new Robot[2];
    int[] randRbtID = new int[4];
    for (int rbtidx = 0; rbtidx < 4; rbtidx++){
      randRbtID[rbtidx] = floor(random(newRbts.length));
    }
    winRbts[0] = (newRbts[randRbtID[0]].fitness > newRbts[randRbtID[1]].fitness)? newRbts[randRbtID[0]] : newRbts[randRbtID[1]];
    winRbts[1] = (newRbts[randRbtID[2]].fitness > newRbts[randRbtID[3]].fitness)? newRbts[randRbtID[2]] : newRbts[randRbtID[3]];
    return winRbts;
  }

  //------------------------------------------------------------------------------------------------------------------------------------------
  //mutate
  void GANonreached(){
    if (!nonReached){
      return;
    }
    for (int i = 0; i < newSwapRbts.length; i++) {
      for (int cmdidx = 0; cmdidx < newSwapRbts[i].brain.Cmds.size(); cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess && cmdidx!= 0 && cmdidx != newSwapRbts[i].brain.Cmds.size()-1){
          int randMorph = floor(random(morphNum));
          newSwapRbts[i].morph = randMorph;
          newSwapRbts[i].brain.Cmds.set(cmdidx,str(randMorph));
        }else if (rand < MutMoveRate) {
          int randomDist = (int)random(2);
          int randInt = (int)random(4);
          while (randomDist >= 0 && cmdidx+randomDist < newSwapRbts[i].brain.Cmds.size()){
            newSwapRbts[i].brain.Cmds.set(cmdidx+randomDist, fourDirString[randInt]);
            randomDist -= 1;
          }
        }
      }
    }
    for (int i = 0; i < newRbts.length; i++) {
      Robot parent = selectParent(parentRobotList);
      newRbts[i] = parent.gimmeBaby();
    }
  }
  
  void GAMutation() {
    for (int i = 0; i < newMutRbts.length; i++) {
      for (int cmdidx = 0; cmdidx < newMutRbts[i].brain.Cmds.size(); cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess && cmdidx!= 0 && cmdidx != newMutRbts[i].brain.Cmds.size()-1){
          int randMorph = floor(random(morphNum));
          newMutRbts[i].morph = randMorph;
          newMutRbts[i].brain.Cmds.set(cmdidx,str(randMorph));
        }else if (rand < MutMoveRate) {
          int randomDist = (int)random(2);
          int randInt = (int)random(4);
          while (randomDist >= 0 && cmdidx+randomDist < newMutRbts[i].brain.Cmds.size()){
            newMutRbts[i].brain.Cmds.set(cmdidx+randomDist, fourDirString[randInt]);
            randomDist -= 1;
          }
        }
      }
    }
  }
  float baseSwapRate = 0.01;
  
  void GASwap() {
    if (nonReached){
      return;
    }
    for (int i = 0; i < newSwapRbts.length; i++) {
      for (int cmdidx = 2; cmdidx < newSwapRbts[i].brain.Cmds.size()-2; cmdidx++) {
        float rand = random(1);
        int randInt = floor(random(4));
        int randCmdidx = cmdidx + randInt - 2;
        if (rand < baseSwapRate){
          String tempCmd = newMutRbts[i].brain.Cmds.get(cmdidx);
          newSwapRbts[i].brain.Cmds.set(cmdidx,newSwapRbts[i].brain.Cmds.get(randCmdidx));
          newSwapRbts[i].brain.Cmds.set(randCmdidx,tempCmd);
        }
      }
    }
  }
  void GACrossover() {
    for (int i = 0; i< newMutRbts.length; i++) {
      if (random(1) < baseCrossoverRate){
        Robot[] parents = tournamentSelect();
        Robot childRbt = parents[0];
        int cutPoint = floor(random(bestTime));
        for (int cmdidx = 0; cmdidx < cutPoint; cmdidx++){
          //childRbt.brain.Cmds[cmdidx] = parents[0].brain.Cmds[cmdidx];
          childRbt.brain.Cmds.set(cmdidx, newRbts[bestRobot].brain.Cmds.get(cmdidx));
        }
        for (int cmdidx = cutPoint; cmdidx < childRbt.brain.Cmds.size(); cmdidx++){
          childRbt.brain.Cmds.set(cmdidx, parents[1].brain.Cmds.get(cmdidx));
        }
        newMutRbts[i].brain = childRbt.brain.clone();
      }
    }
  }
  
  void GARemoveTwoDir() {
    if (nonReached){
      return;
    }
    for (int i = 0; i< newRbts.length; i++) {
      int cmdSize = newRbts[i].brain.Cmds.size();
      int maxToRemove = floor(baseMutRemoveDirRate*cmdSize);
      int numToRemove = floor(random(maxToRemove));
      int count = 0;
      while(numToRemove!= 0 && count < 200){
        cmdSize = newRbts[i].brain.Cmds.size();
        count++;
        int idx1 = floor(random(cmdSize));
        int idx2 = floor(random(cmdSize));
        String str1 = newRbts[i].brain.Cmds.get(idx1);
        String str2 = newRbts[i].brain.Cmds.get(idx2);
        if (idx1 != idx2){
          if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||(str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
            if(idx1 > idx2){
              newRbts[i].brain.Cmds.remove(idx1);
              newRbts[i].brain.Cmds.remove(idx2);
              numToRemove--;
            }else{
              newRbts[i].brain.Cmds.remove(idx2);
              newRbts[i].brain.Cmds.remove(idx1);
              numToRemove--;
            }
          }
        }
      }
    }
    if (forceRemove){
      for (int i = 0; i< newRbts.length; i++) {
        int count = 0;
        boolean noAdjust = false;
        while (!noAdjust && count < 500){
          count++;
          noAdjust = true;
          int cmdSize = newRbts[i].brain.Cmds.size();
          for (int cmdidx = cmdSize-1; cmdidx >= 1; cmdidx-=1){
            String str1 = newRbts[i].brain.Cmds.get(cmdidx);
            String str2 = newRbts[i].brain.Cmds.get(cmdidx-1);
            if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||(str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
              newRbts[i].brain.Cmds.remove(cmdidx);
              newRbts[i].brain.Cmds.remove(cmdidx-1);
              noAdjust = false;
              break;
            }
          }
        }
      }
      
    }
  }
  
  void GARemoveExtraShapes() {
    if (nonReached){
      return;
    }
    for (int i = 0; i< newRbts.length; i++) {
      ArrayList<String> Cmds = newRbts[i].brain.Cmds;
      int cmdSize = Cmds.size();
      //println("Cmds.get(cmdidx) "+cmdSize);
      for (int cmdidx = cmdSize-1; cmdidx >= 0; cmdidx--){
         int inMorph = int(Cmds.get(cmdidx));
         if (inMorph != 0 || Cmds.get(cmdidx).equals("0")){
           if (random(1) < baseMutRemoveShapeRate){
             newRbts[i].brain.Cmds.remove(cmdidx);
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
  
  void nsga2RecordData(){
    // Data Table Recording!!
    String toWrite;
    TableRow newRow = popTable.addRow();
    
    newRow.setInt("genID", gen);
    toWrite = nonReached? "F" : "T";
    newRow.setString("reachGoal", toWrite);
    
    if (!nonReached){
      newRow.setFloat("bC-ID", bestCostRbtID);
      newRow.setFloat("bC-C", unsortRobotListCopy.get(bestCostRbtID).costScore);
      newRow.setFloat("bC-Sm", unsortRobotListCopy.get(bestCostRbtID).smoothScore);
      newRow.setFloat("bC-Sf", unsortRobotListCopy.get(bestCostRbtID).safeScore);
      newRow.setFloat("bC-A", unsortRobotListCopy.get(bestCostRbtID).sumScore);
      
      newRow.setFloat("bSm-ID", bestSmoothRbtID);
      newRow.setFloat("bSm-C", unsortRobotListCopy.get(bestSmoothRbtID).costScore);
      newRow.setFloat("bSm-Sm", unsortRobotListCopy.get(bestSmoothRbtID).smoothScore);
      newRow.setFloat("bSm-Sf", unsortRobotListCopy.get(bestSmoothRbtID).safeScore);
      newRow.setFloat("bSm-A", unsortRobotListCopy.get(bestSmoothRbtID).sumScore);
      
      newRow.setFloat("bSf-ID", bestSafeRbtID);
      newRow.setFloat("bSf-C", unsortRobotListCopy.get(bestSafeRbtID).costScore);
      newRow.setFloat("bSf-Sm", unsortRobotListCopy.get(bestSafeRbtID).smoothScore);
      newRow.setFloat("bSf-Sf", unsortRobotListCopy.get(bestSafeRbtID).safeScore);
      newRow.setFloat("bSf-A", unsortRobotListCopy.get(bestSafeRbtID).sumScore);
      
      newRow.setFloat("bA-ID", bestSumRbtID);
      newRow.setFloat("bA-C", unsortRobotListCopy.get(bestSumRbtID).costScore);
      newRow.setFloat("bA-Sm", unsortRobotListCopy.get(bestSumRbtID).smoothScore);
      newRow.setFloat("bA-Sf", unsortRobotListCopy.get(bestSumRbtID).safeScore); 
      newRow.setFloat("bA-A", unsortRobotListCopy.get(bestSumRbtID).sumScore); 
        
      newRow.setFloat("bCCD-ID", bestCostCDRbtID);
      newRow.setFloat("bCCD-C", unsortRobotListCopy.get(bestCostCDRbtID).costCD);
      newRow.setFloat("bCCD-Sm", unsortRobotListCopy.get(bestCostCDRbtID).smoothCD);
      newRow.setFloat("bCCD-Sf", unsortRobotListCopy.get(bestCostCDRbtID).safeCD);
      newRow.setFloat("bCCD-A", unsortRobotListCopy.get(bestCostCDRbtID).sumCD);
      
      newRow.setFloat("bSmCD-ID", bestSmoothCDRbtID);
      newRow.setFloat("bSmCD-C", unsortRobotListCopy.get(bestSmoothCDRbtID).costCD);
      newRow.setFloat("bSmCD-Sm", unsortRobotListCopy.get(bestSmoothCDRbtID).smoothCD);
      newRow.setFloat("bSmCD-Sf", unsortRobotListCopy.get(bestSmoothCDRbtID).safeCD);
      newRow.setFloat("bSmCD-A", unsortRobotListCopy.get(bestSmoothCDRbtID).sumCD);
      
      newRow.setFloat("bSfCD-ID", bestSafeCDRbtID);
      newRow.setFloat("bSfCD-C", unsortRobotListCopy.get(bestSafeCDRbtID).costCD);
      newRow.setFloat("bSfCD-Sm", unsortRobotListCopy.get(bestSafeCDRbtID).smoothCD);
      newRow.setFloat("bSfCD-Sf", unsortRobotListCopy.get(bestSafeCDRbtID).safeCD);
      newRow.setFloat("bSfCD-A", unsortRobotListCopy.get(bestSafeCDRbtID).sumCD);
      
      newRow.setFloat("bACD-ID", bestSumCDRbtID);
      newRow.setFloat("bACD-C", unsortRobotListCopy.get(bestSumCDRbtID).costCD);
      newRow.setFloat("bACD-Sm", unsortRobotListCopy.get(bestSumCDRbtID).smoothCD);
      newRow.setFloat("bACD-Sf", unsortRobotListCopy.get(bestSumCDRbtID).safeCD); 
      newRow.setFloat("bACD-A", unsortRobotListCopy.get(bestSumCDRbtID).sumCD);   
        
        
      toWrite = "";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestCostCDRbtID).brain.Cmds.size(); cmdidx++){
        toWrite += unsortRobotListCopy.get(bestCostCDRbtID).brain.Cmds.get(cmdidx);
      }
      newRow.setString("Cmd-bCCD", toWrite);
      
      toWrite = "";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSmoothCDRbtID).brain.Cmds.size(); cmdidx++){
        toWrite += unsortRobotListCopy.get(bestSmoothCDRbtID).brain.Cmds.get(cmdidx);
      }
      newRow.setString("Cmd-bSmCD", toWrite);
      
      toWrite = "";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSafeCDRbtID).brain.Cmds.size(); cmdidx++){
        toWrite += unsortRobotListCopy.get(bestSafeCDRbtID).brain.Cmds.get(cmdidx);
      }
      newRow.setString("Cmd-bSfCD", toWrite);
  
      toWrite = "";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSumCDRbtID).brain.Cmds.size(); cmdidx++){
        toWrite += unsortRobotListCopy.get(bestSumCDRbtID).brain.Cmds.get(cmdidx);
      }
      newRow.setString("Cmd-bACD", toWrite);
      
      newRow.setFloat("f_gr", 1);
      newRow.setFloat("f_t", 1/(float)unsortRobotListCopy.get(bestSumCDRbtID).brain.Cmds.size());
      newRow.setFloat("f_sm", unsortRobotListCopy.get(bestSumCDRbtID).smoothScore);
      newRow.setFloat("f_sf", unsortRobotListCopy.get(bestSumCDRbtID).safeScore);
    }
    else{
      newRow.setInt("bf-ID", nrBestCostRbtID);
      newRow.setFloat("bf-C", unsortRobotListCopy.get(nrBestCostRbtID).costScore);
      newRow.setFloat("bf-Sm", unsortRobotListCopy.get(nrBestCostRbtID).smoothScore);
      newRow.setFloat("bf-Sf", unsortRobotListCopy.get(nrBestCostRbtID).safeScore);
      newRow.setFloat("bf-A", unsortRobotListCopy.get(nrBestCostRbtID).sumScore);
      
      toWrite = "";
      for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(nrBestCostRbtID).brain.Cmds.size(); cmdidx++){
        toWrite += unsortRobotListCopy.get(nrBestCostRbtID).brain.Cmds.get(cmdidx);
      }
      newRow.setString("Cmd-bf", toWrite);
      
      newRow.setFloat("f_gr", 1/(1+pow(1/unsortRobotListCopy.get(nrBestCostRbtID).costScore, 0.5)));
      newRow.setFloat("f_t", 1/(float)unsortRobotListCopy.get(nrBestCostRbtID).brain.Cmds.size());
      newRow.setFloat("f_sm", unsortRobotListCopy.get(nrBestCostRbtID).smoothScore);
      newRow.setFloat("f_sf", unsortRobotListCopy.get(nrBestCostRbtID).safeScore);
    }
  }
}
