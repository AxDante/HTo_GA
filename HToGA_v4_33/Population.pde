class Population {
  
  Robot[] prevGoodRbts;
  Robot[] Rbts;
  Robot[] newRbts;
  Obstacle[] Obss;

  float fitnessSum;
  int gen = 0;
  int genSize;

  //GARobot bestRobot;//the index of the best dot in the dots[]
  int bestRobot = 0;

  float[] StepData;
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
  
  boolean nonReached;
  float nrBestCost;
  int nrBestCostRbtID;
  
  Population(int size, int minStep_,  Obstacle[] Obss_) {
    Obss = Obss_;
    bestTime = minStep_;
    genSize = size;
    Rbts = new Robot[genSize*2];
    for (int i = 0; i < genSize*2; i++) {
      Rbts[i] = new Robot();
    }
    StepData = new float[minStep_];
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
      
      if (Rbts[i].reachedGoal) {
        float rbtCmdSize = Rbts[i].brain.Cmds.size();
        tempFitness = 10 + 10000.0/(float)(rbtCmdSize); //REMOVE?
        
        int disSim = 0;
        for (int cmdidx = 0; cmdidx < rbtCmdSize-1; cmdidx++){
          if (Rbts[i].brain.Cmds.get(cmdidx) == Rbts[i].brain.Cmds.get(cmdidx+1)){
            disSim ++;
          }
        }
        tempSmoothScore = disSim/rbtCmdSize/rbtCmdSize; //(float)Rbts[i].brain.Cmds.size();
        Rbts[i].fitness = tempFitness;
        Rbts[i].costScore = tempFitness;
        Rbts[i].smoothScore = tempSmoothScore;
        
        /*
        //println("Robot with ID " + curGARobot.popID + " reached the goal.") + " length: " + nf(curGARobot.brain.Cmds.size(),2);
        print("RobotID " + nf(Rbts[i].popID,3)  + " score: " + nf(Rbts[i].costScore,2,2) + " cmds: " );
        String toWrite;
        toWrite = "";
        for (int cmdidx = 0; cmdidx < Rbts[i].brain.Cmds.size(); cmdidx++){
          toWrite += Rbts[i].brain.Cmds.get(cmdidx);
        }
        println(toWrite);
        */
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
        Rbts[i].costScore = Rbts[i].fitness;
        Rbts[i].smoothScore =  1/bigNum;
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
      float genDiff = (StepData[gen-1-minConvergeSample] - StepData[gen-1]);
      if (dispText) println("generation resutls difference:  " + genDiff);
      if (StepData[gen-1-minConvergeSample] != -1 && (StepData[gen-1-minConvergeSample] - StepData[gen-1]) <= maxConvergeDiff){
        return true;
      }
    }
    return false;
  }


  void nsga2(){

    // copy Rbt array to nsga arraylist
    unsortRobotList = new ArrayList<Robot>();
    for (int i = 0; i< genSize*2; i++) {
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
        
        //println("Robot with ID " + curGARobot.popID + " reached the goal.") + " length: " + nf(curGARobot.brain.Cmds.size(),2);
        print("RobotID " + nf(curGARobot.popID,3)  + " score: " + nf(curGARobot.costScore,2,2) + " cmds: " );
        String toWrite;
        toWrite = "";
        for (int cmdidx = 0; cmdidx < curGARobot.brain.Cmds.size(); cmdidx++){
          toWrite += curGARobot.brain.Cmds.get(cmdidx);
        }
        println(toWrite);
        
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
            curGARobot.frontRank = frontCount;
            Front.get(frontCount).add(curGARobot);
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
        if (Front.get(frontCount).size() == 0){
          break;
        }
        frontCount++;
      }
      
      
      //Update parent robot list
      parentRobotList = new ArrayList<Robot>();
      for (int frontidx = 0; frontidx < frontCount; frontidx++){
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
          //println("arisize " +  ar1.size() + "  popid" +   Front.get(frontidx).get(rbtidx).popID);
          for (int aridx = 0; aridx < ar1.size(); aridx++){
            //println("ar1.get(aridx).popID  " + ar1.get(aridx).popID);
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
          
          
          /*
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
          */
          
           println("Front rbt " + rbtidx + " (c,sm,sf,sum)  = (" + Front.get(frontidx).get(rbtidx).popID + ", " +
                                                                Front.get(frontidx).get(rbtidx).frontRank + ")( " +
                                                                Front.get(frontidx).get(rbtidx).costScore + ", " +
                                                                Front.get(frontidx).get(rbtidx).smoothScore + ", " +
                                                                Front.get(frontidx).get(rbtidx).safeScore + ", " +
                                                                Front.get(frontidx).get(rbtidx).sumScore + ") "); 
          
          if (frontidx == 0){
            if (Front.get(0).get(rbtidx).costCD < bestCostCD && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestCostCD = Front.get(0).get(rbtidx).costCD;
              bestCostCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).smoothCD < bestSmoothCD && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSmoothCD = Front.get(0).get(rbtidx).smoothCD;
              bestSmoothCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).safeCD < bestSafeCD && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSafeCD = Front.get(0).get(rbtidx).safeCD;
              bestSafeCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).sumCD < bestSumCD  && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSumCD = Front.get(0).get(rbtidx).sumCD;
              bestSumCDRbtID = Front.get(0).get(rbtidx).popID;
            }
            
            if (Front.get(0).get(rbtidx).costScore > bestCost && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestCost = Front.get(0).get(rbtidx).costScore;
              bestCostRbtID = Front.get(0).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).smoothScore > bestSmooth && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSmooth = Front.get(frontidx).get(rbtidx).smoothScore;
              bestSmoothRbtID = Front.get(frontidx).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).safeScore > bestSafe && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSafe = Front.get(frontidx).get(rbtidx).safeScore;
              bestSafeRbtID = Front.get(frontidx).get(rbtidx).popID;
            }
            if (Front.get(0).get(rbtidx).sumScore > bestSum  && Front.get(0).get(rbtidx).sumCD < bigNum){
              bestSum = Front.get(0).get(rbtidx).sumScore;
              bestSumRbtID = Front.get(0).get(rbtidx).popID;
            }
          }
        }
      }
      //println("=====================");
      //println("bestSumCDRbtID: " + bestSumCDRbtID + "; bestSafeCDRbtID: " + bestSafeCDRbtID +  "; bestSmoothCDRbtID: " + bestSmoothCDRbtID + "; bestCostCDRbtID: " + bestCostCDRbtID );
      //println("bestSumRbtID: " + bestSumRbtID + "; bestSafeRbtID: " + bestSafeRbtID +  "; bestSmoothRbtID: " + bestSmoothRbtID + "; bestCostRbtID: " + bestCostRbtID );
     // println("=====================");
      
      ArrayList<Robot> arCost = new ArrayList<Robot>();
      arCost = (ArrayList) unsortRobotListCopy.clone();
      Collections.sort(arCost, new sortByCost()); 
      
      //parentRobotList = new ArrayList<Robot>();
      int lowerBound = genSize + parentRobotList.size();
      for (int costidx = arCost.size()-1; costidx >= lowerBound; costidx--){
        parentRobotList.add(arCost.get(costidx));
      }
      //while (parentRobotList.size() < genSize){
      //  parentRobotList.add(unsortRobotListCopy.get(bestSumRbtID));
      //}
      
      //DEBUG
      //println("IN PARENT ROBOT LIST:");
      //for (int i = 0; i< parentRobotList.size(); i++){
      //  println("ID: " + parentRobotList.get(i).popID);
      //}
    }
  }
  
  boolean naturalSelection() {
    
    nsga2();
    nsga2RecordData();
    
    //if (Rbts[bestRobot].reachedGoal) {
    //if (unsortRobotListCopy.get(bestSumRbtID).reachedGoal) {
    //  bestTime = unsortRobotListCopy.get(bestSumRbtID).brain.curTime;
    //  StepData[bestSumRbtID] = bestTime; 
    
    if (nonReached){
      fitnessSum = 0;
      for (int i = 0; i< parentRobotList.size(); i++) {
        fitnessSum += parentRobotList.get(i).fitness;
      }
      newRbts = new Robot[genSize];
      newRbts[0] = unsortRobotList.get(nrBestCostRbtID).gimmeBaby();
      for  (int i = 1; i < newRbts.length; i++) {
        Robot parent = selectParent();
        newRbts[i] = (Robot) parent.gimmeBaby();
      }  
    }
    else{
      if (unsortRobotListCopy.get(bestSumCDRbtID).reachedGoal) {
        //bestScore = unsortRobotListCopy.get(bestSumCDRbtID).brain.sumCD;
        StepData[gen] = unsortRobotListCopy.get(bestSumCDRbtID).sumCD;        
      }
      newRbts = new Robot[genSize];
      for  (int i = 0; i < newRbts.length; i++) {
        newRbts[i] = (Robot) parentRobotList.get(i).gimmeBaby();
      }  
    }

    if (gen < StepData.length){
      gen++;
      return false;
    }
    return true;
  }

  void MergeRobots(){
    Rbts = new Robot[genSize*2];
    
   /*
    if (!nonReached){
      println("----");
      println("before merge:");
      print("newRbts ID: ");
      for (int i = 0; i< newRbts.length; i++){
        print(" " + newRbts[i].popID);
      }
      println(); 
      println("----");
      print("parentRobotList ID: ");
      for (int i = 0; i< parentRobotList.size(); i++){
        print(" " + parentRobotList.get(i).popID);
      }
      println();
    }
    */
    
    String toWrite;
    toWrite = "bestSumRbtCmd: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestSumRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestSumRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    println("bestSumRbtScore  (" + unsortRobotListCopy.get(bestSumRbtID).costScore + ", " +
                                unsortRobotListCopy.get(bestSumRbtID).smoothScore + ", " +
                                unsortRobotListCopy.get(bestSumRbtID).safeScore + ", " +
                                unsortRobotListCopy.get(bestSumRbtID).sumScore + ") ");     
    
    toWrite = "bestCostRbtCmd: ";
    for (int cmdidx = 0; cmdidx < unsortRobotListCopy.get(bestCostRbtID).brain.Cmds.size(); cmdidx++){
      toWrite += unsortRobotListCopy.get(bestCostRbtID).brain.Cmds.get(cmdidx);
    }
    println(toWrite);
    println("bestCostRbtCmd  (" + unsortRobotListCopy.get(bestCostRbtID).costScore + ", " +
                                unsortRobotListCopy.get(bestCostRbtID).smoothScore + ", " +
                                unsortRobotListCopy.get(bestCostRbtID).safeScore + ", " +
                                unsortRobotListCopy.get(bestCostRbtID).sumScore + ") ");
    /*    
    if (!nonReached){
    println("BEF MUT -wwwwwwwwwwwwwwwwwww");

      for (int i=0; i< parentRobotList.size(); i++){
        toWrite = "";
        for (int cmdidx = 0; cmdidx < parentRobotList.get(i).brain.Cmds.size(); cmdidx++){
          toWrite += parentRobotList.get(i).brain.Cmds.get(cmdidx);
        }
        println(toWrite);
      }
      for (int i=0; i< newRbts.length; i++){
        toWrite = "";
        for (int cmdidx = 0; cmdidx < newRbts[i].brain.Cmds.size(); cmdidx++){
          toWrite += newRbts[i].brain.Cmds.get(cmdidx);
        }
        println(toWrite);
      }
    }
    */
    /*
    if (nonReached){
      Robot[] nrNewRbts = new Robot[genSize*2];
      nrNewRbts[0] = unsortRobotListCopy.get(nrBestCostRbtID).gimmeBaby();
      nrNewRbts[0].isBest = true;
      for (int i = 1; i< newRbts.length; i++) {
        Robot parent = selectParent();
        nrNewRbts[i] = parent.gimmeBaby();
      }
      Rbts = nrNewRbts.clone();
    }else{
      */
      
      Rbts = new Robot[genSize*2];
      for (int i = 0; i < genSize; i++) {
        Rbts[i] = (Robot) parentRobotList.get(i).gimmeBaby();
      }
      for (int i = genSize; i < genSize*2; i++) {
        Rbts[i] = (Robot) newRbts[i-genSize].gimmeBaby();
      }  
      
    //}
    /*
    if (!nonReached){
      println("AFT MUT -wwwwwwwwwwwwwwwwwww");
      for (int i=0; i< Rbts.length; i++){
        toWrite = "";
        for (int cmdidx = 0; cmdidx < Rbts[i].brain.Cmds.size(); cmdidx++){
          toWrite += Rbts[i].brain.Cmds.get(cmdidx);
        }
        println(toWrite);
      }
    }
    */
  }

  //-------------------------------------------------------------------------------------------------------------------------------------

  //chooses dot from the population to return randomly(considering fitness)
  Robot selectParent() {
    float rand = random(fitnessSum);
    float runningSum = 0;
    for (int i = 0; i< parentRobotList.size(); i++) {
      runningSum += parentRobotList.get(i).fitness;
      if (runningSum > rand) {
        return parentRobotList.get(i);
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
  void GAMutation() {
    /*
    String toWrite;
    if (!nonReached){
      println("RIGHT BEF MUT -wwwwwwwwwwwwwwwwwww");
      for (int i=0; i< newRbts.length; i++){
        toWrite = "";
        for (int cmdidx = 0; cmdidx < newRbts[i].brain.Cmds.size(); cmdidx++){
          toWrite += newRbts[i].brain.Cmds.get(cmdidx);
        }
        println(toWrite);
      }
    }*/
    for (int i = 1; i < newRbts.length; i++) {
      for (int cmdidx = 0; cmdidx < newRbts[i].brain.Cmds.size(); cmdidx++) {
        float rand = random(1);
        if (rand < baseMutTransRate && mutTransProcess && cmdidx!= 0 && cmdidx != newRbts[i].brain.Cmds.size()-1){
          int randMorph = floor(random(morphNum));
          newRbts[i].morph = randMorph;
          newRbts[i].brain.Cmds.set(cmdidx,str(randMorph));
        }else if (rand < MutMoveRate) {
          int randomDist = (int)random(2);
          int randInt = (int)random(4);
          while (randomDist >= 0 && cmdidx+randomDist < newRbts[i].brain.Cmds.size()){
            newRbts[i].brain.Cmds.set(cmdidx+randomDist, fourDirString[randInt]);
            randomDist -= 1;
          }
        }
      }
    }
    /*
    if (!nonReached){
      println("RIGHT AFT MUT -wwwwwwwwwwwwwwwwwww");
      for (int i=0; i< newRbts.length; i++){
        toWrite = "";
        for (int cmdidx = 0; cmdidx < newRbts[i].brain.Cmds.size(); cmdidx++){
          toWrite += newRbts[i].brain.Cmds.get(cmdidx);
        }
        println(toWrite);
      }
    }
    */
  }
  
  void GACrossover() {
    for (int i = 1; i< newRbts.length; i++) {
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
        newRbts[i].brain = childRbt.brain.clone();
      }
    }
  }
  void GARemoveTwoDir() {
    for (int i = 1; i< newRbts.length; i++) {
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
      if (nonReached){
        for (int i = 1; i< newRbts.length; i++) {
          int count = 0;
          boolean noAdjust = false;
          while (!noAdjust && count < 500){
            count++;
            noAdjust = true;
            int cmdSize = newRbts[0].brain.Cmds.size();
            for (int cmdidx = cmdSize-1; cmdidx >= 1; cmdidx--){
              String str1 = newRbts[i].brain.Cmds.get(cmdidx);
              String str2 = newRbts[i].brain.Cmds.get(cmdidx-1);
              if ((str1 == "F" && str2 == "B")||(str1 == "B" && str2 == "F")||(str1 == "R" && str2 == "L")||(str1 == "L" && str2 == "R")){
                newRbts[i].brain.Cmds.remove(cmdidx);
                newRbts[i].brain.Cmds.remove(cmdidx-1);
                int idx1 = floor(random(fourDirString.length));
                int idx2 = floor(random(fourDirString.length));
                newRbts[i].brain.Cmds.add(fourDirString[idx1]);
                newRbts[i].brain.Cmds.add(fourDirString[idx2]);
                noAdjust = false;
                break;
              }
            }
          }
        }
      }else{
        for (int i = 1; i< newRbts.length; i++) {
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
  }
  
  void GARemoveExtraShapes() {

    for (int i = 1; i< newRbts.length; i++) {
      if (!nonReached){
        ArrayList<String> Cmds = newRbts[i].brain.Cmds;
        int cmdSize = Cmds.size();
        for (int cmdidx = cmdSize-1; cmdidx >= 0; cmdidx--){
           int inMorph = int(Cmds.get(cmdidx));
           if (inMorph != 0){
             if (random(1) < baseMutRemoveShapeRate){
               newRbts[i].brain.Cmds.remove(cmdidx);
             } 
           }
        }
      }else{
        ArrayList<String> Cmds = newRbts[i].brain.Cmds;
        int cmdSize = Cmds.size();
        for (int cmdidx = cmdSize-1; cmdidx >= 0; cmdidx--){
           int inMorph = int(Cmds.get(cmdidx));
           if (inMorph != 0){
             if (random(1) < baseMutRemoveShapeRate){
               newRbts[i].brain.Cmds.remove(cmdidx);
               int idx1 = floor(random(fourDirString.length));
               newRbts[i].brain.Cmds.add(fourDirString[idx1]);
             } 
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
    TableRow newRow = popTable.addRow();
    newRow.setInt("genID", gen);
    newRow.setInt("reachGoal", 0);
    
    newRow.setFloat("bC-ID", bestCostCDRbtID);
    if (unsortRobotListCopy.get(bestCostCDRbtID).reachedGoal){
      newRow.setFloat("bC-C", unsortRobotListCopy.get(bestCostCDRbtID).costCD);
      newRow.setFloat("bC-Sm", unsortRobotListCopy.get(bestCostCDRbtID).smoothCD);
      newRow.setFloat("bC-Sf", unsortRobotListCopy.get(bestCostCDRbtID).safeCD);
      newRow.setFloat("bC-A", unsortRobotListCopy.get(bestCostCDRbtID).sumCD);
    }
    
    newRow.setFloat("bSm-ID", bestSmoothCDRbtID);
    if (unsortRobotListCopy.get(bestSmoothCDRbtID).reachedGoal){
      newRow.setFloat("bSm-C", unsortRobotListCopy.get(bestSmoothCDRbtID).costCD);
      newRow.setFloat("bSm-Sm", unsortRobotListCopy.get(bestSmoothCDRbtID).smoothCD);
      newRow.setFloat("bSm-Sf", unsortRobotListCopy.get(bestSmoothCDRbtID).safeCD);
      newRow.setFloat("bSm-A", unsortRobotListCopy.get(bestSmoothCDRbtID).sumCD);
    }
    
    newRow.setFloat("bSf-ID", bestSafeCDRbtID);
    if (unsortRobotListCopy.get(bestSafeCDRbtID).reachedGoal){
      newRow.setFloat("bSf-C", unsortRobotListCopy.get(bestSafeCDRbtID).costCD);
      newRow.setFloat("bSf-Sm", unsortRobotListCopy.get(bestSafeCDRbtID).smoothCD);
      newRow.setFloat("bSf-Sf", unsortRobotListCopy.get(bestSafeCDRbtID).safeCD);
      newRow.setFloat("bSf-A", unsortRobotListCopy.get(bestSafeCDRbtID).sumCD);
    }
    
    newRow.setFloat("bA-ID", bestSumCDRbtID);
    if (unsortRobotListCopy.get(bestSumCDRbtID).reachedGoal){
      newRow.setFloat("bA-C", unsortRobotListCopy.get(bestSumCDRbtID).costCD);
      newRow.setFloat("bA-Sm", unsortRobotListCopy.get(bestSumCDRbtID).smoothCD);
      newRow.setFloat("bA-Sf", unsortRobotListCopy.get(bestSumCDRbtID).safeCD); 
      newRow.setFloat("bA-A", unsortRobotListCopy.get(bestSumCDRbtID).sumCD); 
    }
    
    
    String toWrite;
    /*
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
    println("(Cost, Smooth, Safe, Sum) = (" + unsortRobotListCopy.get(bestSumCDRbtID).costCD + "," +
                                              unsortRobotListCopy.get(bestSumCDRbtID).smoothCD + "," +
                                              unsortRobotListCopy.get(bestSumCDRbtID).safeCD + "," +
                                              unsortRobotListCopy.get(bestSumCDRbtID).sumCD + ")" );
    */
  }
}
