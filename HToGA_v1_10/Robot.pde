class Robot {
  
  PVector pos;
  PVector vel;
  PVector acc;
  Brain brain;

  boolean dead = false;
  boolean reachedGoal = false;
  boolean isBest = false;//true if this dot is the best dot from the previous generation

  float fitness = 0;

  Block[] Blks;
  int morph = 2;
  Morphology Morph;
  float blkWidth = 20;

  Robot() {
    
    Morph = new Morphology();
      brain = new Brain(1001);//new brain with 1000 instructions

    //start the dots at the bottom of the window with a no velocity or acceleration
    pos = new PVector(width/2, height- 100);
    vel = new PVector(0, 0);
    acc = new PVector(0, 0);
    
    
    Blks = new Block[4];
    for(int blkidx = 0; blkidx < 4; blkidx++){
      //Blks[blkidx] = new Block(blkidx, pos, 0);
    }
    Blks[0] = new Block(0, blkWidth , pos.copy().add(new PVector(blkWidth,0)), 0);
    Blks[1] = new Block(1, blkWidth , pos.copy().add(new PVector(0,0)), 0);
    Blks[2] = new Block(2, blkWidth , pos.copy().add(new PVector(-blkWidth,0)), 0);
    Blks[3] = new Block(3, blkWidth , pos.copy().add(new PVector(-blkWidth*2,0)), 0);
    
    updateBlockDesHeading();
  }
  
  //-----------------------------------------------------------------------------------------------------------------
  // update desired heading to each robot blocks
  void updateBlockDesHeading(){
    int newMorph = brain.Cmds[brain.step].transMorph;
    if (newMorph != -1 && newMorph != morph){
      morph = newMorph;
    }
    
    int modMorph = morph % 7;
    int divMorph = floor((morph-1)/7);
    for(int blkidx = 0; blkidx < 4; blkidx++){
      float newHeading = Morph.RelAng[modMorph][blkidx] - divMorph * PI/2;
      // Set desired heading of each block according to the array values
      Blks[blkidx].desHeading = newHeading;
    }
  }

  //-----------------------------------------------------------------------------------------------------------------
  // perform robot shape-shifting
  boolean shapeShift(){
    if (debugMode){
      println("r, posx: " + pos.x + " posy:" + pos.y );
      println("b0, posx: " + Blks[0].pos.x + " posy:" + Blks[0].pos.y+ " ang:" + Blks[0].heading + " desAng:" + Blks[0].desHeading);
      println("b1, posx: " + Blks[1].pos.x + " posy:" + Blks[1].pos.y+ " ang:" + Blks[1].heading + " desAng:" + Blks[1].desHeading);
      println("b2, posx: " + Blks[2].pos.x + " posy:" + Blks[2].pos.y+ " ang:" + Blks[2].heading + " desAng:" + Blks[2].desHeading);
      println("b3, posx: " + Blks[3].pos.x + " posy:" + Blks[3].pos.y+ " ang:" + Blks[3].heading + " desAng:" + Blks[3].desHeading);
    }
    
    if (abs(Blks[0].heading - Blks[0].desHeading) > rotThreshold){
      // Block 2 rotates with respect to bottom-right corner of block 1
      float rotateAng = (Blks[0].desHeading > Blks[0].heading)? rotAngVel : -rotAngVel;
      Rotate(0,rotateAng, Blks[1].getCorner(3));
      if (debugMode){
        println("Block 0 rotating w.r.t. (x,y) = (" + Blks[1].getCorner(3).x + ", " +Blks[1].getCorner(3).y+ ").");
      }
      return true;
    }
    else if (abs(Blks[1].heading - Blks[1].desHeading) > rotThreshold){
     // Blks[1].Rotate(Blks[1].desHeading-Blks[1].heading, Blks[1].getCorner(3));
     return true;
    }
    else if (abs(Blks[2].heading - Blks[2].desHeading) > rotThreshold){
      // Block 2 rotates with respect to upper-right corner of block 1
      float rotateAng = (Blks[2].desHeading > Blks[2].heading)? rotAngVel : -rotAngVel;
      Rotate(2,rotateAng, Blks[1].getCorner(0));
      if (debugMode){
        println("Block 2 rotating w.r.t. (x,y) = (" + Blks[1].getCorner(0).x + ", " +Blks[1].getCorner(0).y+ ").");
      }
      return true;
    }
    else if (abs(Blks[3].heading - Blks[3].desHeading) > rotThreshold){
      // Block 3 rotates with respect to upper-left corner of block 2
      float rotateAng = (Blks[3].desHeading > Blks[3].heading)? rotAngVel : -rotAngVel;
      Rotate(3,rotateAng, Blks[2].getCorner(1));
      if (debugMode){
        println("Block 3 rotating w.r.t. (x,y) = (" + Blks[2].getCorner(1).x + ", " +Blks[2].getCorner(1).y+ ").");
      }
      return true;
    }
    return false;
  }
  
  void Rotate(int blkId, float angle, PVector posCtr){
    Blks[blkId].heading = Blks[blkId].heading + angle;
    PVector posToCenter = Blks[blkId].pos.copy().sub(posCtr);
    PVector posShift = posToCenter.copy().rotate(angle).sub(posToCenter);
    Blks[blkId].pos.add(posShift);
    
    if (blkId == 2){
      Rotate(3, angle, posCtr);
    }
    if(debugMode){
      println("Block " + blkId + " Rotation:");
      println("posCtr (x,y) = (" + posCtr.x + ", " + posCtr.y +").");
      println("posToCenter (x,y) = (" + posToCenter.x + ", " + posToCenter.y +").");
      println("posToCenter (x,y).rotate(angle) = (" + posToCenter.rotate(angle).x + ", " + posToCenter.rotate(angle).y +").");
      println("posToCenter (x,y).rotate(angle).sub(posToCenter) = (" + posToCenter.rotate(angle).sub(posToCenter).x + ", " + posToCenter.rotate(angle).sub(posToCenter).y +").");
    }
  }
  

  //-----------------------------------------------------------------------------------------------------------------
  //draws the dot on the screen
  void show() {
    
    //if this dot is the best dot from the previous generation then draw it as a big green dot
    if (isBest) {
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 255, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
      }
    } else {//all other dots are just smaller black dots
      fill(0);
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 0, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
      }
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------
  //moves the dot according to the brains directions
  void move() {
    updateBlockDesHeading();
    if (brain.Cmds.length > brain.step) {  //if there are still directions left then set the acceleration as the next PVector in the direcitons array
      if (!shapeShift()){
        acc = brain.Cmds[brain.step].moveDir;
        //apply the acceleration and move the dot
        vel.add(acc);
        vel.limit(5);//not too fast
        pos.add(vel);
          
        for (int blkidx = 0; blkidx < Blks.length; blkidx++){
          Blks[blkidx].pos.add(vel);
        }
      }
      brain.step++;
      
    } else {  //if at the end of the directions array then the dot is dead
      dead = true;
    }
  
  }

  //-------------------------------------------------------------------------------------------------------------------
  //calls the move function 
  void update() {
    if (!dead && !reachedGoal) {
      move();
      if (Blks[1].pos.x < blkWidth/2.0|| Blks[1].pos.y < blkWidth/2.0 || Blks[1].pos.x > width - blkWidth/2.0 || Blks[1].pos.y > height - blkWidth/2.0) { //if near the edges of the window then kill it 
        dead = true;
      } else if (dist(Blks[1].pos.x, Blks[1].pos.y, goal.x, goal.y) < 5) {//if reached goal
        reachedGoal = true;
      }
    }
  }
  
  //--------------------------------------------------------------------------------------------------------------------------------------
  // mutate
  void mutate(){
    for (int i = 0; i < brain.Cmds.length; i++) {
      float rand = random(1);
      if (rand < baseMutTransRate){
        int randMorph = floor(random(7));
        morph = randMorph;
        brain.Cmds[i].transMorph = randMorph;
      }else if (rand < baseMutMoveRate) {
        //set this direction as a random direction 
        float randomAngle = random(2*PI);
        brain.Cmds[i].moveDir = PVector.fromAngle(randomAngle);
      }
    }
  }



  //--------------------------------------------------------------------------------------------------------------------------------------
  //calculates the fitness
  void calculateFitness() {
    if (reachedGoal) {//if the dot reached the goal then the fitness is based on the amount of steps it took to get there
      fitness = 1.0/16.0 + 10000.0/(float)(brain.step * brain.step);
    } else {//if the dot didn't reach the goal then the fitness is based on how close it is to the goal
      float distanceToGoal = dist(Blks[1].pos.x, Blks[1].pos.y, goal.x, goal.y);
      fitness = 1.0/(distanceToGoal * distanceToGoal);
    }
  }

  //---------------------------------------------------------------------------------------------------------------------------------------
  //clone it 
  Robot gimmeBaby() {
    Robot baby = new Robot();
    baby.brain = brain.clone();//babies have the same brain as their parents
    return baby;
  }
}
