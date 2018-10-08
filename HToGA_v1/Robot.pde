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
  int morph = 1;
  Morphology Morph;
  float blkWidth = 15;

  Robot() {
    
    Morph = new Morphology();
    brain = new Brain(1000);//new brain with 1000 instructions

    //start the dots at the bottom of the window with a no velocity or acceleration
    pos = new PVector(width/2, height- 100);
    vel = new PVector(0, 0);
    acc = new PVector(0, 0);
    
    
    Blks = new Block[4];
    for(int blkidx = 0; blkidx < 4; blkidx++){
      //Blks[blkidx] = new Block(blkidx, pos, 0);
    }
    Blks[0] = new Block(0, blkWidth , pos.add(new PVector(blkWidth,0)), 0);
    Blks[1] = new Block(1, blkWidth , pos.add(new PVector(0,0)), 0);
    Blks[2] = new Block(2, blkWidth , pos.add(new PVector(blkWidth,0)), 0);
    Blks[3] = new Block(3, blkWidth , pos.add(new PVector(blkWidth,0)), 0);
    updateBlockDesHeading();
  }
  
  //-----------------------------------------------------------------------------------------------------------------
  // update desired heading to each robot blocks
  void updateBlockDesHeading(){
    int modMorph = morph % 7;
    int divMorph = floor((morph-1)/7);
    for(int blkidx = 0; blkidx < 4; blkidx++){
      float newHeading = Morph.RelAng[modMorph][blkidx] - divMorph * PI/2;
      // Set desired heading of each block according to the array values
      Blks[blkidx].desHeading = newHeading;
    }
    shapeShift();
  }

  //-----------------------------------------------------------------------------------------------------------------
  // perform robot shape-shifting
  void shapeShift(){
    if (Blks[0].heading != Blks[0].desHeading){
      Blks[0].Rotate(Blks[0].desHeading-Blks[0].heading, Blks[1].getCorner(3));
    }
    if (Blks[1].heading != Blks[1].desHeading){
     // Blks[1].Rotate(Blks[1].desHeading-Blks[1].heading, Blks[1].getCorner(3));
    }
    if (Blks[2].heading != Blks[2].desHeading){
      Blks[2].Rotate(Blks[2].desHeading-Blks[3].heading, Blks[1].getCorner(0));
    }
    if (Blks[3].heading != Blks[3].desHeading){
      Blks[3].Rotate(Blks[3].desHeading-Blks[3].heading, Blks[2].getCorner(1));
    }
  }


  //-----------------------------------------------------------------------------------------------------------------
  //draws the dot on the screen
  void show() {
    shapeShift();
    //if this dot is the best dot from the previous generation then draw it as a big green dot
    if (isBest) {
      for (int blkidx = 0; blkidx < Blks.length; blkidx++){
        fill(0, 255, 0);
        rect(Blks[blkidx].pos.x, Blks[blkidx].pos.y, Blks[blkidx].blkWidth, Blks[blkidx].blkWidth);
      }
    } else {//all other dots are just smaller black dots
      fill(0);
      rect(pos.x, pos.y, 10, 10);
    }
  }

  //-----------------------------------------------------------------------------------------------------------------------
  //moves the dot according to the brains directions
  void move() {

    if (brain.directions.length > brain.step) {//if there are still directions left then set the acceleration as the next PVector in the direcitons array
      acc = brain.directions[brain.step];
      brain.step++;
    } else {//if at the end of the directions array then the dot is dead
      dead = true;
    }

    //apply the acceleration and move the dot
    vel.add(acc);
    vel.limit(5);//not too fast
    pos.add(vel);
  }

  //-------------------------------------------------------------------------------------------------------------------
  //calls the move function 
  void update() {
    if (!dead && !reachedGoal) {
      move();
      if (pos.x< 2|| pos.y<2 || pos.x>width-2 || pos.y>height -2) {//if near the edges of the window then kill it 
        dead = true;
      } else if (dist(pos.x, pos.y, goal.x, goal.y) < 5) {//if reached goal

        reachedGoal = true;
      //} else if (pos.x< 600 && pos.y < 310 && pos.x > 0 && pos.y > 300) {//if hit obstacle
      //  dead = true;
      }
    }
  }
  //-------------------------------------------------------------------------------------------------------------------
  // check for collisions
  void isCollide (){
    
  }

  //--------------------------------------------------------------------------------------------------------------------------------------
  //calculates the fitness
  void calculateFitness() {
    if (reachedGoal) {//if the dot reached the goal then the fitness is based on the amount of steps it took to get there
      fitness = 1.0/16.0 + 10000.0/(float)(brain.step * brain.step);
    } else {//if the dot didn't reach the goal then the fitness is based on how close it is to the goal
      float distanceToGoal = dist(pos.x, pos.y, goal.x, goal.y);
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
