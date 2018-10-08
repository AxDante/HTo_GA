Population test;
PVector goal  = new PVector(400, 10);
Obstacle[] Obss;

boolean debugMode = true;

void setup() {
  
  if (debugMode){
    Robot r = new Robot();
    println("r, posx: " + r.pos.x + " posy:" + r.pos.y );
    println("b0, posx: " + r.Blks[0].pos.x + " posy:" + r.Blks[0].pos.y+ " ang:" + r.Blks[0].heading + " desAng:" + r.Blks[0].desHeading);
    
    println("b0UR, posx: " + r.Blks[0].getCorner(0).x + " posy:" + r.Blks[0].getCorner(0).y);
    println("b0UL, posx: " + r.Blks[0].getCorner(1).x + " posy:" + r.Blks[0].getCorner(1).y);
    println("b0BL, posx: " + r.Blks[0].getCorner(2).x + " posy:" + r.Blks[0].getCorner(2).y);
    println("b0BR, posx: " + r.Blks[0].getCorner(3).x + " posy:" + r.Blks[0].getCorner(3).y);
    println("b1, posx: " + r.Blks[1].pos.x + " posy:" + r.Blks[1].pos.y+ " ang:" + r.Blks[1].heading + " desAng:" + r.Blks[1].desHeading);
    println("b2, posx: " + r.Blks[2].pos.x + " posy:" + r.Blks[2].pos.y+ " ang:" + r.Blks[2].heading + " desAng:" + r.Blks[2].desHeading);
    println("b3, posx: " + r.Blks[3].pos.x + " posy:" + r.Blks[3].pos.y+ " ang:" + r.Blks[3].heading + " desAng:" + r.Blks[3].desHeading);
    //r.pos = new PVector(300, 300);
    
  
  }else if (!debugMode){
    
    size(800, 800); //size of the window
    frameRate(100);//increase this to make the dots go faster
    
    int totPopulation = 1000;
    
    Obss = new Obstacle[3];
    Obss[0] = new Obstacle(new PVector(0, 300), new PVector(600, 10));
    Obss[1] = new Obstacle(new PVector(300,500), new PVector(500, 10));
    Obss[2] = new Obstacle(new PVector(500,100), new PVector(100, 100));
    
    test = new Population(totPopulation, Obss);//create a new population with 1000 members
  }
}


void draw() { 
  if (!debugMode){
    background(255);
  
    //draw goal
    fill(255, 0, 0);
    ellipse(goal.x, goal.y, 10, 10);
  
    //draw obstacle(s)
    fill(0, 0, 255);
  
    for (int intobs = 0; intobs < Obss.length; intobs++){
      rect(Obss[intobs].pos.x,Obss[intobs].pos.y, Obss[intobs].size.x, Obss[intobs].size.y);
      //rect(0, 300, 600, 10);
    }
  
    if (test.allDotsDead()) {
      //genetic algorithm
      test.calculateFitness();
      test.naturalSelection();
      test.mutateDemBabies();
    } else {
      //if any of the dots are still alive then update and then show them
  
      test.update();
      test.show();
    }
  }
}
