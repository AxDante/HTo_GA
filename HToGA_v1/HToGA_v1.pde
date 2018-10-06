Population test;
PVector goal  = new PVector(400, 10);
Obstacle[] Obss;

void setup() {
  size(800, 800); //size of the window
  frameRate(100);//increase this to make the dots go faster
  
  int totPopulation = 1000;
  
  Obss = new Obstacle[3];
  Obss[0] = new Obstacle(new PVector(0, 300), new PVector(600, 10));
  Obss[1] = new Obstacle(new PVector(300,500), new PVector(500, 10));
  Obss[2] = new Obstacle(new PVector(500,100), new PVector(100, 100));
  
  test = new Population(totPopulation, Obss);//create a new population with 1000 members
}


void draw() { 
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
