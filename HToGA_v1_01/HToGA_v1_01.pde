Population trial;
PVector goal  = new PVector(400, 40);
Obstacle[] Obss;

boolean debugMode = false;
boolean gridBasedMode = true;


float rotAngVel = PI/12.0;  // Rotation angular velocity
float rotThreshold = PI/22.0; // Rotation threshold angle for the robot to stop 
float baseMutMoveRate = 0.005; // Mutation rate of robot moving direction
float baseMutTransRate = 0.001; // Mutation rate of robt transformation
float moveTransRatio = 60.0;

void setup() {
 
  int totPopulation = 1000;
  int minStep = 1000;
  
  
  size(800, 800); //size of the window
  frameRate(100);//increase this to make the dots go faster
 
  Obss = new Obstacle[3];
  Obss[0] = new Obstacle(new PVector(0, 300), new PVector(500, 10));
  Obss[1] = new Obstacle(new PVector(300,500), new PVector(500, 10));
  Obss[2] = new Obstacle(new PVector(500,100), new PVector(100, 100));
    
  trial = new Population(totPopulation, minStep, Obss);//create a new population with 1000 members

  
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
    }
  
    if (trial.allDotsDead()) {
      //genetic algorithm
      trial.calculateFitness();
      trial.naturalSelection();
      trial.mutateDemBabies();
    } else {
      trial.update();
      trial.show();
    }
}
