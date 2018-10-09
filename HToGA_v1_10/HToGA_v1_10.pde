Population test;
int mapID = 0;
int wpID;

boolean debugMode = false;
boolean gridBasedMode = true;


float rotAngVel = PI/12.0;  // Rotation angular velocity
float rotThreshold = PI/22.0; // Rotation threshold angle for the robot to stop 
float baseMutMoveRate = 0.005; // Mutation rate of robot moving direction
float baseMutTransRate = 0.0001; // Mutation rate of robt transformation
float moveTransRatio = 230.0;

void setup() {
 
  int totPopulation = 1000;
  int minStep = 1000;
  
  
  size(800, 800); //size of the window
  frameRate(100);//increase this to make the dots go faster
 
  test = new Population(totPopulation, minStep, Obss);//create a new population with 1000 members

}


void draw() { 
  
  background(255);
  
  //draw goal
  fill(255, 0, 0);
  for (int wpidx = 0; wpidx < MapDB.Maps[mapID].Wps.length ; wpidx++){
  }
  ellipse(goal.x, goal.y, 10, 10);
  
  //draw obstacle(s)
  fill(0, 0, 255);
  
  for (int intobs = 0; intobs < Obss.length; intobs++){
    rect(Obss[intobs].pos.x,Obss[intobs].pos.y, Obss[intobs].size.x, Obss[intobs].size.y);
  }
  
  if (test.allRobotsDead()) {
      //genetic algorithm
      test.calculateFitness();
      test.naturalSelection();
      test.mutateDemBabies();
  } else {
    test.update();
    test.show();
  }
}
