class Morphology{
  
  float[][] RelAng;
  
  Morphology(){
    RelAng = new float[][]{{0, 0, 0, 0}, 
                             {0, 0, -PI, -PI},
                             {PI, 0, PI/2.0, PI/2.0},
                             {PI, 0, 0, 0},
                             {PI, 0, 0, PI},
                             {PI/2.0, 0, -PI, 0},
                             {PI, 0, -PI/2.0, -PI/2.0}};
  }
                             
}
