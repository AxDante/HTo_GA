public class Grid {
  public int x;
  public int y;
  public boolean checked = false;
  public boolean isWall;
  
  
  public double g;
  public double h;
  public double f;
  
  private Grid parent = null;
  
  public void link() {
    for (int i = x-1; i < x+2; i++) {
      for (int j = y-1; j < y+2; j++) {
        if ((i >= 0 && j >= 0) && (j < mapH && i < mapW)) {
          //System.out.println(i +" " + j);
          boolean closer = false;
          for (int c = 0; c < closed.size(); c++) {
            if (grids[i][j] == closed.get(c)) {
              closer = true;
            }
          }
          if (!closer && !grids[i][j].isWall) {
            boolean opener = false;
            for (int b = 0; b < open.size(); b++) {
              if (grids[i][j] == open.get(b)) {
                //println("ok");
                opener = true;
              }
            }
            if (opener) {
              if (g < grids[i][j].g-1 && (i == x || j == y)) {
                grids[i][j].g = g+1;
                grids[i][j].setParent(grids[x][y]);
                grids[i][j].calcF();
                
              } else if (g < grids[i][j].g-diag) {
                grids[i][j].g = g+diag;
                grids[i][j].setParent(grids[x][y]);
                grids[i][j].calcF();
              }
            } else {
              open.add(grids[i][j]);
              grids[i][j].checked = true;
              if (i != x && j != y) {
                grids[i][j].g = g + diag;
              } else {
                grids[i][j].g = g+1;
              }
              grids[i][j].setParent(grids[x][y]);
              grids[i][j].calcF();
            }
          }
        }
      }
    }
  }
  
  public void setParent(Grid p) {
    parent = p;
  }
  
  public Grid (int xcord, int ycord, Obstacle[] Obss) {
    x = xcord;
    y = ycord;
    for (int intobs = 0; intobs < Obss.length; intobs++){
      if (x > Obss[intobs].pos.x && x < Obss[intobs].pos.x +  Obss[intobs].size.x &&
      y > Obss[intobs].pos.y && y < Obss[intobs].pos.y +  Obss[intobs].size.y){
        isWall = true;
      }
    }
  }
  
  public void heuristic () {
    h = Math.sqrt(Math.pow((mapW-x), 2) + Math.pow((mapH-y), 2));
  }
  
  public void calcF() {
    f = g+h;
  }
  
  public double getF() {
    return f;
  }
  
  public void show() {
    if (parent!=null) {
      stroke(0);
      line(x*blkWidth+blkWidth/2, y*blkWidth+blkWidth/2, parent.x*blkWidth+blkWidth/2, parent.y*blkWidth+blkWidth/2);
      parent.show();
    }
  }
  
  
  
  
  
  
}




Grid getMin() {
  if (open.size() > 0) {
    Grid min = open.get(0);
    for (int i = 1; i < open.size(); i++) {
      if (open.get(i).getF() < min.getF()) {
        min = open.get(i);
      }
    }
    return min;
  }
  return null;
}
