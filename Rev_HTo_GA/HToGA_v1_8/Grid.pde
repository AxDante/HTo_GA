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
  
  public Grid (int xcord, int ycord, int[][] obsTable) {
    x = xcord;
    y = ycord;
    if (obsTable[x][y] == 1){
      isWall = true;
    }
    if (x == startGrid[0] && y == startGrid[1]){
      g = 0;
      checked = true;
    }
    if (x == goalGrid[0] && y == goalGrid[1]){
      isWall = false;
    }
  }
  
  public void heuristic () {
    h = Math.sqrt(Math.pow(goalGrid[0]-x, 2) + Math.pow(goalGrid[1]-y, 2));
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
      line(x*blkW+blkW/2, y*blkW+blkW/2, parent.x*blkW+blkW/2, parent.y*blkW+blkW/2);
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
