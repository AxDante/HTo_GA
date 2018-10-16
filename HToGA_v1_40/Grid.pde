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
  
  public Grid (int xcord, int ycord) {
    x = xcord;
    y = ycord;
    float chance = random(2);
    if (chance < 1) {
      isWall = true;
    } else {
      isWall = false;
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
