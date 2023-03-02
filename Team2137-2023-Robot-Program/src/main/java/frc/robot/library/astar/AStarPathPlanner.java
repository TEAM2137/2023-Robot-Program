package frc.robot.library.astar;

import edu.wpi.first.math.Pair;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Stack;

public class AStarPathPlanner {
    boolean[][] grid;
    boolean[][] closedList; //Final points that are viable
    BoundaryMapEntry[][] cellDetails; //2D array to hold the details of every cell

    LinkedList<OpenListEntry> openList;
    Pair<Integer, Integer> dest;
    Pair<Integer, Integer> src;

    int ROW = 0;
    int COL = 0;

    public AStarPathPlanner(boolean[][] _grid, Pair<Integer, Integer> srcCord, Pair<Integer, Integer> destCord) {
        grid = _grid;

        ROW = grid.length;
        COL = grid[0].length;

        closedList = new boolean[ROW][COL];
        cellDetails = new BoundaryMapEntry[ROW][COL];

        openList = new LinkedList<>();

        //Reset all the nodes
        for(int i = 0; i < ROW; i++) {
            for(int j = 0; j < COL; j++){
                cellDetails[i][j] = new BoundaryMapEntry();

                cellDetails[i][j].f = Double.MAX_VALUE;
                cellDetails[i][j].g = Double.MAX_VALUE;
                cellDetails[i][j].h = Double.MAX_VALUE;
                cellDetails[i][j].parent_i = -1;
                cellDetails[i][j].parent_j = -1;
            }
        }

        //Initialising the parameters of the starting node
        int i = srcCord.getFirst();
        int j = srcCord.getSecond();
        cellDetails[i][j].f = 0.0;
        cellDetails[i][j].g = 0.0;
        cellDetails[i][j].h = 0.0;
        cellDetails[i][j].parent_i = i;
        cellDetails[i][j].parent_j = j;

        dest = destCord;
        src = srcCord;
    }

    public ArrayList<Pair<Integer, Integer>> run() {
        int i = src.getFirst();
        int j = src.getSecond();

        openList.add(new OpenListEntry(i, j, 0.0));

        boolean foundDestinationFlag = false;

        //Run until there are no points left to check
        while(!openList.isEmpty()) {
            OpenListEntry p = openList.pop(); //Take the first element off the list

            i = p.i;
            j = p.j;
            //System.out.printf("Current Point: (%d, %d)\n", i, j);
            closedList[i][j] = true;

            //Check each of the possible child cells
            for(BoundaryMapEntry.Direction direction : BoundaryMapEntry.Direction.values()) {
                int testI = i + direction.getI();
                int testJ = j + direction.getJ();

                //System.out.printf("Checking Point: (%d, %d) -> ", testI, testJ);

                //Only process cells that are of valid type
                if(isValid(testI, testJ)) {
                    //Check if we found our goal point
                    if (isDestination(testI, testJ)) {
                        BoundaryMapEntry entry = cellDetails[testI][testJ];
                        entry.parent_i = i;
                        entry.parent_j = j;
                        //System.out.println("The destination cell is found!!");
                        foundDestinationFlag = true;
                        return tracePath(src, dest);
                    } else if (!closedList[testI][testJ]
                            && isUnBlocked(testI, testJ)) {

                        double gNew = cellDetails[i][j].g + 1.0;
                        double hNew = calculateHValue(testI, testJ);
                        double fNew = gNew + hNew;

                        if (cellDetails[testI][testJ].f == Double.MAX_VALUE
                                || cellDetails[testI][testJ].f > fNew) {

                            //System.out.println("Valid Point!");

                            openList.add(new OpenListEntry(testI, testJ, fNew));

                            cellDetails[testI][testJ].f = fNew;
                            cellDetails[testI][testJ].g = gNew;
                            cellDetails[testI][testJ].h = hNew;
                            cellDetails[testI][testJ].parent_i = i;
                            cellDetails[testI][testJ].parent_j = j;

                            continue;
                        }
                    }
                }

                //System.out.println("Invalid Point.");
            }
        }

        if(!foundDestinationFlag) {
            System.out.println("No Path Found");
        }

        return null;
    }

    public boolean isValid(int i, int j) { //Function that decides if a point is valid type
        return (i >= 0) && (i < ROW) && (j >= 0) && (j < COL);
    }

    public boolean isDestination(int i, int j) {
        if(i == dest.getFirst() && j == dest.getSecond())
            return true;
        else
            return false;
    }

    public boolean isUnBlocked(int i, int j) {
        if (grid[i][j] == false)
            return true;
        else
            return false;
    }

    public ArrayList<Pair<Integer, Integer>> tracePath(Pair<Integer, Integer> src, Pair<Integer, Integer> dest) {
        //System.out.println("The Path is... ");
        BoundaryMapEntry current = cellDetails[dest.getFirst()][dest.getSecond()];

        Stack<BoundaryMapEntry> cells = new Stack<>();

        while(src.getFirst() != current.parent_i || src.getSecond() != current.parent_j) {
            cells.push(current);
            //System.out.printf("-> (%d, %d)\n", current.parent_i, current.parent_j);
            current = cellDetails[current.parent_i][current.parent_j];
        }

        cells.push(cellDetails[src.getFirst()][src.getSecond()]);

        //boolean[][] map = new boolean[ROW][COL];
        ArrayList<Pair<Integer, Integer>> points = new ArrayList<>();

        while(!cells.isEmpty()) {
            BoundaryMapEntry tmp = cells.pop();

            points.add(Pair.of(tmp.parent_i, tmp.parent_j));
            //map[tmp.parent_i][tmp.parent_j] = true;
            //System.out.printf("-> (%d, %d)\n", tmp.parent_i, tmp.parent_j);
        }

        points.add(Pair.of(dest.getFirst(), dest.getSecond()));
        //map[dest.getFirst()][dest.getSecond()] = true;

        //printGrid(map);

        //System.out.printf("-> (%d, %d)\n", dest.getFirst(), dest.getSecond());
        return points;
    }

    public void printGrid(boolean[][] map) {
        for(int y = 0; y < map.length; y++) {
            System.out.print("|");
            for(int x = 0; x < map[0].length; x++) {
                if(map[y][x]) {
                    System.out.print("X|");
                } else if(grid[y][x]) {
                    System.out.print("O|");
                } else {
                    System.out.print(" |");
                }
            }
            System.out.println();
        }
    }

    public double calculateHValue(int i, int j) {
        return Math.sqrt(Math.pow(i - dest.getFirst(), 2) + Math.pow(j + dest.getSecond(), 2));
    }

    class OpenListEntry {
        public double f;
        public int i;
        public int j;

        public OpenListEntry(int _i, int _j, double _f) {
            i = _i;
            j = _j;
            f = _f;
        }
    }
}
