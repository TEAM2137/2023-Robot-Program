package frc.robot.library.astar;

public class BoundaryMapEntry {
    public enum Direction {
        NORTH       (-1, 0),
        NORTH_EAST  (-1, 1),
        EAST        (0,  1),
        SOUTH_EAST  (1,  1),
        SOUTH       (1,  0),
        SOUTH_WEST  (1, -1),
        WEST        (0, -1),
        NORTH_WEST  (-1,-1);

        final int i, j;

        Direction(int _i, int _j) {
            i = _i;
            j = _j;
        }

        public int getI(){
            return i;
        }

        public int getJ() {
            return j;
        }
    }

    int parent_i; //Row index of parent cell
    int parent_j; //Column index of parent cell

    double f; // f = g + h
    double g;
    double h;

    public BoundaryMapEntry() {

    }
}
