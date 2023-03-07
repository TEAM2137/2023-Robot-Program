package frc.robot.camera.objects;

import java.util.ArrayList;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Gets the data of objects in the camera's vision from the raspberry pi networktables. 
 * <p> The update method must be called often to refresh the data!
 */
public class ObjectTracker {

    private double[] coneXPositions;
    private double[] coneYPositions;
    private double[] cubeXPositions;
    private double[] cubeYPositions;

    private double[] coneWidths;
    private double[] coneHeights;
    private double[] cubeWidths;
    private double[] cubeHeights;

    private int amountOfCubes;
    private int amountOfCones;

    private boolean wasFlippedLastUpdate;
    // 10 updates of timeout, might need to change this
    private int maxFlipTimeout = 10;
    private int flipTimeout = maxFlipTimeout;

    private String subTable;

    private NetworkTableInstance inst;
    private NetworkTable coneTable;
    private NetworkTable cubeTable;
    
    // TODO calculate conversion values
    private double CONE_UP_DST_INCHES = 0.1;
    private double CONE_DOWN_DST_INCHES = 0.1;
    private double CUBE_DST_INCHES = 0.1;

    /**
     * Creates a new instance of the object tracker with a default sub table of "vision"
     */
    public ObjectTracker() {
        this("vision");
    }

    /**
     * @param subTable the name of the network table inside of the "RPI" table to get data from.
     */
    public ObjectTracker(String subTable) {
        this.subTable = subTable;
    }

    /**
     * @return an <code>ArrayList</code> of rectangles outlining the cones in the camera's vision.
     */
    public ArrayList<Rect> getConeBounds() {
        ArrayList<Rect> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCones; i++) {
            positions.add(new Rect((int) coneXPositions[i], (int) coneYPositions[i], (int) coneWidths[i], (int) coneHeights[i]));
        }
        return positions;
    }

    /**
     * @return an array of rectangles outlining the cones in the camera's vision.
     */
    public Rect[] getConeBoundsArray() {
        return (Rect[]) getConeBounds().toArray();
    }

    /**
     * @return an <code>ArrayList</code> of rectangles outlining the cubes in the camera's vision.
     */
    public ArrayList<Rect> getCubeBounds() {
        ArrayList<Rect> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCubes; i++) {
            positions.add(new Rect((int) cubeXPositions[i], (int) cubeYPositions[i], (int) cubeWidths[i], (int) cubeHeights[i]));
        }
        return positions;
    }

    /**
     * @return an array of rectangles outlining the cubes in the camera's vision.
     */
    public Rect[] getCubeBoundsArray() {
        return (Rect[]) getCubeBounds().toArray();
    }

    /**
     * @return an <code>ArrayList</code> of points at the center positions of the cones in the camera's vision.
     */
    public ArrayList<Point> getConePositions() {
        ArrayList<Point> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCones; i++) {
            positions.add(new Point((int) coneXPositions[i], (int) coneYPositions[i]));
        }
        return positions;
    }

    /**
     * @return an array of points at the center positions of the cones in the camera's vision.
     */
    public Point[] getConePositionsArray() {
        return (Point[]) getConePositions().toArray();
    }

    /**
     * @return a list of points at the center positions of the cubes in the camera's vision.
     */
    public ArrayList<Point> getCubePositions() {
        ArrayList<Point> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCubes; i++) {
            positions.add(new Point((int) cubeXPositions[i], (int) cubeYPositions[i]));
        }
        return positions;
    }

    /**
     * @return an array of points at the center positions of the cubes in the camera's vision.
     */
    public Point[] getCubePositionsArray() {
        return (Point[]) getCubePositions().toArray();
    }

    /**
     * @return an <code>ArrayList</code> of points for the sizes of cones in the camera's vision. 
     * <p> The x positions are the widths, and the y positions are the heights.
     */
    public ArrayList<Point> getConeSizes() {
        ArrayList<Point> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCones; i++) {
            positions.add(new Point((int) coneWidths[i], (int) coneHeights[i]));
        }
        return positions;
    }

    /**
     * @return an array of points for the sizes of cones in the camera's vision. 
     * <p> The x positions are the widths, and the y positions are the heights.
     */
    public Point[] getConeSizesArray() {
        return (Point[]) getConeSizes().toArray();
    }

    /**
     * @return an <code>ArrayList</code> of points for the sizes of cubes in the camera's vision. 
     * <p> The x positions are the widths, and the y positions are the heights.
     */
    public ArrayList<Point> getCubeSizes() {
        ArrayList<Point> positions = new ArrayList<>();
        for (int i = 0; i < amountOfCubes; i++) {
            positions.add(new Point((int) cubeWidths[i], (int) cubeHeights[i]));
        }
        return positions;
    }

    /**
     * @return an array of rectangles outlining the cones in the camera's vision.
     * <p> The x positions are the widths, and the y positions are the heights.
     */
    public Point[] getCubeSizesArray() {
        return (Point[]) getCubeSizes().toArray();
    }

    /**
     * @return the total number of objects in the camera's vision (cones or cubes).
     */
    public int getTotalObjectCount() {
        return amountOfCones + amountOfCubes;
    }

    /**
     * @return the total number of cubes in the camera's vision
     */
    public int getCubeCount() {
        return amountOfCubes;
    }

    /**
     * @return the total number of cubes in the camera's vision
     */
    public int getConeCount() {
        return amountOfCones;
    }

    /**
     * @param index the cone to distance check
     * @return the distance of the specified cone, in inches.
     */
    public double getConeDistance(int index) {
        double dst = 0;
        if (isConeUpright(index)) {
            dst = coneWidths[index] * CONE_UP_DST_INCHES;
        } else {
            dst = coneHeights[index] * CONE_DOWN_DST_INCHES;
        }
        return dst;
    }

    /**
     * @param index the cube to distance check
     * @return the distance of the specified cube, in inches.
     */
    public double getCubeDistance(int index) {
        double area = 0;
        double dst = 0;
        area = cubeWidths[index] * cubeHeights[index];
        dst = area * CUBE_DST_INCHES;
        return dst;
    }

    /**
     * @return the index closest clone to the camera.
     * <p> The index can be used to get distance or determine if the cone is upright.
     */
    public int getClosestCone() {
        if (coneWidths.length > 0) {
            double closest = 0;
            int index = 0;
            for (int i = 0; i < coneWidths.length; i++){
                double area = coneWidths[i] * coneHeights[i];
                if (area > closest) {
                    closest = area;
                    index = i;
                }
            }
            return index;
        } else {
            return 0;
        }
    }

    /**
     * @return true if the cone at the index is standing up (w > h) and false if it is on its side.
     * @param index which cone to check
     * <p>Should be run once every update
     */
    public boolean isConeUpright(int index) {
        boolean output = true;
        Point cone = new Point(coneWidths[index], coneHeights[index]);
        if (cone.x + 10 > cone.y){
            output = false;
        }
        if (wasFlippedLastUpdate && !output) {
            if (flipTimeout > 0) {
                flipTimeout--;
                output = true;
                wasFlippedLastUpdate = true;
            } else {
                flipTimeout = 10;
            }
        } else {
            flipTimeout = 10;
        }
        wasFlippedLastUpdate = output;
        return output;
    }

    /**
     * Gets the data from the networktables. Should be run every time camera data is needed.
     */
    public void update() {
        inst = NetworkTableInstance.getDefault();
        cubeTable = inst.getTable("RPI").getSubTable(subTable).getSubTable("cubes");
        cubeTable = inst.getTable("RPI").getSubTable(subTable).getSubTable("cones");

        cubeXPositions = cubeTable.getEntry("xPositions").getDoubleArray(new double[10]);
        cubeYPositions = cubeTable.getEntry("yPositions").getDoubleArray(new double[10]);
        cubeWidths = cubeTable.getEntry("widths").getDoubleArray(new double[10]);
        cubeHeights = cubeTable.getEntry("heights").getDoubleArray(new double[10]);
        amountOfCubes = (int) cubeTable.getEntry("amountDetected").getInteger((long)0);

        coneXPositions = coneTable.getEntry("xPositions").getDoubleArray(new double[10]);
        coneYPositions = coneTable.getEntry("yPositions").getDoubleArray(new double[10]);
        coneWidths = coneTable.getEntry("widths").getDoubleArray(new double[10]);
        coneHeights = coneTable.getEntry("heights").getDoubleArray(new double[10]);
        amountOfCones = (int) coneTable.getEntry("amountDetected").getInteger((long)0);
    }
}