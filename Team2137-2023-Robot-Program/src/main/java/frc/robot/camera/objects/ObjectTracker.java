package frc.robot.camera.objects;

import java.util.ArrayList;

import org.opencv.core.Rect;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Gets the data of objects in the camera's vision from the raspberry pi networktables. Call the update method to refresh the data.
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

    private NetworkTableInstance inst;
    private NetworkTable coneTable;
    private NetworkTable cubeTable;

    /**
     * @return a list of rectangles outlining the cones in the camera's vision.
     */
    public ArrayList<Rect> getConeBounds() {
        ArrayList<Rect> positions = new ArrayList<>();
        for (int i = 0; i < coneXPositions.length; i++) {
            positions.add(new Rect((int) coneXPositions[i], (int) coneYPositions[i], (int) coneWidths[i], (int) coneHeights[i]));
        }
        return positions;
    }

    /**
     * @return a list of rectangles outlining the cubes in the camera's vision.
     */
    public ArrayList<Rect> getCubeBounds() {
        ArrayList<Rect> positions = new ArrayList<>();
        for (int i = 0; i < cubeXPositions.length; i++) {
            positions.add(new Rect((int) cubeXPositions[i], (int) cubeYPositions[i], (int) cubeWidths[i], (int) cubeHeights[i]));
        }
        return positions;
    }

    /**
     * @return the total number of objects in the camera's vision (cones or cubes).
     */
    public int getTotalObjectCount() {
        return cubeXPositions.length + cubeYPositions.length;
    }

    /**
     * @return the total number of cubes in the camera's vision
     */
    public int getCubeCount() {
        return cubeXPositions.length;
    }

    /**
     * @return the total number of cubes in the camera's vision
     */
    public int getConeCount() {
        return coneXPositions.length;
    }

    /**
     * Gets the data from the networktables. Should be run every time camera data is needed.
     */
    public void update() {
        inst = NetworkTableInstance.getDefault();
        cubeTable = inst.getTable("RPI").getSubTable("vision").getSubTable("cubes");
        cubeTable = inst.getTable("RPI").getSubTable("vision").getSubTable("cones");

        cubeXPositions = cubeTable.getEntry("xPositions").getDoubleArray(new double[10]);
        cubeYPositions = cubeTable.getEntry("yPositions").getDoubleArray(new double[10]);
        cubeWidths = cubeTable.getEntry("widths").getDoubleArray(new double[10]);
        cubeHeights = cubeTable.getEntry("heights").getDoubleArray(new double[10]);

        coneXPositions = coneTable.getEntry("xPositions").getDoubleArray(new double[10]);
        coneYPositions = coneTable.getEntry("yPositions").getDoubleArray(new double[10]);
        coneWidths = coneTable.getEntry("widths").getDoubleArray(new double[10]);
        coneHeights = coneTable.getEntry("heights").getDoubleArray(new double[10]);
    }
}