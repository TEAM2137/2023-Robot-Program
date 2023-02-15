package frc.robot.camera.objects;

import java.util.ArrayList;

import org.opencv.core.Rect;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Uses GRIP to get the positions of objects (cones/cubes) on a camera's vision.
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

    public void update() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        coneXPositions = table.getEntry("conePositionsX").getDoubleArray(new double[10]);
        coneYPositions = table.getEntry("conePositionsY").getDoubleArray(new double[10]);
        cubeXPositions = table.getEntry("cubePositionsX").getDoubleArray(new double[10]);
        cubeYPositions = table.getEntry("cubePositionsY").getDoubleArray(new double[10]);
        coneWidths = table.getEntry("coneWidths").getDoubleArray(new double[10]);
        coneHeights = table.getEntry("coneHeights").getDoubleArray(new double[10]);
        cubeWidths = table.getEntry("cubeWidths").getDoubleArray(new double[10]);
        cubeHeights = table.getEntry("cubeHeights").getDoubleArray(new double[10]);
    }
}