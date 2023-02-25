package frc.robot.camera.limelight;

import org.opencv.core.Point;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Uses the limelight NetworkTable data to detect information about the reflective tape pipeline.
 */
public class ReflectiveTape {
    
    private static NetworkTable table;

    // Amount of targets
    private static int tv;
    // Horizontal/vertical offset
    private static int tx;
    private static int ty;

    public static void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight-rtape");
    }

    public static void updateValues() {
        tv = (int) table.getEntry("tv").getDouble(0);
        tx = (int) table.getEntry("tx").getDouble(0);
        ty = (int) table.getEntry("ty").getDouble(0);
    }

    /**
     * @return <code>true</code> if the limelight has one or more targets, and <code>false</code> if it doesn't.
     */
    public static boolean hasTarget() {
        return tv > 0;
    }

    /**
     * @return a point containing the position of the target closest to the center of the
     * limelight's vision.
     * <p> Returns (0, 0) if there is no target. Use <code>hasTarget()</code> to detect if there is a 
     * target being seen by the limelight.
     */
    public static Point targetPosition() {
        return new Point(tx, ty);
    }
}