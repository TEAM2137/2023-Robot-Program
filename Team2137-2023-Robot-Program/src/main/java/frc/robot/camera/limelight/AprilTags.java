package frc.robot.camera.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTags {
    
    private static NetworkTable table;
    private static NetworkTableEntry botpose;
    private static double[] pose;

    public static final double fieldSizeX = 651.22;
    public static final double fieldSizeY = 319;

    private static double posX;
    private static double posY;

    private static double velocityX;
    private static double velocityY;

    private static double lastPosX;
    private static double lastPosY; 

    private static double rotation;

    public static void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botpose = table.getEntry("botpose");
    }

    /**
     * Updates the position of the robot using the aprilTags
     */
    public static void updateValues() {
        pose = botpose.getDoubleArray(new double[6]);

        double poseX = pose[0];
        double poseY = pose[1];

        rotation = pose[5];

        posX = (pose[0] * 39.37) + (fieldSizeX / 2);
        posY = (pose[1] * 39.37) + (fieldSizeY / 2);

        velocityX = posX - lastPosX;
        velocityY = posY - lastPosY;

        lastPosX = posX;
        lastPosY = posY;
    
        // Post botpose to smart dashboard
        SmartDashboard.putNumber("Field Position X", poseX);
        SmartDashboard.putNumber("Field Position Y", poseY);

        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);

        SmartDashboard.putNumber("Robot Rotation", rotation);
    }

    /**
     * @return the x position of the robot relative to the bottom left corner of the field (inches)
     */
    public static double getX() {
        return posX;
    }

    /**
     * @return the amount the robot moved on the x axis since the last update (inches)
     */
    public static double getXVelocity() {
        return velocityX;
    }
    
    /**
     * @return the y position of the robot relative to the bottom left corner of the field (inches)
     */
    public static double getY() {
        return posY;
    }

    /**
     * @return the amount the robot moved on the y axis since the last update (inches)
     */
    public static double getYVelocity() {
        return velocityY;
    }

     /**
     * @return the z rotation of the robot (degrees)
     */
    public static double getRotation() {
        return rotation;
    }

}
