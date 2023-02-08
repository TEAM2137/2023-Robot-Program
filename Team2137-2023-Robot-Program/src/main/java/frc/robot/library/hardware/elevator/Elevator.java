package frc.robot.library.hardware.elevator;

import org.w3c.dom.Element;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;

public class Elevator extends EntityGroup {

    private FileLogger logger;
    private double distance;
    private double rotation;

    public Elevator(Element element, int depth, EntityGroup parent, FileLogger fileLogger) {
        super(element, depth, parent, fileLogger);
        logger = fileLogger;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Distance (inches)", distance);
        SmartDashboard.putNumber("Rotation (degrees)", rotation);
    }

    /**
     * Rotates the elevator a set amount
     * @param degrees the amount to rotate (degrees)
     */
    public void rotateArmDegrees(double degrees) { }

    /**
     * Rotates the elevator at a certain speed
     * @param speed how fast the arm is rotated
     */
    public void rotateArmSpeed(double speed) { }

    /**
     * Extends the elevator at a set distance
     * @param distance the distance to move (inches)
     */
    public void extendArmDistance(double distance) { }

    /**
     * Extends the arm at a certain speed
     * @param speed how fast the elevator is extended
     */
    public void extendArmSpeed(double speed) { }

    /**
     * @return the distance the elevator is currently extended (inches)
     */
    public double getExtendedDistance() {
        return distance;
    }

    /**
     * @return the amount the elevator is currently rotated (degrees)
     */
    public double getRotation() {
        return rotation;
    }

    public void logElevatorState() {
        StringBuilder builder = new StringBuilder();

        builder.append("QES~");
        builder.append(distance);
        builder.append(rotation);

        logger.writeLine(builder.toString());
    }
    
}
