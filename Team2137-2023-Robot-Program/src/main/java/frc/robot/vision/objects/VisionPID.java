package frc.robot.vision.objects;

import org.opencv.core.Point;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.library.Constants;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;

public class VisionPID {
    
    private SwerveDrivetrain mDrivetrain;
    private PIDController drivePIDController = new PIDController(0.7, 0.1, 0.3);

    /**
     * Creates a new vision pid with an offset defaulted to 0
     */
    public VisionPID() {
        drivePIDController.setSetpoint(0);
    }

    /**
     * Creates a new vision pid with the specified offset
     * @param offset makes the robot go more left or right of the targeted object, in pixels
     */
    public VisionPID(double offset) {
        drivePIDController.setSetpoint(offset);
    }

    /**
     * Moves the robot to horizontally center on the position of an object
     * in the camera's vision
     * @param objectPosition the position of the object that is being targeted
     */
    public synchronized void move(Point objectPosition) {
        // PID calculations
        double xVelocity = Math.max(-1, Math.min(1, drivePIDController.calculate(objectPosition.x)));

        // Movement
        SwerveDrivetrain swerve = mDrivetrain;
        if (Math.abs(objectPosition.x) < 10) {
            swerve.setSwerveModuleStates(swerve.calculateSwerveMotorSpeeds(
                xVelocity, 0, 0, 1, 1, Constants.DriveControlType.VELOCITY));
        } else {
            swerve.setSwerveModuleStates(swerve.calculateSwerveMotorSpeeds(
                0, 0, 0, 1, 1, Constants.DriveControlType.VELOCITY));
        }
    }

}
