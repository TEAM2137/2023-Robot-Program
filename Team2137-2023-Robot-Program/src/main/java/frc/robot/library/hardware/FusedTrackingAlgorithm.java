package frc.robot.library.hardware;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.library.Constants;
import frc.robot.library.hardware.deadReckoning.DeadWheelActiveTracking;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.CartesianValue;

import java.util.LinkedList;
import java.util.function.Consumer;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class FusedTrackingAlgorithm {

    private class Reading {
        public CartesianValue<Rotation2d> gyroAngleReading;
        public CartesianValue<Velocity> gyroSpeedReading;
        public CartesianValue<Acceleration> gyroAccelReading;
        public CartesianValue<Velocity> desiredVelocities;
        public CartesianValue<Distance> robotCenterOfGravity;
        public SwerveModuleState[] swerveModuleStates;
        public Pose2d visionReading;

        public Reading(CartesianValue<Rotation2d> _GyroAngleReading, CartesianValue<Velocity> _GyroSpeedReading, CartesianValue<Acceleration> _GyroAcceleration, CartesianValue<Velocity> _DesiredVelocities, CartesianValue<Distance> _RobotCenterOfGravity, SwerveModuleState[] _SwerveModuleStates, Pose2d _VisionReadings) {
            gyroAngleReading = _GyroAngleReading;
            gyroSpeedReading = _GyroSpeedReading;
            gyroAccelReading = _GyroAcceleration;
            desiredVelocities = _DesiredVelocities;
            robotCenterOfGravity = _RobotCenterOfGravity;
            swerveModuleStates = _SwerveModuleStates;
            visionReading = _VisionReadings;
        }
    }

    private static final int sizeOfPastReadings = 10;
    CartesianValue<Distance> robotDimensions;
    private static double robotMass = 45.0;
    private static final double dblMaxRobotAccelerationCoast = -3;
    private static final double dblMaxRobotAccelerationBreak = -10;
    LinkedList<Reading> pastReadings = new LinkedList<>();

    private double currentX;
    private double currentY;

    private Consumer<Acceleration> tipEvent;

    public FusedTrackingAlgorithm(double _robotMass, Consumer<Acceleration> _tipEvent) {
        robotMass = _robotMass;
        tipEvent = _tipEvent;
    }

    public void update(DeadWheelActiveTracking deadWheelActiveTracking, CartesianValue<Rotation2d> gyroAngleReading, CartesianValue<Velocity> gyroSpeedReading, CartesianValue<Acceleration> gyroAccelerationReading, CartesianValue<Velocity> desiredVelocities, CartesianValue<Velocity> currentVelocity, CartesianValue<Distance> centerOfGravity, SwerveModuleState[] moduleStates, Pose2d visionReading) {
        if(pastReadings.size() == sizeOfPastReadings)
            pastReadings.remove(); //If full remove the first item

        pastReadings.add(new Reading(gyroAngleReading, gyroSpeedReading, gyroAccelerationReading, desiredVelocities, centerOfGravity, moduleStates, visionReading)); //Add the new item
    }

    /**
     * Uses a linear line of best fit to find the Acceleration from the applied voltage to the motor.
     * @param voltages
     * @return
     */
    private static Acceleration[] getSwerveFalconAcceleration(double[] voltages) {
        Acceleration[] returner = new Acceleration[voltages.length];

        for(int i = 0; i < voltages.length; i++) {
            returner[i] = new Acceleration((-1.42253 * voltages[i]) + 16.3615, METER_PER_SECOND2);
        }

        return returner;
    }

    public static double calculateTipAcceleration(Translation2d cog, Translation2d tipPoint, Rotation2d gyroDirectionalAngle) {
        double horizontalDistance = cog.getX() - tipPoint.getX();
        double verticalDistance = cog.getY() - tipPoint.getY();
        Rotation2d theta = Rotation2d.fromRadians(Math.atan(horizontalDistance / verticalDistance)).minus(gyroDirectionalAngle);

        double top = -9.81 * (Math.cos(theta.getRadians()) - 1);
        double bottom = cog.getDistance(tipPoint) * Math.cos((Math.PI / 2.0) - theta.getRadians());
        return top / bottom;
    }

    private CartesianValue<Distance> calculateCenterOfGravityPosition() {
        Reading latest = getLatestReading();
        return Constants.rotate3DInertialFrame(latest.robotCenterOfGravity, latest.gyroAngleReading);
    }

    public static void tipDetection(CartesianValue<Velocity> desiredVelocities, CartesianValue<Velocity> currentVelocity, CartesianValue<Rotation2d> gyroAngleReading, CartesianValue<Distance> staticCenterOfGravity, CartesianValue<Distance> robotDimensions) {
        CartesianValue<Distance> centerOfGravity = Constants.rotate3DInertialFrame(staticCenterOfGravity, gyroAngleReading);
        //Can only tip straight directions
        Rotation2d gyroDirectionAngle;
        double actualVelocityDifference;
        Translation2d actualCenterOfGravity;
        Translation2d tipPoint;

        Translation2d velocityDifferences = new Translation2d(desiredVelocities.getX().getValue(METER_PER_SECOND) - currentVelocity.getX().getValue(METER_PER_SECOND), desiredVelocities.getY().getValue(METER_PER_SECOND) - currentVelocity.getY().getValue(METER_PER_SECOND));

        if(velocityDifferences.getX() > velocityDifferences.getY()) {
            if(velocityDifferences.getX() > 0) {
                tipPoint = new Translation2d(robotDimensions.getX().getValue(METER) * 0.5, 0);
            } else {
                tipPoint = new Translation2d(robotDimensions.getX().getValue(METER) * -0.5, 0);
            }

            gyroDirectionAngle = gyroAngleReading.getY();
            actualCenterOfGravity = new Translation2d(centerOfGravity.getX().getValue(METER), centerOfGravity.getZ().getValue(METER));
            actualVelocityDifference = velocityDifferences.getX();
        } else {
            if(velocityDifferences.getY() > 0) {
                tipPoint = new Translation2d(robotDimensions.getY().getValue(METER) * 0.5, 0);
            } else {
                tipPoint = new Translation2d(robotDimensions.getY().getValue(METER) * -0.5, 0);
            }

            gyroDirectionAngle = gyroAngleReading.getX();
            actualCenterOfGravity = new Translation2d(centerOfGravity.getY().getValue(METER), centerOfGravity.getZ().getValue(METER));
            actualVelocityDifference = velocityDifferences.getY();
        }

        double tipAcceleration = calculateTipAcceleration(actualCenterOfGravity, tipPoint, gyroDirectionAngle);

        SmartDashboard.putNumber("TipAcceleration", tipAcceleration);

        //if( > tipAcceleration) {
            System.out.println("TIPPPPPP");
//            tipEvent.accept(new Acceleration(tipAcceleration, METER_PER_SECOND2));
        //}
    }

    private void bumpDetection() {

    }

    private Reading getLatestReading() {
        return pastReadings.getLast();
    }
}
