package frc.robot.library.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.library.Constants;
import frc.robot.library.hardware.deadReckoning.DeadWheelActiveTracking;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.CartesianValue;
import frc.robot.library.units.UnitContainers.Point2d;
import frc.robot.library.units.UnitContainers.Pose2d;

import java.awt.*;
import java.util.LinkedList;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class FusedTrackingAlgorithm {

    private Point2d<Distance> currentEstimatedRobotPosition;

    private Callable<Point2d<Distance>> mOdometryCallback;
    private Callable<Point2d<Distance>> mAprilTagCallback;

    private static double mAprilTagWeight = 0.5;

    public FusedTrackingAlgorithm(Callable<Point2d<Distance>> odometryReading, Callable<Point2d<Distance>> aprilTagReading, int updatePeriod) {
        mOdometryCallback = odometryReading;
        mAprilTagCallback = aprilTagReading;

        Robot.threadPoolExecutor.scheduleAtFixedRate(this::update, 0, 50, TimeUnit.MILLISECONDS);
    }

    private void update() {
        try {
            Point2d<Distance> odometryResults = mOdometryCallback.call();
            Point2d<Distance> apriltagResults = mAprilTagCallback.call();

            Distance estimatedX = odometryResults.getX().times(mAprilTagWeight).add(apriltagResults.getX().times(1 - mAprilTagWeight));
            Distance estimatedY = odometryResults.getY().times(mAprilTagWeight).add(apriltagResults.getY().times(1 - mAprilTagWeight));

            currentEstimatedRobotPosition = new Point2d<Distance>(estimatedX, estimatedY);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
