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
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class FusedTrackingAlgorithm {

    private Point2d<Distance> currentEstimatedRobotPosition;

    private ArrayList<Callable<Point2d<Distance>>> mPositionCallbacks;
    private ArrayList<Double> mPositionWeights;

    public FusedTrackingAlgorithm(ArrayList<Callable<Point2d<Distance>>> positionCallbacks, ArrayList<Double> weights, int updatePeriod) {
        mPositionCallbacks = positionCallbacks;
        mPositionWeights = weights;

        Robot.threadPoolExecutor.scheduleAtFixedRate(this::update, 0, updatePeriod, TimeUnit.MILLISECONDS);
    }

    private void update() {
        try {
            Distance estimatedX = new Distance(0, FOOT);
            Distance estimatedY = new Distance(0, FOOT);

            for(int i = 0; i < mPositionCallbacks.size(); i++) {
                Point2d<Distance> results = mPositionCallbacks.get(i).call();

                estimatedX = estimatedX.add(results.getX().times(mPositionWeights.get(i)));
                estimatedY = estimatedY.add(results.getY().times(mPositionWeights.get(i)));
            }

            currentEstimatedRobotPosition = new Point2d<Distance>(estimatedX, estimatedY);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}
