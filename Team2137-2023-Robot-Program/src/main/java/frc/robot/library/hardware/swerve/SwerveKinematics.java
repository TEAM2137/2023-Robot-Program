package frc.robot.library.hardware.swerve;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.*;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.UnitContainers.Point2d;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitContainers.Vector2d;
import org.ejml.simple.SimpleMatrix;

import java.io.File;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.Time.TimeUnits.MILLISECONDS;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.*;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class SwerveKinematics {
    private static SimpleMatrix forwardKinematicForm;
    private static SimpleMatrix inverseKinematicForm;

    private static Point2d<Distance> latestRobotPosition;
    public static double lastTime;
    public static double scaleFactorDistance;


    public SwerveKinematics(Distance robotXLength, Distance robotYLength, double scaleFactor) {

        inverseKinematicForm = new SimpleMatrix(new double[][] {
                //Left Front
                new double[] { 1, 0, -robotYLength.getValue(METER)},
                new double[] { 0, 1, -robotXLength.getValue(METER)},
                //Right Front
                new double[] { 1, 0, -robotYLength.getValue(METER)},
                new double[] { 0, 1, robotXLength.getValue(METER)},
                //Left Back
                new double[] { 1, 0, robotYLength.getValue(METER)},
                new double[] { 0, 1, -robotYLength.getValue(METER)},
                //Right Back
                new double[] { 1, 0, robotYLength.getValue(METER)},
                new double[] { 0, 1, robotXLength.getValue(METER)},
        });

        latestRobotPosition = new Point2d<Distance>(new Distance(0, FOOT), new Distance(0, FOOT));

        forwardKinematicForm = inverseKinematicForm.pseudoInverse();
        lastTime = System.currentTimeMillis();
        scaleFactor = scaleFactorDistance;
    }

    private SimpleMatrix getForwardKinematic(SimpleMatrix moduleComponentVelocity) {
        return forwardKinematicForm.mult(moduleComponentVelocity);
    }

    private SimpleMatrix getInverseKinematic(SimpleMatrix robotComponentVelocity) {
        return inverseKinematicForm.mult(robotComponentVelocity);
    }

    public Vector2d<Velocity> getSwerveKinematics(SwerveModuleState[] currentVelocityState) {
        Vector2d<Velocity> module1Components = null;
        Vector2d<Velocity> module2Components = null;
        Vector2d<Velocity> module3Components = null;
        Vector2d<Velocity> module4Components = null;

        for(SwerveModuleState state : currentVelocityState) {
            switch(state.getPosition()) {
                case LEFT_FRONT:
                    module1Components = new Vector2d<Velocity>(state.getSpeed2d(), state.getRotation2d());
                    break;
                case RIGHT_FRONT:
                    module2Components = new Vector2d<Velocity>(state.getSpeed2d(), state.getRotation2d());
                    break;
                case LEFT_BACK:
                    module3Components = new Vector2d<Velocity>(state.getSpeed2d(), state.getRotation2d());
                    break;
                case RIGHT_BACK:
                    module4Components = new Vector2d<Velocity>(state.getSpeed2d(), state.getRotation2d());
                    break;
            }
        }

        SimpleMatrix current = new SimpleMatrix(new double[][] {
                new double[] { module1Components.getX().getValueInPrimaryUnit() },
                new double[] { module1Components.getY().getValueInPrimaryUnit() },
                new double[] { module2Components.getX().getValueInPrimaryUnit() },
                new double[] { module2Components.getY().getValueInPrimaryUnit() },
                new double[] { module3Components.getX().getValueInPrimaryUnit() },
                new double[] { module3Components.getY().getValueInPrimaryUnit() },
                new double[] { module4Components.getX().getValueInPrimaryUnit() },
                new double[] { module4Components.getY().getValueInPrimaryUnit() },
        });

        SimpleMatrix robotComponents = getForwardKinematic(current);

//        Vector2d<Velocity> velocityVector = new Vector2d<Velocity>(robotComponents.get(1, 0), -robotComponents.get(0, 0), METER_PER_SECOND);
        Vector2d<Velocity> velocityVector = new Vector2d<Velocity>(-robotComponents.get(0, 0), -robotComponents.get(1, 0), METER_PER_SECOND);

        SmartDashboard.putNumber("XVel", velocityVector.getX().getValue(FEET_PER_SECOND));
        SmartDashboard.putNumber("YVel", velocityVector.getY().getValue(FEET_PER_SECOND));

        return velocityVector;
    }

    public void resetSwerveKinematics() {
        lastTime = System.currentTimeMillis();
        latestRobotPosition = new Point2d<Distance>(new Distance(0, INCH), new Distance(0, INCH));
    }

    public Point2d<Distance> updateSwerveKinematics(SwerveModuleState[] currentVelocityState) {
        Vector2d<Velocity> velocityVector = getSwerveKinematics(currentVelocityState);

        Time dt = new Time(System.currentTimeMillis() - lastTime, MILLISECONDS);
        Vector2d<Distance> result = (Vector2d<Distance>) velocityVector.times(dt);
        result.scale(scaleFactorDistance);
        latestRobotPosition = new Point2d<Distance>(latestRobotPosition.getX().add(result.getX()), latestRobotPosition.getY().add(result.getY()));

        lastTime = System.currentTimeMillis();

        return latestRobotPosition;
    }

    //Returns Left Front, Right Front, Right Back, Left Back
    public SwerveModuleState[] getSwerveModuleState(Unit<?, ? extends UnitEnum> x, Unit<?, ? extends UnitEnum> y, Unit<?, ? extends UnitEnum> omega, Unit<?, ? extends UnitEnum> maximum) {
        UnitEnum unit = x.getPrimaryUnit();
        UnitEnum unitRotation = omega.getPrimaryUnit();

        SimpleMatrix desiredComponentsMatrix = new SimpleMatrix(new double[][] {
                new double[] { x.getValueInPrimaryUnit() },
                new double[] { y.getValueInPrimaryUnit() },
                new double[] { omega.getValueInPrimaryUnit() }
        });

        SimpleMatrix states = getInverseKinematic(desiredComponentsMatrix);

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        try {
            moduleStates[0] = getStateValue(states.get(0, 0), states.get(1, 0), unit, unitRotation, SwerveModuleState.SwerveModulePositions.LEFT_FRONT);
            moduleStates[1] = getStateValue(states.get(2, 0), states.get(3, 0), unit, unitRotation, SwerveModuleState.SwerveModulePositions.RIGHT_FRONT);
            moduleStates[2] = getStateValue(states.get(4, 0), states.get(5, 0), unit, unitRotation, SwerveModuleState.SwerveModulePositions.LEFT_BACK);
            moduleStates[3] = getStateValue(states.get(6, 0), states.get(7, 0), unit, unitRotation, SwerveModuleState.SwerveModulePositions.RIGHT_BACK);

//            moduleStates = normalizeStates(moduleStates, maximum);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return moduleStates;
    }

//    private SwerveModuleState[] normalizeStates(SwerveModuleState[] states, Unit<?, ? extends UnitEnum> max) {
//        double maxValue = 0;
//
//        for(SwerveModuleState state : states) {
//            double value = Math.abs(state.getDirectionalValue().getValueInPrimaryUnit());
//            if(value > maxValue)
//                maxValue = value;
//        }
//
//        if (maxValue < max.getValueInPrimaryUnit())
//            return states;
//
//        SwerveModuleState[] newStates = new SwerveModuleState[states.length];
//
//        for(int i = 0; i < newStates.length; i++) {
//            newStates[i] = new SwerveModuleState((Unit<?, ? extends UnitEnum>) states[i].getDirectionalValue().divide(maxValue), states[i].getRotation2d(), states[i].getPosition());
//        }
//
//        return newStates;
//    }

    public void reset(Point2d<Distance> newPosition) {
        latestRobotPosition = new Point2d<Distance>(newPosition.getX(), newPosition.getY());
    }

    private SwerveModuleState getStateValue(double x, double y, UnitEnum unit, UnitEnum angularUnit, SwerveModuleState.SwerveModulePositions pos) {
        double rad = Math.atan2(y, x);
        double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        Rotation2d offset = Rotation2d.fromDegrees(90);
        return new SwerveModuleState(UnitUtil.create(mag, unit), UnitUtil.create(rad + offset.getRadians(), angularUnit), pos);
    }

    public synchronized Point2d<Distance> getCurrentRobotPosition() {
        return latestRobotPosition;
    }
}
