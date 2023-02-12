package frc.robot.library.hardware.swerve;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.*;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitContainers.Vector2d;
import org.ejml.simple.SimpleMatrix;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.Time.TimeUnits.MILLISECONDS;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class SwerveKinematics<T extends Unit<?, ? extends UnitEnum>> {
    private static SimpleMatrix forwardKinematicForm;
    private static SimpleMatrix inverseKinematicForm;

    private static Vector2d<Distance> latestRobotPosition;
    private static Vector2d<Velocity> latestRobotVelocity;
    private static double lastTime;

    public SwerveKinematics(Distance robotXLength, Distance robotYLength) {
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

        latestRobotPosition = new Vector2d<Distance>(new Distance(0, FOOT), new Distance(0, FOOT));
        latestRobotVelocity = new Vector2d<Velocity>(new Velocity(0, FEET_PER_SECOND), new Velocity(0, FEET_PER_SECOND));

        forwardKinematicForm = inverseKinematicForm.pseudoInverse();
        lastTime = System.currentTimeMillis();
    }

    private SimpleMatrix getForwardKinematic(SimpleMatrix moduleComponentVelocity) {
        return forwardKinematicForm.mult(moduleComponentVelocity);
    }

    private SimpleMatrix getInverseKinematic(SimpleMatrix robotComponentVelocity) {
        return inverseKinematicForm.mult(robotComponentVelocity);
    }

    public Vector2d<Distance> updateSwerveKinematics(SwerveModuleState[] currentVelocityState) {
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

        Vector2d<Velocity> velocityVector = new Vector2d<Velocity>(robotComponents.get(0, 0), robotComponents.get(1, 0), METER_PER_SECOND);

        Time dt = new Time(System.currentTimeMillis() - lastTime, MILLISECONDS);
        latestRobotPosition.mutableAdd((Vector2d<Distance>) velocityVector.times(dt));

        lastTime = System.currentTimeMillis();

        return latestRobotPosition;
    }

    public Vector2d<Distance> updateSwerveKinematics(SwerveModuleState[] currentVelocityState, SwerveModuleState[] currentAccelerationState) {
        Vector2d<Velocity> module1Components = null;
        Vector2d<Velocity> module2Components = null;
        Vector2d<Velocity> module3Components = null;
        Vector2d<Velocity> module4Components = null;

        Vector2d<Acceleration> module1AccelComponent = null;
        Vector2d<Acceleration> module2AccelComponent = null;
        Vector2d<Acceleration> module3AccelComponent = null;
        Vector2d<Acceleration> module4AccelComponent = null;

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

        for(SwerveModuleState state : currentAccelerationState) {
            switch(state.getPosition()) {
                case LEFT_FRONT:
                    module1AccelComponent = new Vector2d<Acceleration>(state.getAcceleration(), state.getRotationalAccel());
                    break;
                case RIGHT_FRONT:
                    module2AccelComponent = new Vector2d<Acceleration>(state.getAcceleration(), state.getRotationalAccel());
                    break;
                case LEFT_BACK:
                    module3AccelComponent = new Vector2d<Acceleration>(state.getAcceleration(), state.getRotationalAccel());
                    break;
                case RIGHT_BACK:
                    module4AccelComponent = new Vector2d<Acceleration>(state.getAcceleration(), state.getRotationalAccel());
                    break;
            }
        }

//        SimpleMatrix current = new SimpleMatrix(new double[][] {
//                new double[] { module1Components.getX().getValueInPrimaryUnit() },
//                new double[] { module1Components.getY().getValueInPrimaryUnit() },
//                new double[] { module2Components.getX().getValueInPrimaryUnit() },
//                new double[] { module2Components.getY().getValueInPrimaryUnit() },
//                new double[] { module3Components.getX().getValueInPrimaryUnit() },
//                new double[] { module3Components.getY().getValueInPrimaryUnit() },
//                new double[] { module4Components.getX().getValueInPrimaryUnit() },
//                new double[] { module4Components.getY().getValueInPrimaryUnit() },
//        });
        SimpleMatrix current = new SimpleMatrix(new double[][] {
                new double[] { module1Components.getX().getValueInPrimaryUnit(), module1AccelComponent.getX().getValueInPrimaryUnit() },
                new double[] { module1Components.getY().getValueInPrimaryUnit(), module1AccelComponent.getY().getValueInPrimaryUnit() },
                new double[] { module2Components.getX().getValueInPrimaryUnit(), module2AccelComponent.getX().getValueInPrimaryUnit() },
                new double[] { module2Components.getY().getValueInPrimaryUnit(), module2AccelComponent.getY().getValueInPrimaryUnit() },
                new double[] { module3Components.getX().getValueInPrimaryUnit(), module3AccelComponent.getX().getValueInPrimaryUnit() },
                new double[] { module3Components.getY().getValueInPrimaryUnit(), module3AccelComponent.getY().getValueInPrimaryUnit() },
                new double[] { module4Components.getX().getValueInPrimaryUnit(), module4AccelComponent.getX().getValueInPrimaryUnit() },
                new double[] { module4Components.getY().getValueInPrimaryUnit(), module4AccelComponent.getY().getValueInPrimaryUnit() },
        });

        SimpleMatrix robotComponents = getForwardKinematic(current);

        Vector2d<Velocity> velocityVector = new Vector2d<Velocity>(robotComponents.get(0, 0), robotComponents.get(1, 0), METER_PER_SECOND);
        Vector2d<Acceleration> accelerationVector = new Vector2d<Acceleration>(robotComponents.get(0, 1), robotComponents.get(1, 1), METER_PER_SECOND2);

        Time dt = new Time(System.currentTimeMillis() - lastTime, MILLISECONDS);
        latestRobotPosition.mutableAdd((Vector2d<Distance>) velocityVector.times(dt));
        latestRobotVelocity.mutableAdd((Vector2d<Velocity>) accelerationVector.times(dt));

        lastTime = System.currentTimeMillis();

        return latestRobotPosition;
    }

    //Returns Left Front, Right Front, Right Back, Left Back
    public SwerveModuleState[] getSwerveModuleState(Unit<?, ? extends UnitEnum> x, Unit<?, ? extends UnitEnum> y, Unit<?, ? extends UnitEnum> omega) {
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
        } catch (Exception e) {
            e.printStackTrace();
        }

        return moduleStates;
    }

    private SwerveModuleState getStateValue(double x, double y, UnitEnum unit, UnitEnum angularUnit, SwerveModuleState.SwerveModulePositions pos) {
        double rad = Math.atan2(y, x);
        double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        Rotation2d offset = Rotation2d.fromDegrees(90);
        return new SwerveModuleState(UnitUtil.create(mag, unit), UnitUtil.create(rad + offset.getRadians(), angularUnit), pos);
    }

    public Vector2d<Velocity> getCurrentRobotVelocity() {
        return latestRobotVelocity;
    }

    public Vector2d<Distance> getCurrentRobotPosition() {
        return latestRobotPosition;
    }
}
