//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package frc.robot.library.hardware.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.objects.Gyro;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.library.Constants;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngleUnit;
import frc.robot.library.units.Number;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.deadReckoning.DeadWheelActiveTracking;
import frc.robot.library.hardware.swerve.module.SwerveModule;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitContainers.CartesianValue;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitContainers.Vector2d;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;
import org.ejml.simple.SimpleMatrix;
import org.w3c.dom.Element;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.*;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;


public class SwerveDrivetrain extends EntityGroup implements DriveTrain {

    public SwerveModule leftFrontModule;
    public SwerveModule leftBackModule;
    public SwerveModule rightFrontModule;
    public SwerveModule rightBackModule;

    private Distance robotLength;
    private Distance robotWidth;

    public SwerveKinematics<Number> swerveKinematics;
    private DeadWheelActiveTracking mDeadWheelActiveTracking;

    public Motor.MotorTypes driveTrainType;

    private final FileLogger logger;

    private final PigeonIMU pigeonIMU;

    public SwerveDrivetrain(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        logger = fileLogger;
        logger.setTag("SwerveDriveTrainConstructor");

        robotLength = (Distance) Robot.settingsEntityGroup.getEntity("RobotLength");
        if(robotLength == null) {
            logger.writeEvent(1, FileLogger.EventType.Error, "RobotLength Element is not found! Using default 28\"");
            robotLength = new Distance(28, INCH);
        }

        robotWidth = (Distance) Robot.settingsEntityGroup.getEntity("RobotWidth");
        if(robotWidth == null) {
            logger.writeEvent(1, FileLogger.EventType.Error, "RobotWidth Element is not found! Using default 28\"");
            robotWidth = new Distance(28, INCH);
        }

        logger.writeEvent(6, FileLogger.EventType.Debug, "Creating Swerve Kinematics class...");
        swerveKinematics = new SwerveKinematics<>(robotWidth, robotLength);

        logger.writeEvent(6, FileLogger.EventType.Debug, "Finding swerve modules from EntityGroup");
        try {
            leftFrontModule = (SwerveModule) getChildEntityGroup("LeftFront");
            leftBackModule = (SwerveModule) getChildEntityGroup("LeftBack");
            rightFrontModule = (SwerveModule) getChildEntityGroup("RightFront");
            rightBackModule = (SwerveModule) getChildEntityGroup("RightBack");
        } catch (Exception e) {
            logger.writeEvent(0, FileLogger.EventType.Error, "Failed to find or convert to Swerve Modules!!");
            logger.writeLine(e.getMessage());
        }

        logger.writeEvent(6, FileLogger.EventType.Debug, "Creating Pigeon Gyro...");
        Gyro gyro = (Gyro) getEntity("Pigeon");
        pigeonIMU = new PigeonIMU(gyro.getID());
        pigeonIMU.configFactoryDefault();
        pigeonIMU.setFusedHeading(gyro.getOffset());

        this.addSubsystemCommand("rawDrive", this::rawDrive);
    }

    public void rawDrive(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT) {
            SwerveModuleState[] states = calculateSwerveMotorSpeedsFieldCentric(step.getXDistance(), -step.getYDistance(), step.getParm(1));

            this.setSwerveModuleStates(states);
        }
    }

    @Override
    public void periodic() {
        SwerveModuleState lfVelState = leftFrontModule.getSwerveModuleState();
        SwerveModuleState lbVelState = leftBackModule.getSwerveModuleState();
        SwerveModuleState rfVelState = rightFrontModule.getSwerveModuleState();
        SwerveModuleState rbVelState = rightBackModule.getSwerveModuleState();

        Vector2d<Distance> dist = swerveKinematics.updateSwerveKinematics(new SwerveModuleState[] { lfVelState, lbVelState, rfVelState, rbVelState });

        SmartDashboard.putNumber("Dist X", dist.getX().getValue(FOOT));
        SmartDashboard.putNumber("Dist Y", dist.getX().getValue(FOOT));
    }

    @Override
    public Pose2d<Distance> getCurrentOdometryPosition() {
        Vector2d<Distance> pos = swerveKinematics.getCurrentRobotPosition();

        return new Pose2d<>(pos.getX(), pos.getY(), getAngle());
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(Unit<?, ? extends UnitEnum> xMag, Unit<?, ? extends UnitEnum> yMag, AngleUnit<?, ? extends UnitEnum> rMag) {
        SimpleMatrix fieldCentric = Constants.convertFrame(getAngle(), Constants.createFrameMatrix(xMag.getValueInPrimaryUnit(), yMag.getValueInPrimaryUnit(), rMag.getValueInPrimaryUnit()));

        Unit<?, ? extends UnitEnum> x = UnitUtil.create(fieldCentric.get(0, 0), xMag.getPrimaryUnit());
        Unit<?, ? extends UnitEnum> y = UnitUtil.create(fieldCentric.get(1, 0), yMag.getPrimaryUnit());
        Unit<?, ? extends UnitEnum> w = UnitUtil.create(fieldCentric.get(2, 0), rMag.getPrimaryUnit());

        return swerveKinematics.getSwerveModuleState(x, y, w);
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(double xMag, double yMag, double rMag) {
        return calculateSwerveMotorSpeeds(Constants.convertFrame(getAngle(), Constants.createFrameMatrix(xMag, yMag, rMag)), 1, 1, Constants.DriveControlType.RAW);
    }

    @Deprecated
    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(double xMag, double yMag, double rMag, double trackWidth, double wheelBase, Constants.DriveControlType controlType) {
        return calculateSwerveMotorSpeeds(Constants.convertFrame(getAngle(), Constants.createFrameMatrix(xMag, yMag, rMag)), trackWidth, wheelBase, controlType);
    }

    /**
     * input matrix ->
     * [X1]
     * [Y1]
     * [R1]
     *
     * Resultant ->
     * [Left Front Speed]   [Left Front Angle]
     * [Left Back Speed]    [Left Back Angle]
     * [Right Front Speed]  [Right Front Angle]
     * [Right Back Speed]   [Right Back Angle]
     */
    @Deprecated
    public SwerveModuleState[] calculateSwerveMotorSpeeds(SimpleMatrix directions, double trackWidth, double wheelBase, Constants.DriveControlType controlType) {
        return calculateSwerveMotorSpeeds(directions.get(0, 0), directions.get(1, 0), directions.get(2, 0), trackWidth, wheelBase, controlType);
    }

    /**
     * Returns the speeds of the Swerve Drive Train when given the Controller Values
     *
     * Inputs can be velocities for -1 to 1 from the joy stick
     * xD - The left joystick -1 ~ 1
     * yD - The left joystick -1 ~ 1
     * tD - The rightjoy stick or turning button -1 ~ 1
     * @return An Array of Points with x being drive speed and y wheel angle in degree
     */
    @Deprecated
    public SwerveModuleState[] calculateSwerveMotorSpeeds(double xMag, double yMag, double rMag, double axelDistance, double wheelBase, Constants.DriveControlType controlType) {
        //double r = Math.sqrt((axelDistance * axelDistance) + (wheelBase * wheelBase)) / 2.0; //Distance between adjacent wheel

        double a = xMag - (rMag * (axelDistance / 2.0)); // translatedSpeeds[2] * (axleDistance / r) is the ratio of wheel distance from other wheels
        double b = xMag + (rMag * (axelDistance / 2.0));
        double c = yMag - (rMag * (wheelBase / 2.0));
        double d = yMag + (rMag * (wheelBase / 2.0));

        double[][] speeds = new double[][] {
                new double[] {Math.sqrt((b * b) + (d * d)), -Math.atan2(b, d)}, // Left Front
                new double[] {Math.sqrt((a * a) + (d * d)), -Math.atan2(a, d)}, // Left Back
                new double[] {Math.sqrt((b * b) + (c * c)), -Math.atan2(b, c)}, // Right Front
                new double[] {Math.sqrt((a * a) + (c * c)), -Math.atan2(a, c)}, // Right Back
        };

        double joystickRadialValue = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));

        if (joystickRadialValue < 0.07 && Math.abs(rMag) < 0.07){
            speeds[0][1] = leftFrontModule.getModuleGoalAngle().getRadians();
            speeds[1][1] = leftBackModule.getModuleGoalAngle().getRadians();
            speeds[2][1] = rightFrontModule.getModuleGoalAngle().getRadians();
            speeds[3][1] = rightBackModule.getModuleGoalAngle().getRadians();
        }

        if (controlType == Constants.DriveControlType.VELOCITY) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(new Velocity(speeds[0][0], FEET_PER_SECOND), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(new Velocity(speeds[1][0], FEET_PER_SECOND), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(new Velocity(speeds[2][0], FEET_PER_SECOND), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(new Velocity(speeds[3][0], FEET_PER_SECOND), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        } else if(controlType == Constants.DriveControlType.DISTANCE) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(new Distance(speeds[0][0], INCH), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(new Distance(speeds[1][0], INCH), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(new Distance(speeds[2][0], INCH), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(new Distance(speeds[3][0], INCH), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        } else {
            return new SwerveModuleState[]{
                    new SwerveModuleState(speeds[0][0], new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(speeds[1][0], new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(speeds[2][0], new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(speeds[3][0], new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        }
    }

    /**
     * Corrected Angle (0~PI)
     */
    public static double normalizeRadianAngle(double value) {
        value %= Math.PI;
        return value < 0 ? value + Math.PI : value;
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        for(SwerveModuleState state : swerveModuleStates) {
            switch(state.getPosition()) {
                case LEFT_FRONT:
                    this.leftFrontModule.setSwerveModuleState(state);
                    break;
                case LEFT_BACK:
                    this.leftBackModule.setSwerveModuleState(state);
                    break;
                case RIGHT_FRONT:
                    this.rightFrontModule.setSwerveModuleState(state);
                    break;
                case RIGHT_BACK:
                    this.rightBackModule.setSwerveModuleState(state);
                    break;
            }
        }
    }

    /**
     * Turns the controller input into field centric values
     * @param xSpeed - Desired X speed on field
     * @param ySpeed - Desired Y speed on field
     * @param radianPerSecond - Desired Turn speed on field
     * @param robotAngle - Current robot Angle on field
     * @return Array or double {xSpeed, ySpeed, turnSpeed}
     */
    public static double[] toFieldRelativeChassisSpeeds(double xSpeed, double ySpeed, double radianPerSecond, Rotation2d robotAngle) {
        return new double[]{
                -(xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin()),
                -(-xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos()),
                radianPerSecond};
    }

    @Override
    public void setLeftSpeed(double speed) {
        this.leftFrontModule.setRawDriveSpeed(speed);
        this.leftBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public void setRightSpeed(double speed) {
        this.rightFrontModule.setRawDriveSpeed(speed);
        this.rightBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public Angle getAngle() {
        double raw = pigeonIMU.getYaw() % 360;
        return new Angle(raw, DEGREE);
    }


    @Override
    public void setAngleOffset(Rotation2d offset) {
        this.pigeonIMU.setFusedHeading(offset.getDegrees());
    }

    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        leftFrontModule.configDrivetrainControlType(control);
        rightFrontModule.configDrivetrainControlType(control);
        leftBackModule.configDrivetrainControlType(control);
        rightBackModule.configDrivetrainControlType(control);
    }

    public Motor.MotorTypes getDrivetrainType() {
        return driveTrainType;
    }

    @Override
    public CartesianValue<Rotation2d> getGyroReading() {
        return new CartesianValue<>(Rotation2d.fromRadians(pigeonIMU.getPitch()), Rotation2d.fromRadians(pigeonIMU.getRoll()), Rotation2d.fromRadians(pigeonIMU.getYaw()));
    }
}
