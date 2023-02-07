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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.objects.Gyro;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.Constants;
import frc.robot.library.hardware.FalconCharacteristics;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Number;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.deadReckoning.DeadWheelActiveTracking;
import frc.robot.library.hardware.swerve.module.SwerveModule;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.CartesianValue;
import frc.robot.library.units.UnitContainers.Vector2d;
import org.ejml.simple.SimpleMatrix;
import org.w3c.dom.Element;

import java.text.DecimalFormat;

import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.*;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;


public class SwerveDrivetrain extends EntityGroup implements DriveTrain {

    public SwerveModule leftFrontModule;
    public SwerveModule leftBackModule;
    public SwerveModule rightFrontModule;
    public SwerveModule rightBackModule;

    Distance dia = new Distance(4, INCH);

    public SwerveKinematics<Number> swerveKinematics;

    private DeadWheelActiveTracking mDeadWheelActiveTracking;

    public Translation2d currentRobotPosition = new Translation2d(10, 10);

    public Motor.MotorTypes driveTrainType;

    private final FileLogger logger;

    private PigeonIMU pigeonIMU;

    public SwerveDrivetrain(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        logger = fileLogger;
        swerveKinematics = new SwerveKinematics<>(new Distance(28, INCH), new Distance(28, INCH));

        try {
            leftFrontModule = (SwerveModule) getChildEntityGroup("LeftFront");
            leftBackModule = (SwerveModule) getChildEntityGroup("LeftBack");
            rightFrontModule = (SwerveModule) getChildEntityGroup("RightFront");
            rightBackModule = (SwerveModule) getChildEntityGroup("RightBack");

            //driveTrainType = leftFrontModule.get
        } catch (Exception e) {
            e.printStackTrace();
        }

        Gyro gyro = (Gyro) getEntity("Pigeon");
        pigeonIMU = new PigeonIMU(gyro.getID());
        pigeonIMU.configFactoryDefault();
        pigeonIMU.setFusedHeading(gyro.getOffset());
    }

    @Override
    public void periodic() {
//        SwerveModuleState lfAccelState = leftFrontModule.getSwerveModuleAccelerationState(12.0);
//        SwerveModuleState lbAccelState = leftBackModule.getSwerveModuleAccelerationState(12.0);
//        SwerveModuleState rfAccelState = rightFrontModule.getSwerveModuleAccelerationState(12.0);
//        SwerveModuleState rbAccelState = rightBackModule.getSwerveModuleAccelerationState(12.0);

        SwerveModuleState lfVelState = leftFrontModule.getSwerveModuleState();
        SwerveModuleState lbVelState = leftBackModule.getSwerveModuleState();
        SwerveModuleState rfVelState = rightFrontModule.getSwerveModuleState();
        SwerveModuleState rbVelState = rightBackModule.getSwerveModuleState();

//        Vector2d<Distance> dist = swerveKinematics.updateSwerveKinematics(new SwerveModuleState[] { lfVelState, lbVelState, rfVelState, rbVelState }, new SwerveModuleState[] { lfAccelState, lbAccelState, rfAccelState, rbAccelState });
        Vector2d<Distance> dist = swerveKinematics.updateSwerveKinematics(new SwerveModuleState[] { lfVelState, lbVelState, rfVelState, rbVelState });
//        Vector2d<Velocity> vel = swerveKinematics.getCurrentRobotVelocity();

//        SmartDashboard.putNumber("Vel X", vel.getX().getValue(FEET_PER_SECOND));
//        SmartDashboard.putNumber("Vel Y", vel.getY().getValue(FEET_PER_SECOND));

        SmartDashboard.putNumber("Dist X", dist.getX().getValue(FOOT));
        SmartDashboard.putNumber("Dist Y", dist.getX().getValue(FOOT));
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(Velocity xMag, Velocity yMag, AngularVelocity rMag, double trackWidth, double wheelBase) {
        SimpleMatrix fieldCentric = Constants.convertFrame(getAngle().getRadians(), Constants.createFrameMatrix(xMag.getValue(METER_PER_SECOND), yMag.getValue(METER_PER_SECOND), rMag.getValue(RADIAN_PER_SECOND)));

        Velocity x = new Velocity(fieldCentric.get(0, 0), METER_PER_SECOND);
        Velocity y = new Velocity(fieldCentric.get(1, 0), METER_PER_SECOND);
        AngularVelocity w = new AngularVelocity(fieldCentric.get(2, 0), RADIAN_PER_SECOND);

        return swerveKinematics.getSwerveModuleState(x, y, w);
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(double xMag, double yMag, double rMag, double trackWidth, double wheelBase, Constants.DriveControlType controlType) {
        return calculateSwerveMotorSpeeds(Constants.convertFrame(getAngle().getRadians(), Constants.createFrameMatrix(xMag, yMag, rMag)), trackWidth, wheelBase, controlType);
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
    public Rotation2d getAngle() {
        double raw = pigeonIMU.getYaw() % 360;

//        if(raw >= 180)
//            raw -= 360;
//        else if(raw < -180)
//            raw += 360;

        return Rotation2d.fromDegrees(raw);
//        return Rotation2d.fromDegrees(0);
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

    public void logModuleStates() {
        logModuleStates(new SwerveModuleState[] {leftFrontModule.getSwerveModuleState(), leftBackModule.getSwerveModuleState(), rightFrontModule.getSwerveModuleState(), rightBackModule.getSwerveModuleState()});
    }

    public void logModuleStates(SwerveModuleState[] states) {
        logModuleStates(states, new Transform2d(currentRobotPosition, getAngle()));
    }

    public void logModuleStates(SwerveModuleState[] states, Transform2d robot) {
        StringBuilder builder = new StringBuilder();
        DecimalFormat formater = new DecimalFormat();
        formater.setMaximumFractionDigits(4);

        builder.append("Q~SWDSE~ ");
        builder.append(formater.format(leftFrontModule.getModuleAngle().getRadians())).append(" ");
        builder.append(formater.format(leftFrontModule.getRawDrivePower())).append(" ");
        builder.append(formater.format(leftBackModule.getModuleAngle().getRadians())).append(" ");
        builder.append(formater.format(leftBackModule.getRawDrivePower())).append(" ");
        builder.append(formater.format(rightFrontModule.getModuleAngle().getRadians())).append(" ");
        builder.append(formater.format(rightFrontModule.getRawDrivePower())).append(" ");
        builder.append(formater.format(rightBackModule.getModuleAngle().getRadians())).append(" ");
        builder.append(formater.format(rightBackModule.getRawDrivePower())).append(" ");
        builder.append(formater.format(robot.getX())).append(" ");
        builder.append(formater.format(robot.getY())).append(" ");
        builder.append(formater.format(robot.getRotation().getRadians())).append(" ");

        logger.writeLine(builder.toString());
    }
}
