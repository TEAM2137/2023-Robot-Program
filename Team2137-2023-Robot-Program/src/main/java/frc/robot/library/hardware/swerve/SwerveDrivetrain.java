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
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.Gyro;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.Constants;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.swerve.module.SwerveModule;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.hardware.swerve.module.SwerveNEODriveModule;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import org.ejml.simple.SimpleMatrix;
import org.w3c.dom.Element;

import java.awt.*;
import java.io.File;
import java.text.DecimalFormat;

public class SwerveDrivetrain extends EntityGroup implements DriveTrain {

    public SwerveModule leftFrontModule;
    public SwerveModule leftBackModule;
    public SwerveModule rightFrontModule;
    public SwerveModule rightBackModule;

    public Translation2d currentRobotPosition = new Translation2d(10, 10);

    public Motor.MotorTypes driveTrainType;

    private FileLogger logger;

    private PigeonIMU pigeonIMU;

    public SwerveDrivetrain(Element element, int depth, EntityGroup parent, FileLogger fileLogger) {
        super(element, depth, parent, fileLogger);

        logger = fileLogger;

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
//        pigeonIMU = new PigeonIMU(encoder.getID());
//        pigeonIMU.configFactoryDefault();
//        pigeonIMU.setFusedHeading(encoder.getOffset());
    }

    @Override
    public void periodic() {
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
//        double r = Math.sqrt((axelDistance * axelDistance) + (wheelBase * wheelBase)); //Distance between adjectent wheel
//
//        double a = xMag - (rMag * (axelDistance / r)); // translatedSpeeds[2] * (axleDistance / r) is the ratio of wheel distance from other wheels
//        double b = xMag + (rMag * (axelDistance / r));
//        double c = yMag - (rMag * (wheelBase / r));
//        double d = yMag + (rMag * (wheelBase / r));

        double a = xMag - (rMag * (axelDistance / 2.0)); // translatedSpeeds[2] * (axleDistance / r) is the ratio of wheel distance from other wheels
        double b = xMag + (rMag * (axelDistance / 2.0));
        double c = yMag - (rMag * (wheelBase / 2.0));
        double d = yMag + (rMag * (wheelBase / 2.0));

        double[][] speeds = new double[][] {
                new double[] {Math.sqrt(b * b + d * d), -Math.atan2(b, d)}, // Left Front
                new double[] {Math.sqrt(a * a + d * d), -Math.atan2(a, d)}, // Left Back
                new double[] {Math.sqrt(b * b + c * c), -Math.atan2(b, c)}, // Right Front
                new double[] {Math.sqrt(a * a + c * c), -Math.atan2(a, c)}, // Right Back
        };

        if (controlType == Constants.DriveControlType.VELOCITY) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(new Speed2d(speeds[0][0]), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(new Speed2d(speeds[1][0]), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(new Speed2d(speeds[2][0]), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(new Speed2d(speeds[3][0]), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
            };
        } else if(controlType == Constants.DriveControlType.DISTANCE) {
            return new SwerveModuleState[]{
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[0][0]), new Rotation2d(speeds[0][1]), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[1][0]), new Rotation2d(speeds[1][1]), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[2][0]), new Rotation2d(speeds[2][1]), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                    new SwerveModuleState(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, speeds[3][0]), new Rotation2d(speeds[3][1]), SwerveModuleState.SwerveModulePositions.RIGHT_BACK),
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
        this.leftFrontModule.setSwerveModuleState(swerveModuleStates[0]);
        this.leftBackModule.setSwerveModuleState(swerveModuleStates[1]);
        this.rightFrontModule.setSwerveModuleState(swerveModuleStates[2]);
        this.rightBackModule.setSwerveModuleState(swerveModuleStates[3]);
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
                xSpeed * robotAngle.getCos() + ySpeed * robotAngle.getSin(),
                -xSpeed * robotAngle.getSin() + ySpeed * robotAngle.getCos(),
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
//        double raw = pigeonIMU.getFusedHeading() % 360;
//
//        if(raw >= 180)
//            raw -= 360;
//        else if(raw < -180)
//            raw += 360;
//
//        return Rotation2d.fromDegrees(raw);
        return Rotation2d.fromDegrees(0);
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
