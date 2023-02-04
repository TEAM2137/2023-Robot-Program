// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.splines.QuinticSpline;
import frc.robot.functions.splines.VelocityGenerator;
import frc.robot.library.Constants;
import frc.robot.library.PurePursuit.PurePursuitGenerator;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.*;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static frc.robot.library.hardware.FusedTrackingAlgorithm.calculateTipAcceleration;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main
{
    private Main() {}

    public static FileLogger fileLogger;
   /**
    * Main initialization method. Do not perform any initialization here.
    * <p>
    * If you change your main Robot class (name), change the parameter type.
    */
    public static void main(String... args)
    {
        //X is right to left    (+, -)
        //Y is forward to back  (+, -)
        //Z is up to down       (+, -)
//        CartesianValue<Distance2d> inital = new CartesianValue<Distance2d>(new Distance2d(1, Distance2d.DistanceUnits.FEET),
//                new Distance2d(0, Distance2d.DistanceUnits.FEET),
//                new Distance2d(1, Distance2d.DistanceUnits.FEET));


//        CartesianValue<Distance2d> cog = new CartesianValue<Distance2d>(new Distance2d(14, Distance2d.DistanceUnits.INCH),
//                new Distance2d(14, Distance2d.DistanceUnits.INCH),
//                new Distance2d(28, Distance2d.DistanceUnits.INCH));
//
//        CartesianValue<Distance2d> tipPoint = new CartesianValue<Distance2d>(new Distance2d(14, Distance2d.DistanceUnits.INCH),
//                new Distance2d(0, Distance2d.DistanceUnits.INCH),
//                new Distance2d(0, Distance2d.DistanceUnits.INCH));
//
//        Vector3d backAccel = new Vector3d(new Distance2d(0, Distance2d.DistanceUnits.METER), new Distance2d(5, Distance2d.DistanceUnits.METER), new Distance2d(0, Distance2d.DistanceUnits.METER));
//
//        Vector3d vect = calculateMaximumAccelTip(cog, tipPoint, backAccel);
//        System.out.println("Acceleration Vect: ");
//        vect.print();

//        System.out.println("Value: " + calculateTipAcceleration(new Translation2d(.3556, .7112), new Translation2d(0, 0.05), Rotation2d.fromDegrees(0)));

        RobotBase.startRobot(Robot::new);
    }

    /**
     * Test XML Reading and writing....
     */
    public static void testXMLFunction(XMLSettingReader settingReader, XMLStepReader stepReader) {

        EntityGroup robotHardware = settingReader.getRobot();

        SwerveDrivetrain swerveDrivetrain = (SwerveDrivetrain) robotHardware.getEntityGroupByType("DriveTrain");

        //        SwerveModuleState[] states = swerveDrivetrain.calculateSwerveMotorSpeeds(0, 0, 0, 3, 3, Constants.DriveControlType.RAW);
//
//        swerveDrivetrain.setSwerveModuleStates(states);
//        swerveDrivetrain.logModuleStates(states);
        List<Translation2d> wayPointList = new ArrayList<>();
        for(Step step : stepReader.getSteps()) {
            if(step.getCommand().equalsIgnoreCase("drive")) {
                wayPointList.add(new Translation2d(step.getXDistance(), step.getYDistance()));
            }
        }
        //testSpline(swerveDrivetrain, wayPointList);
//        for(int i = 5; i < 25; i+=3.5) {
//            swerveDrivetrain.currentRobotPosition = new Translation2d(i, i);
//            SwerveModuleState[] states = swerveDrivetrain.calculateSwerveMotorSpeeds(0, 0, 1, 3, 3, Constants.DriveControlType.RAW);
//
//            swerveDrivetrain.setSwerveModuleStates(states);
//            swerveDrivetrain.logModuleStates(states);
//        }
        fileLogger.flush();
    }
/*
    public static void testSpline(SwerveDrivetrain drivetrain, List<Translation2d> waypoints) {
//        list.add(new Translation2d(10, 20));
//        list.add(new Translation2d(5, 10));
//        list.add(new Translation2d(0, 15));

        QuinticSpline spline = new QuinticSpline(waypoints, 0.8, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0));

        List<PoseWithCurvature> tmp = spline.getSplinePoints();

        VelocityGenerator velocityGenerator = new VelocityGenerator(tmp, Speed2d.fromFeetPerSecond(1), Speed2d.fromFeetPerSecond(.25), 1);
        List<Speed2d> tmp2 = velocityGenerator.getSpeeds();

        //PurePursuitGenerator generator = new PurePursuitGenerator(Distance2d.fromUnit(Distance2d.DistanceUnits.FEET, 1.5), tmp);

        double currentX = waypoints.get(0).getX();
        double currentY = waypoints.get(0).getY();

//        for(int i = 0; i < 200; i++) {
        int i = 0;
        while (true) {
//            drivetrain.currentRobotPosition = new Translation2d(tmp.get(i).poseMeters.getX(), tmp.get(i).poseMeters.getY());
            drivetrain.currentRobotPosition = new Translation2d(currentX, currentY);

//            double dy = tmp.get(i + 1).poseMeters.getY() - tmp.get(i).poseMeters.getY();
//            double dx = tmp.get(i + 1).poseMeters.getX() - tmp.get(i).poseMeters.getX();
//            double a = Math.atan2(dy, dx);
//            SwerveModuleState[] states = drivetrain.calculateSwerveMotorSpeeds(Math.cos(a), Math.sin(a), 0, 3, 3, Constants.DriveControlType.RAW);

            Map.Entry<Transform2d, Map.Entry<Translation2d, Translation2d>> generatedPoseInfo = generator.calculateGoalPose(new Translation2d(currentX, currentY));
            Transform2d transform2d = generatedPoseInfo.getKey();

            if(currentX > 17 && currentY > 14.4)
                return;

            Translation2d goalPoint = transform2d.getTranslation();

            Vector2d vector = new Vector2d(Distance2d.fromUnit( Distance2d.DistanceUnits.FEET,goalPoint.getX() - currentX), Distance2d.fromUnit(Distance2d.DistanceUnits.FEET, goalPoint.getY() - currentY));

            vector = vector.normalize();

            currentX += vector.getX().getValue(Distance2d.DistanceUnits.FEET) * .5;
            currentY += vector.getY().getValue(Distance2d.DistanceUnits.FEET) * .5;

            SwerveModuleState[] states = drivetrain.calculateSwerveMotorSpeeds(vector.getX().getValue(Distance2d.DistanceUnits.FEET), vector.getY().getValue(Distance2d.DistanceUnits.FEET), 0, 3, 3, Constants.DriveControlType.RAW);

            drivetrain.setSwerveModuleStates(states);
            drivetrain.logModuleStates(states);

            StringBuilder builder = new StringBuilder();
            DecimalFormat formater = new DecimalFormat();
            formater.setMaximumFractionDigits(4);

            builder.append("Q~RPGA~ ");
            builder.append(formater.format(currentX)).append(" ");
            builder.append(formater.format(currentY)).append(" ");

            fileLogger.writeLine(builder.toString());

            builder = new StringBuilder();

            builder.append("Q~RPGA~ ");
            builder.append(formater.format(generatedPoseInfo.getValue().getValue().getX())).append(" ");
            builder.append(formater.format(generatedPoseInfo.getValue().getValue().getY())).append(" ");

            fileLogger.writeLine(builder.toString());

//            fileLogger.flush();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

 */

//    public void testPurePursuit(List<PoseWithCurvature> spline) {
//        PurePursuitGenerator generator = new PurePursuitGenerator(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, 1.5), spline);
//
//        double xCurrent = 0;
//        double yCurrent = 0;
//
//        for(int t = 0; t < generator.pointList.size(); t++) {
//            Translation2d goalPoint = generator.calculateGoalPose(new Translation2d(xCurrent, yCurrent)).getTranslation();
//
//            Vector2d vector = new Vector2d(Distance2d.fromUnit( Distance2d.DistanceUnits.INCH,goalPoint.getX() - xCurrent), Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, goalPoint.getY() - yCurrent));

//            double distance = Math.sqrt(Math.pow(goalPoint.getX() - xCurrent, 2) + Math.pow(goalPoint.getY() - yCurrent, 2));
//            double driveLength = vector.magnitude();
//            vector = vector.normalize();
//
//            xCurrent += vector.getX().getValue(Distance2d.DistanceUnits.INCH);// * driveLength;
//            yCurrent += vector.getY().getValue(Distance2d.DistanceUnits.INCH);// * driveLength;
//
//            System.out.print("(" + xCurrent + ", " + yCurrent + "), ");
//        }
//    }
}
