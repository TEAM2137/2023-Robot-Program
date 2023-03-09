// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.splines.ObjectAvoidance;
import frc.robot.functions.splines.QuinticSpline;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

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
        /*String[] files = new String[] {
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\bluezone.txt",
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\redzone.txt",
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\humanplayer.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\communitywall.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\scorezone.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\chargezone.txt" };
*/
//        List<Translation2d> trans = new ArrayList<>();
//        trans.add(new Translation2d(0, 0));
//        trans.add(new Translation2d(4, 4));
//
//        List<Pose2d> poseList = calculateAngles(trans);
//
//        QuinticSpline spline = new QuinticSpline(poseList, 0.6);
//        List<PoseWithCurvature> outputList = spline.getSplinePoints();
//
//        for(PoseWithCurvature pose : outputList) {
//            System.out.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());
//        }

//        for(int i = 0; i < outputList.size(); i++) {
//            PoseWithCurvature pose = outputList.get(i);
//            Pose2d newPose = new Pose2d(pose.poseMeters.getX() * 30.48, pose.poseMeters.getY() * 30.48, pose.poseMeters.getRotation());
//
//            outputList.set(i, new PoseWithCurvature(newPose, pose.curvatureRadPerMeter));
//        }

//        ObjectAvoidance avoidance = new ObjectAvoidance(outputList, files);
//        avoidance.correctSpline(100);
//
//        avoidance.printOutput();*/

        RobotBase.startRobot(Robot::new);
    }

    public static List<Pose2d> calculateAngles(List<Translation2d> translation) {
        List<Pose2d> poses = new ArrayList<>();

        poses.add(new Pose2d(translation.get(0), Rotation2d.fromDegrees(90)));

        for(int i = 1; i < translation.size() - 1; i++) {
            double angle1 = Math.atan2(translation.get(i).getY() - translation.get(i - 1).getY(), translation.get(i).getX() - translation.get(i - 1).getX());
            double angle2 = Math.atan2(translation.get(i).getY() - translation.get(i + 1).getY(), translation.get(i).getX() - translation.get(i + 1).getX());

            double avgAngle = (angle1 + angle2) / 2.0;
            double newSlope = 1 / -Math.tan(avgAngle);

            poses.add(new Pose2d(translation.get(i), Rotation2d.fromRadians(Math.atan(newSlope))));
        }

        poses.add(new Pose2d(translation.get(translation.size() - 1), Rotation2d.fromDegrees(0)));

        return poses;
    }

}
