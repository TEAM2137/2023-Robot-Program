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
import frc.robot.functions.splines.QuinticSpline;
import frc.robot.functions.splines.VelocityGenerator;
import frc.robot.library.PurePursuit.PurePursuitGenerator;
import frc.robot.library.astar.AStarPathPlanner;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.FEET_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main
{
    private Main() {}

    public static FileLogger fileLogger;

    public static void openfile(String filename, boolean[][] grid) {
        // File path is passed as parameter
        File file = new File(filename);

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int row = 0;

            while ((line = br.readLine()) != null) {
                for (int col = 0; col < line.length(); col++){
                    char c = line.charAt(col);

                    if(c == '1')
                        grid[row][col] = true;
//                    else
//                        grid[row][col] = false;
                }

                row++;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static boolean[][] inflateAvoidanceZone(boolean[][] grid, double r) {
        boolean[][] inflated = new boolean[grid.length][grid[0].length];

        ArrayList<Pair<Integer, Integer>> circlePoints = new ArrayList<>();

        for(int x = (int) -r; x <= r; ++x)
            for(int y = (int) -r; y <= r; ++y)
                if(Math.pow(x, 2) + Math.pow(y, 2) <= Math.pow(r, 2))   {
                    circlePoints.add(Pair.of(x, y));
                }

        for(int row = 0; row < grid.length; row++) {
            for(int col = 0; col < grid[0].length; col++) {
                if(!grid[row][col] && row != 0 & col != 0 && row != grid.length - 1 && col != grid[0].length - 1)
                    continue;

                for(Pair<Integer, Integer> point : circlePoints) {
                    int tmpR = point.getFirst() + row;
                    int tmpC = point.getSecond() + col;

                    if(tmpR < 0 || tmpR >= 200 || tmpC < 0 || tmpC >= 412)
                        continue;

                    inflated[tmpR][tmpC] = true;
                }
            }
        }

        return inflated;
    }

    static List<PoseWithCurvature> outputList = new ArrayList<>();
   /**
    * Main initialization method. Do not perform any initialization here.
    * <p>
    * If you change your main Robot class (name), change the parameter type.
    */
    public static void main(String... args)
    {
        /*
        String[] files = new String[] {
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\bluezone.txt",
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\redzone.txt",
                //"C:\\Users\\Wyatt\\Downloads\\FeildBits\\humanplayer.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\communitywall.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\scorezone.txt",
                "C:\\Users\\Wyatt\\Downloads\\FeildBits\\chargezone.txt" };

        boolean[][] grid = new boolean[200][412];

        for(String name : files) {
            openfile(name, grid);
        }

        grid = inflateAvoidanceZone(grid, 10);

        List<Translation2d> trans = new ArrayList<>();
        trans.add(new Translation2d(7.129921, 9.879921));
        trans.add(new Translation2d(19.68504, 9.879921));
        trans.add(new Translation2d(21.32546, 6.56168));

//        List<Pose2d> poseList = new ArrayList<>();
//        poseList.add(new Pose2d(7.129921, 9.879921, Rotation2d.fromDegrees(90)));
//        poseList.add(new Pose2d(19.68504, 9.879921, Rotation2d.fromDegrees(90)));
//        poseList.add(new Pose2d(21.32546, 6.56168, Rotation2d.fromDegrees(0)));

        List<Pose2d> poseList = calculateAngles(trans);
//
//        for(Pose2d pose : poseList) {
//            System.out.println(pose.toString());
//        }

        QuinticSpline spline = new QuinticSpline(poseList, 0.6);
        outputList = spline.getSplinePoints();

        for(int i = 0; i < outputList.size(); i++) {
            PoseWithCurvature pose = outputList.get(i);
            Pose2d newPose = new Pose2d(pose.poseMeters.getX() * 30.48, pose.poseMeters.getY() * 30.48, pose.poseMeters.getRotation());

            outputList.set(i, new PoseWithCurvature(newPose, pose.curvatureRadPerMeter));
        }

//        for(PoseWithCurvature pose : outputList) {
//            System.out.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());
//        }

//        for (int i = 0; i < 4; i++) {
//            correctSpline(grid, poseList);
//        }

        while(correctSpline(grid, poseList));

//        for(PoseWithCurvature pose : outputList) {
//            System.out.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());
//        }

//        boolean[][] map = new boolean[grid.length][grid[0].length];
//
//        for(PoseWithCurvature pose : outputList) {
//            int x = (int) (pose.poseMeters.getX() / 4.0);
//            int y = 200 - (int) (pose.poseMeters.getY() / 4.0);
//            //System.out.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());
//
//            map[y][x] = true;
//        }
//
//        for(int y = 0; y < grid.length; y++) {
//            System.out.print("|");
//            for (int x = 0; x < grid[0].length; x++) {
//                if (map[y][x]) {
//                    System.out.print("O|");
//                } else if (grid[y][x]) {
//                    System.out.print("X|");
//                } else {
//                    System.out.print(" |");
//                }
//            }
//            System.out.println();
//        }

        VelocityGenerator velocityGenerator = new VelocityGenerator(outputList, new Velocity(16, FEET_PER_SECOND), new Acceleration(8, FEET_PER_SECOND2), 1);
        List<Velocity> velocities = velocityGenerator.getSpeeds();
        Distance mPurePursuitLookaheadDistance = new Distance(20, FOOT);
        PurePursuitGenerator mDrivePurePursuitGenerator = new PurePursuitGenerator(mPurePursuitLookaheadDistance, outputList, velocities);

//        for(PoseWithCurvature pose : outputList) {
//            Map.Entry<Pose2d, Velocity> result = mDrivePurePursuitGenerator.calculateGoalPose(new Translation2d(pose.poseMeters.getX(), pose.poseMeters.getY()));
//
//            System.out.printf("(%.4f, %.4f)\n", result.getKey().getX(), result.getKey().getY());
//        }


        double time = 0;
        double xCurrent = 218.32;
        double yCurrent = 301.1403;
        while(time < 15) {
            Map.Entry<Pose2d, Velocity> result = mDrivePurePursuitGenerator.calculateGoalPose(new Translation2d(xCurrent, yCurrent));

            double xTmp = result.getKey().getX();
            double yTmp = result.getKey().getY();
            double max = Math.max(xTmp, yTmp);

            xTmp /= max;
            yTmp /= max;

            xTmp *= 1;
            yTmp *= 1;

            xCurrent += xTmp;
            yCurrent += yTmp;

            System.out.printf("(%.4f, %.4f)\n", xCurrent, yCurrent);

            time += 0.01;
        }*/
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

    public static boolean correctSpline(boolean[][] grid, List<Pose2d> pointList) {
        int collisionStartPoint = -1;

        for(int i = 0; i < outputList.size(); i++) {
            PoseWithCurvature pose = outputList.get(i);
            int x = (int) (pose.poseMeters.getX() / 4.0);
            int y = 200 - (int) (pose.poseMeters.getY() / 4.0);

            if (grid[y][x]) {
                System.out.printf("Collision at (%d, %d)\n", y, x);
                collisionStartPoint = i;
                break;
            }
        }

        if(collisionStartPoint == -1) {
            return false;
        }

        int collisionEndPoint = collisionStartPoint;
        boolean foundCollision = false;

        for(int i = collisionStartPoint; i < outputList.size(); i++) {
            PoseWithCurvature pose = outputList.get(i);
            int x = (int) (pose.poseMeters.getX() / 4.0);
            int y = 200 - (int) (pose.poseMeters.getY() / 4.0);

            if (!grid[y][x]) {
                System.out.printf("Collision ends at (%d, %d)\n", y, x);
                collisionEndPoint = i;
                foundCollision = true;
                break;
            }
        }

        if(!foundCollision) {
            return false;
        }

        double distance = 0;

        for(int i = collisionStartPoint; i < collisionEndPoint; i++) {
            Pose2d currentPose = outputList.get(i).poseMeters;
            Pose2d futurePose = outputList.get(i + 1).poseMeters;

            distance += Math.sqrt(Math.pow(futurePose.getX() - currentPose.getX(), 2) + Math.pow(futurePose.getY() - currentPose.getY(), 2));
        }

        double halfDistance = distance / 2.0;
        int centerPointIdx = 0;
        double tmpDistance = 0;
//        boolean first = false;

        for(int i = collisionStartPoint; i < outputList.size() - 1; i++) {
            Pose2d currentPose = outputList.get(i).poseMeters;
            Pose2d futurePose = outputList.get(i + 1).poseMeters;

            tmpDistance += Math.sqrt(Math.pow(futurePose.getX() - currentPose.getX(), 2) + Math.pow(futurePose.getY() - currentPose.getY(), 2));

            if(tmpDistance > halfDistance) {
//                if(!first) {
                    centerPointIdx = i;
                    System.out.printf("Found avoidance point at (%.4f, %.4f) %d\n", currentPose.getX(), currentPose.getY(), centerPointIdx);
                    break;
//                } else {
//                    first = true;
//                }
            }
        }

        double slope;
        double inverseAngle;
        Pose2d centerPoint = outputList.get(centerPointIdx).poseMeters;

        if(centerPointIdx != 0) {
            Pose2d previousPoint = outputList.get(centerPointIdx - 1).poseMeters;
            if(centerPoint.getX() - previousPoint.getX() == 0) {
                inverseAngle = Math.atan(0);
                slope = Double.POSITIVE_INFINITY;
            } else {
                slope = (centerPoint.getY() - previousPoint.getY()) / (centerPoint.getX() - previousPoint.getX());
                inverseAngle = Math.atan(1 / -(slope));
            }
        } else {
            slope = Math.tan(pointList.get(0).getRotation().getRadians());
            inverseAngle = Math.atan(1 / -(slope));
        }

        double newX1 = centerPoint.getX();
        double newY1 = centerPoint.getY();

        int requirePoints = 4;
        int pointCounter = 0;

        //Find Tangent
        while(true) {
            newX1 += Math.cos(inverseAngle) * 1.0;
            newY1 += Math.sin(inverseAngle) * 1.0;

            int x = (int) (newX1 / 4.0);
            int y = 200 - (int) (newY1 / 4.0);

            if (!grid[y][x]) {
                if(pointCounter < requirePoints) {
                    pointCounter++;
                } else {
                    System.out.printf("Found avoidance point 1 at (%.4f, %.4f)\n", newX1, newY1);
                    break;
                }
            }
        }

        double newX2 = centerPoint.getX();
        double newY2 = centerPoint.getY();
        pointCounter = 0;

        //Find Other Tangent
        while(true) {
            newX2 -= Math.cos(inverseAngle) * 1.0;
            newY2 -= Math.sin(inverseAngle) * 1.0;

            int x = (int) (newX2 / 4.0);
            int y = 200 - (int) (newY2 / 4.0);

            if (!grid[y][x]) {
                if(pointCounter < requirePoints) {
                    pointCounter++;
                } else {
                    System.out.printf("Found avoidance point 2 at (%.4f, %.4f)\n", newX2, newY2);
                    break;
                }
            }
        }

        double newX;
        double newY;
        boolean add;

        if((Math.pow(newX2 - centerPoint.getX(), 2) + Math.pow(newY2 - centerPoint.getY(), 2)) > (Math.pow(newX1 - centerPoint.getX(), 2) + Math.pow(newY1 - centerPoint.getY(), 2))){
            newX = newX1;
            newY = newY1;
            add = true;
        } else {
            newX = newX2;
            newY = newY2;
            add = false;
        }

        double dDistance = Math.sqrt(Math.pow(newX - centerPoint.getX(), 2) + Math.pow(newY - centerPoint.getY(), 2));
        Pose2d lastPoint = outputList.get(centerPointIdx).poseMeters;
        tmpDistance = 0;

        //sin(x) * distance
        for(int i = centerPointIdx + 1; i < outputList.size(); i++) {
            Pose2d currentPoint = outputList.get(i).poseMeters;
            tmpDistance += Math.sqrt(Math.pow(currentPoint.getX() - lastPoint.getX(), 2) + Math.pow(currentPoint.getY() - lastPoint.getY(), 2));

            if(tmpDistance > dDistance * 2) {
                break;
            }

            double dd = (dDistance - tmpDistance);
            //dd = Math.sin((dd / dDistance) * (Math.PI / 2)) * dd;
            double val = (Math.sin((dd / dDistance) * (Math.PI / 2)) * (dDistance / 2) + (dDistance / 2));
//            double alteredX = currentPoint.getX() + ((dd) * Math.cos(inverseAngle) * (add ? 1 : -1));
            double alteredX = currentPoint.getX() + ((val) * Math.cos(inverseAngle) * (add ? 1 : -1));
//            double alteredY = currentPoint.getY() + ((dd) * Math.sin(inverseAngle) * (add ? 1 : -1));
            double alteredY = currentPoint.getY() + ((val) * Math.sin(inverseAngle) * (add ? 1 : -1));

            Pose2d newPose = new Pose2d(alteredX, alteredY, currentPoint.getRotation());

            lastPoint = currentPoint;
            outputList.set(i, new PoseWithCurvature(newPose, outputList.get(i).curvatureRadPerMeter));
        }

        lastPoint = outputList.get(centerPointIdx).poseMeters;
        tmpDistance = 0;
        for(int i = centerPointIdx - 1; i > 0; i--) {
            Pose2d currentPoint = outputList.get(i).poseMeters;
            tmpDistance += Math.sqrt(Math.pow(currentPoint.getX() - lastPoint.getX(), 2) + Math.pow(currentPoint.getY() - lastPoint.getY(), 2));

            if(tmpDistance > dDistance * 2)
                break;

            double dd = (dDistance - tmpDistance);
//            dd = Math.sin((dd / dDistance) * (Math.PI / 2)) * dd;
            double val = (Math.sin((dd / dDistance) * (Math.PI / 2)) * (dDistance / 2) + (dDistance / 2));
//            double alteredX = currentPoint.getX() + (dd * Math.cos(inverseAngle) * (add ? 1 : -1));
            double alteredX = currentPoint.getX() + (val * Math.cos(inverseAngle) * (add ? 1 : -1));
//            double alteredY = currentPoint.getY() + (dd * Math.sin(inverseAngle) * (add ? 1 : -1));
            double alteredY = currentPoint.getY() + (val * Math.sin(inverseAngle) * (add ? 1 : -1));

            Pose2d newPose = new Pose2d(alteredX, alteredY, currentPoint.getRotation());

            lastPoint = currentPoint;

            outputList.set(i, new PoseWithCurvature(newPose, outputList.get(i).curvatureRadPerMeter));
        }

        Pose2d currentPoint = outputList.get(centerPointIdx).poseMeters;

        double alteredX = currentPoint.getX() + (dDistance * Math.cos(inverseAngle) * (add ? 1 : -1));
        double alteredY = currentPoint.getY() + (dDistance * Math.sin(inverseAngle) * (add ? 1 : -1));

        Pose2d newPose = new Pose2d(alteredX, alteredY, currentPoint.getRotation());

        outputList.set(centerPointIdx, new PoseWithCurvature(newPose, outputList.get(centerPointIdx).curvatureRadPerMeter));

//        System.out.println("Spline point adding.." + pointList.size());
//
//        for(int i = 0; i < pointList.size(); i++) {
//            Pose2d pose = pointList.get(i);
//
//            if(pose.getX() > newX) {
//                pointList.add(i, new Pose2d(newX, newY, Rotation2d.fromRadians(Math.tan(slope))));
//                break;
//            }
//        }

//        List<Pose2d> poses = calculateAngles(pointList);

//        System.out.println("Spline generation started");
//        QuinticSpline spline2 = new QuinticSpline(pointList, 0.6);
//        System.out.println("Spline generation ended");
//        outputList = spline2.getSplinePoints();

        return true;
    }
}
