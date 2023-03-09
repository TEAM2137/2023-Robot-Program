package frc.robot.functions.splines;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.spline.PoseWithCurvature;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class ObjectAvoidance {

    boolean[][] grid;
    List<PoseWithCurvature> outputList;

    public ObjectAvoidance(List<PoseWithCurvature> poses, String[] files) {
        grid = new boolean[200][412];

        for(String name : files) {
            openfile(name);
        }

        grid = inflateAvoidanceZone(grid, 10);

        outputList = poses;
    }

    public List<PoseWithCurvature> correctSpline(int maxIterations) {
        int iteration = 0;

        while(correctSpline() && iteration++ < maxIterations);

        return outputList;
    }

    public boolean[][] getGrid() {
        return grid;
    }

    public void printOutput() {
        boolean[][] map = new boolean[200][412];

        for(PoseWithCurvature pose : outputList) {
            int x = (int) (pose.poseMeters.getX() / 4.0);
            int y = 200 - (int) (pose.poseMeters.getY() / 4.0);
            //System.out.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());

            map[y][x] = true;
        }

        for(int y = 0; y < map.length; y++) {
            System.out.print("|");
            for (int x = 0; x < grid[0].length; x++) {
                if (map[y][x]) {
                    System.out.print("O|");
                } else if (grid[y][x]) {
                    System.out.print("X|");
                } else {
                    System.out.print(" |");
                }
            }
            System.out.println();
        }
    }

    private static boolean[][] inflateAvoidanceZone(boolean[][] grid, double r) {
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

    private void openfile(String filename) {
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
                }

                row++;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private boolean correctSpline() {
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

        for(int i = collisionStartPoint; i < outputList.size() - 1; i++) {
            Pose2d currentPose = outputList.get(i).poseMeters;
            Pose2d futurePose = outputList.get(i + 1).poseMeters;

            tmpDistance += Math.sqrt(Math.pow(futurePose.getX() - currentPose.getX(), 2) + Math.pow(futurePose.getY() - currentPose.getY(), 2));

            if(tmpDistance > halfDistance) {
                centerPointIdx = i;
                System.out.printf("Found avoidance point at (%.4f, %.4f) %d\n", currentPose.getX(), currentPose.getY(), centerPointIdx);
                break;
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
            slope = Math.tan(outputList.get(0).poseMeters.getRotation().getRadians());
            inverseAngle = Math.atan(1 / -(slope));
        }

        double newX1 = centerPoint.getX();
        double newY1 = centerPoint.getY();

        int requirePoints = 4;
        int pointCounter = 0;

        //Find Tangent
        while(true) {
            newX1 += Math.cos(inverseAngle);
            newY1 += Math.sin(inverseAngle);

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
            newX2 -= Math.cos(inverseAngle);
            newY2 -= Math.sin(inverseAngle);

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
            double val = (Math.sin((dd / dDistance) * (Math.PI / 2)) * (dDistance / 2) + (dDistance / 2));
            double alteredX = currentPoint.getX() + ((val) * Math.cos(inverseAngle) * (add ? 1 : -1));
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
            double val = (Math.sin((dd / dDistance) * (Math.PI / 2)) * (dDistance / 2) + (dDistance / 2));
            double alteredX = currentPoint.getX() + (val * Math.cos(inverseAngle) * (add ? 1 : -1));
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

        return true;
    }
}
