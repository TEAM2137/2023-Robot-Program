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

package frc.robot.library.PurePursuit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import frc.robot.functions.splines.QuinticSpline;
import frc.robot.functions.splines.VelocityGenerator;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import frc.robot.library.units.Vector2d;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;


public class PurePursuitGenerator {

    public List<Translation2d> pointList;
    private final Distance2d lookAheadDistance;

    public PurePursuitGenerator(List<Translation2d> points, Distance2d _lookAheadDistance) {
        pointList = points;
        lookAheadDistance = _lookAheadDistance;
    }

    public PurePursuitGenerator(Distance2d _lookAheadDistance, List<PoseWithCurvature> pose) {
        pointList = new ArrayList<>();
        for(PoseWithCurvature a : pose) {
            pointList.add(a.poseMeters.getTranslation());
        }
        lookAheadDistance = _lookAheadDistance;
    }

    public Map.Entry<Transform2d, Map.Entry<Translation2d, Translation2d>> calculateGoalPose(Translation2d currentPosition) {
        Map.Entry<Translation2d, Map.Entry<Translation2d, Translation2d>> lookAheadInfo = getLookaheadPoint(currentPosition.getX(), currentPosition.getY(), lookAheadDistance.getValue(Distance2d.DistanceUnits.FEET));

//        if (lookAhead == null) {
//            for(int i = 1; i < pointList.size(); i++) {
//                Translation2d upperPoint = pointList.get(i);
//                Translation2d lowerPoint = pointList.get(i - 1);

//                System.out.println("Current X: " + currentPosition.getX() + " y: " + currentPosition.getY());
//                System.out.println("X Lower: " + lowerPoint.getX() + " Y Lower: " + lowerPoint.getY());
//                System.out.println("X Upper: " + upperPoint.getX() + " Y Upper: " + upperPoint.getY());

//                if ((currentPosition.getX() <= upperPoint.getX() && currentPosition.getX() >= lowerPoint.getX())
//                    && (currentPosition.getY() <= upperPoint.getY() && currentPosition.getY() >= lowerPoint.getY())) {
//
//                    lookAhead = upperPoint;
//                }
//            }
//
//            if(lookAhead == null) {
//                lookAhead = pointList.get(0);
//            }
//        }

        Translation2d lookAhead = lookAheadInfo.getKey();

        Rotation2d rotation;
        try {
            rotation = new Rotation2d(lookAhead.getX() - currentPosition.getX(), lookAhead.getY() - currentPosition.getY());
        } catch (Exception e) {
            rotation = null;
        }
        return Map.entry(new Transform2d(lookAhead, rotation), lookAheadInfo.getValue());
    }

    /**
     * Returns the sign of the input number n. Note that the function returns 1 for n = 0 to satisfy the requirements
     * set forth by the line-circle intersection formula.
     *
     * @param n The number to return the sign of.
     * @return A float value of the sign of the number (-1.0f for n < 0, else 1.0f).
     */
    private double signum(double n) {
        if (n == 0) return 1;
        else return Math.signum(n);
    }

    /**
     * Generate the furthest lookahead point on the pointList that is distance r from the point (x, y).
     *
     * @param x The x of the origin.
     * @param y The y of the origin.
     * @param r The lookahead distance.
     * @return A float[] coordinate pair if the lookahead point exists, or null.
     * @see <a href="http://mathworld.wolfram.com/Circle-LineIntersection.html">Circle-Line Intersection</a>
     */
    private Map.Entry<Translation2d, Map.Entry<Translation2d, Translation2d>> getLookaheadPoint(double x, double y, double r) {
        Translation2d lookahead = null;
        Map.Entry<Translation2d, Translation2d> startAndEnd = null;

        // iterate through all pairs of points
        for (int i = 0; i < pointList.size() - 1; i++) {
            // form a segment from each two adjacent points
            Translation2d segmentStart = pointList.get(i);
            Translation2d segmentEnd = pointList.get(i + 1);

            // translate the segment to the origin
            double[] p1 = new double[]{segmentStart.getX() - x, segmentStart.getY() - y};
            double[] p2 = new double[]{segmentEnd.getX() - x, segmentEnd.getY() - y};

            // calculate an intersection of a segment and a circle with radius r (lookahead) and origin (0, 0)
            double dx = p2[0] - p1[0];
            double dy = p2[1] - p1[1];
            double d = Math.sqrt(dx * dx + dy * dy);
            double D = p1[0] * p2[1] - p2[0] * p1[1];

            // if the discriminant is zero or the points are equal, there is no intersection
            double discriminant = r * r * d * d - D * D;
            if (discriminant < 0 || Arrays.equals(p1, p2)) continue;

            // the x components of the intersecting points
            double x1 = (D * dy + signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);
            double x2 = (D * dy - signum(dy) * dx * Math.sqrt(discriminant)) / (d * d);

            // the y components of the intersecting points
            double y1 = (-D * dx + Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);
            double y2 = (-D * dx - Math.abs(dy) * Math.sqrt(discriminant)) / (d * d);

            // whether each of the intersections are within the segment (and not the entire line)
            boolean validIntersection1 = Math.min(p1[0], p2[0]) < x1 && x1 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y1 && y1 < Math.max(p1[1], p2[1]);
            boolean validIntersection2 = Math.min(p1[0], p2[0]) < x2 && x2 < Math.max(p1[0], p2[0])
                    || Math.min(p1[1], p2[1]) < y2 && y2 < Math.max(p1[1], p2[1]);

            // remove the old lookahead if either of the points will be selected as the lookahead
            if (validIntersection1 || validIntersection2) lookahead = null;

            // select the first one if it's valid
            if (validIntersection1) {
                lookahead = new Translation2d(x1 + x, y1 + y);
                startAndEnd = Map.entry(segmentStart, segmentEnd);
            }

            // select the second one if it's valid and either lookahead is none,
            // or it's closer to the end of the segment than the first intersection
            if (validIntersection2) {
                if (lookahead == null || Math.abs(x1 - p2[0]) > Math.abs(x2 - p2[0]) || Math.abs(y1 - p2[1]) > Math.abs(y2 - p2[1])) {
                    lookahead = new Translation2d(x2 + x, y2 + y);
                    startAndEnd = Map.entry(segmentStart, segmentEnd);
                }
            }
        }

        // special case for the very last point on the pointList
        if (pointList.size() > 0) {
            Translation2d lastPoint = pointList.get(pointList.size() - 1);

            double endX = lastPoint.getX();
            double endY = lastPoint.getY();

            // if we are closer than lookahead distance to the end, set it as the lookahead
            if (Math.sqrt((endX - x) * (endX - x) + (endY - y) * (endY - y)) <= r) {
                return Map.entry(new Translation2d(endX, endY), Map.entry(lastPoint, lastPoint));
            }
        }

        return Map.entry(lookahead, startAndEnd);
    }
}

