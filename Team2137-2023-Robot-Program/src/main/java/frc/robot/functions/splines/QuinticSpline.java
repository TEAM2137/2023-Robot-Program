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

package frc.robot.functions.splines;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import frc.robot.functions.splines.Spline;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.functions.splines.Spline.parameterizeSpline;

public class QuinticSpline {
    private SimpleMatrix hermiteBasisMatrix;
    private List<Spline> splines = new ArrayList<Spline>();

    public QuinticSpline(List<Pose2d> waypoints, double horizontalScaler) {
        hermiteBasisMatrix = getHermiteBasis();

        for (int a = 0; a < waypoints.size() - 1; a++) {
            ControlVectorPair controlVector = new ControlVectorPair(waypoints.get(a), waypoints.get(a + 1), horizontalScaler);

            var x = controlVector.getXControlMatrix();
            var y = controlVector.getYControlMatrix();

            var xCoeffs = (hermiteBasisMatrix.mult(x)).transpose();
            var yCoeffs = (hermiteBasisMatrix.mult(y)).transpose();

            SimpleMatrix tmp = new SimpleMatrix(6, 6);
            tmp.setRow(0, 0, matrixRowToArray(xCoeffs.rows(0, 1)));
            tmp.setRow(1, 0, matrixRowToArray(yCoeffs.rows(0, 1)));

            for (int i = 0; i < 6; i++) {
                tmp.set(2, i, tmp.get(0, i) *(5 - i));
                tmp.set(3, i, tmp.get(1, i) *(5 - i));
            }
            for (int i = 0; i < 6; i++) {
                tmp.set(4, i, tmp.get(2, i) *(4 - i));
                tmp.set(5, i, tmp.get(3, i) *(4 - i));
            }

            splines.add(new Spline(tmp));
        }
    }

    public QuinticSpline(List<Translation2d> waypoints, double horizontalScaler, Rotation2d startAngle, Rotation2d endAngle) {
        List<Pose2d> pose = new ArrayList<Pose2d>();

        pose.add(new Pose2d(waypoints.get(0), startAngle));

        for (int a = 1; a < waypoints.size() - 1; a++) {
            //var c = waypoints[a].vectorizeToNext(waypoints[a + 1]);
            //var d = waypoints[a-1].vectorizeToNext(waypoints[a]);
            //pose.Add(new Pose2d(c.getX(), c.getY(), Rotation2d.fromDegrees((c.getRotation().getDegrees() + d.getRotation().getDegrees()) / 2)));
            Rotation2d angleToNextWayPoint = new Rotation2d(
                    Math.atan2(waypoints.get(a + 1).getY() - waypoints.get(a).getY(),
                    waypoints.get(a + 1).getX() - waypoints.get(a).getX()));

            pose.add(new Pose2d(waypoints.get(a), angleToNextWayPoint));
        }

        pose.add(new Pose2d(waypoints.get(waypoints.size() - 1), endAngle));

        QuinticSpline b = new QuinticSpline(pose, horizontalScaler);
        splines = b.getSplines();
    }

    public double[] matrixRowToArray(SimpleMatrix matrix) {
        assert matrix.numRows() == 1;
        double[] returner = new double[matrix.numCols()];

        for(int i = 0; i < matrix.numCols(); i++) {
            returner[i] = matrix.get(0, i);
        }

        return returner;
    }

    public static SimpleMatrix getHermiteBasis() {
        SimpleMatrix matrix = new SimpleMatrix(new double[][]
        {
            new double[] {-06.0, -03.0, -00.5, +06.0, -03.0, +00.5},
            new double[] {+15.0, +08.0, +01.5, -15.0, +07.0, +01.0},
            new double[] {-10.0, -06.0, -01.5, +10.0, -04.0, +00.5},
            new double[] {+00.0, +00.0, +00.5, +00.0, +00.0, +00.0},
            new double[] {+00.0, +01.0, +00.0, +00.0, +00.0, +00.0},
            new double[] {+01.0, +00.0, +00.0, +00.0, +00.0, +00.0},
        });
        return matrix;
    }

    public List<Spline> getSplines() {
        return splines;
    }

    public List<PoseWithCurvature> getSplinePoints() {
        var splinePoints = new ArrayList<PoseWithCurvature>();

        splinePoints.add(splines.get(0).getPoint(0.0));

        for (int i = 0; i < splines.size(); i++) {
            var points = parameterizeSpline(splines.get(i));
            points.remove(0);
            splinePoints.addAll(points);
        }

        return splinePoints;
    }
}

class ControlVectorPair {
    private final Pose2d poseInitial;
    private final Pose2d poseFinal;
    private final double scalar;

    public ControlVectorPair(Pose2d pose1, Pose2d pose2, double _horizontalScalar) {
        poseInitial = pose1;
        poseFinal = pose2;
        scalar = _horizontalScalar * pose1.getTranslation().getDistance(pose2.getTranslation());
    }

    public SimpleMatrix getXControlMatrix() {
        return new SimpleMatrix(new double[][]
        {
            {poseInitial.getX()}, {poseInitial.getRotation().getCos() * scalar}, {0.0},
            {poseFinal.getX()}, {poseFinal.getRotation().getCos() * scalar}, {0.0}
        });
    }

    public SimpleMatrix getYControlMatrix() {
        return new SimpleMatrix(new double[][]
        {
            {poseInitial.getY()}, {poseInitial.getRotation().getSin() * scalar}, {0.0},
            {poseFinal.getY()}, {poseFinal.getRotation().getSin() * scalar}, {0.0}
        });
    }
}
