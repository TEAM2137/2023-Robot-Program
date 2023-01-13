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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.List;

public class Spline {
    private int m_degree;
    private SimpleMatrix coefficients;

    public Spline(int degree, SimpleMatrix coe) {
        m_degree = degree;
        coefficients = coe;
    }

    public Spline(SimpleMatrix coe) {
        m_degree = 5;
        coefficients = coe;
    }

    public SimpleMatrix getCoefficients() {
        return coefficients;
    }

    public PoseWithCurvature getPoint(double t) {
        SimpleMatrix polynomialBases = new SimpleMatrix(m_degree + 1, 1);
                //Matrix < double>.Build.DenseOfArray(new double[m_degree + 1, 1])

        for (int i = 0; i <= m_degree; i++) {
            polynomialBases.set(i, 0, Math.pow(t, m_degree - i));
        }

        SimpleMatrix mixed = coefficients.mult(polynomialBases);

        double x = mixed.get(0, 0);
        double y = mixed.get(1, 0);

        double dx, dy, ddx, ddy;

        if (t == 0) {
            dx = coefficients.get(2, m_degree - 1);
            dy = coefficients.get(3, m_degree - 1);
            ddx = coefficients.get(4, m_degree - 2);
            ddy = coefficients.get(5, m_degree - 2);
        } else {
            dx = mixed.get(2, 0) / t;
            dy = mixed.get(3, 0) / t;

            ddx = mixed.get(4, 0) / t / t;
            ddy = mixed.get(5, 0) / t / t;
        }

        double curve = (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)));

        return new PoseWithCurvature(new Pose2d(new Translation2d(x, y), new Rotation2d(dx, dy)), curve);
    }

    public static List<PoseWithCurvature> parameterizeSpline(Spline spline, double t0, double t1, double kMaxDy, double kMaxDx, double kMaxDTheata, double kMaxIterations){
        var splinePoints = new ArrayList<PoseWithCurvature>();

        splinePoints.add(spline.getPoint(t0));

        Translation2d current;
        PoseWithCurvature start;
        PoseWithCurvature end;

        List<Translation2d> toRun = new ArrayList<Translation2d>();
        toRun.add(new Translation2d(t0, t1));

        int index = 0;

        while (toRun.size() != 0) {
            current = toRun.get(0);
            toRun.remove(0);
            start = spline.getPoint(current.getX());
            end = spline.getPoint(current.getY());

            var twist = start.poseMeters.log(end.poseMeters);

            //Console.WriteLine(start.pose.getRotation().getRadians() + "," + end.pose.getRotation().getRadians()); This is fine
            //Console.WriteLine(twist.getAngularComponent().getRadians());
            //Console.WriteLine(current.getX() + "," + current.getY() + "," + Math.Abs(twist.getLinearComponent().getY()) + "," + Math.Abs(twist.getLinearComponent().getX()) + "," + Math.Abs(twist.getAngularComponent().getRadians()) + ","  + start.pose.getX() + "," + start.pose.getY() + "," + end.pose.getX() + "," + end.pose.getY());

            if (Math.abs(twist.dy) > kMaxDy
                    || Math.abs(twist.dx) > kMaxDx
                    || Math.abs(twist.dtheta) > kMaxDTheata) {
                toRun.add(0, new Translation2d((current.getX() + current.getY()) / 2, current.getY()));
                //Console.WriteLine("Adding " + ((current.getX() + current.getY()) / 2) + " and " + current.getY());

                toRun.add(0, new Translation2d(current.getX(), (current.getX() + current.getY()) / 2));
                //Console.WriteLine("Adding " + (current.getX()) + " and " + ((current.getX() + current.getY()) / 2));
            } else {
                splinePoints.add(spline.getPoint(current.getY()));
            }

            index++;

            if (index >= kMaxIterations) {
                System.out.println("Could Not parameterize a malformed spline");
            }
        }

        return splinePoints;
    }

    public static List<PoseWithCurvature> parameterizeSpline(Spline spline) {
        return parameterizeSpline(spline, 0.0, 1.0, 0.00127, 0.127, 0.0872, 5000);
    }
}