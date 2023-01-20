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
import edu.wpi.first.math.spline.PoseWithCurvature;
import frc.robot.library.units.Speed2d;

import java.util.ArrayList;
import java.util.List;

public class VelocityGenerator {

    private final List<Speed2d> speeds = new ArrayList<>();

    //Corner percent is the amount to stretch the original slow down determined by the percent of curve (degree change / 180)
    public VelocityGenerator(List<PoseWithCurvature> poses, Speed2d maxDrivetrainVelocity, Speed2d maxDrivetrainAcceleration, double cornerPercent) {//, Speed2d startSpeed, Speed2d endSpeed) {
        //Max RadPerMeter is 2Pi
        speeds.add(Speed2d.fromFeetPerSecond(0));

        for(int i = 1; i < poses.size(); i++) {
            double previousSpeed = speeds.get(i - 1).getValue();
            double maxAccel = maxDrivetrainAcceleration.getValue();
            double distanceBetweenWaypoint = Math.sqrt(
                    Math.pow(poses.get(i).poseMeters.getX() - poses.get(i - 1).poseMeters.getX(), 2) +
                    Math.pow(poses.get(i).poseMeters.getY() - poses.get(i - 1).poseMeters.getY(), 2));

            double maxAccelSpeed = Math.sqrt(Math.pow(previousSpeed, 2) + (2 * maxAccel * distanceBetweenWaypoint));
            double curveSpeed = (1 - Math.abs(poses.get(i).curvatureRadPerMeter / Math.PI)) * maxDrivetrainVelocity.getValue();
            double stretchedValue = ((curveSpeed - maxDrivetrainVelocity.getValue()) * cornerPercent) + maxDrivetrainVelocity.getValue();

            if(stretchedValue < maxAccelSpeed)
                speeds.add(Speed2d.fromFeetPerSecond(stretchedValue));
            else
                speeds.add(Speed2d.fromFeetPerSecond(maxAccelSpeed));
        }

        speeds.set(speeds.size() - 1, Speed2d.fromFeetPerSecond(0));

        for(int i = poses.size() - 2; i >= 0; i--) {
            double previousSpeed = speeds.get(i + 1).getValue();
            double maxAccel = maxDrivetrainAcceleration.getValue();
            double distanceBetweenWaypoint = Math.sqrt(
                    Math.pow(poses.get(i).poseMeters.getX() - poses.get(i + 1).poseMeters.getX(), 2) +
                            Math.pow(poses.get(i).poseMeters.getY() - poses.get(i + 1).poseMeters.getY(), 2));

            double maxAccelSpeed = Math.sqrt(Math.pow(previousSpeed, 2) + (2 * maxAccel * distanceBetweenWaypoint));
            double curveSpeed = (1 - Math.abs(poses.get(i).curvatureRadPerMeter / Math.PI)) * maxDrivetrainVelocity.getValue();
            double stretchedValue = ((curveSpeed - maxDrivetrainVelocity.getValue()) * cornerPercent) + maxDrivetrainVelocity.getValue();
            Speed2d speed;

            if(stretchedValue < maxAccelSpeed)
                speed = Speed2d.fromFeetPerSecond(stretchedValue);
            else
                speed = Speed2d.fromFeetPerSecond(maxAccelSpeed);

            if(speed.getValue() < speeds.get(i).getValue())
                speeds.set(i, speed);
        }
    }

    public List<Speed2d> getSpeeds() {
        return speeds;
    }
}
