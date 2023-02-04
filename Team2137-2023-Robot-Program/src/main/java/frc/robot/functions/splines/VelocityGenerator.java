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
import frc.robot.library.units.Acceleration;
import frc.robot.library.units.Units;
import frc.robot.library.units.Velocity;

import java.util.ArrayList;
import java.util.List;

public class VelocityGenerator {

    private final List<Velocity> speeds = new ArrayList<>();

    //Corner percent is the amount to stretch the original slow down determined by the percent of curve (degree change / 180)
    public VelocityGenerator(List<PoseWithCurvature> poses, Velocity maxDrivetrainVelocity, Acceleration maxDrivetrainAcceleration, double cornerPercent) {//, Velocity startSpeed, Velocity endSpeed) {
        //Max RadPerMeter is 2Pi
        speeds.add(new Velocity(0, Units.Unit.FEET_PER_SECOND));

        for(int i = 1; i < poses.size(); i++) {
            double previousSpeed = speeds.get(i - 1).getValue(Units.Unit.FEET_PER_SECOND);
            double maxAccel = maxDrivetrainAcceleration.getValue(Units.Unit.FEET_PER_SECOND2);
            double distanceBetweenWaypoint = Math.sqrt(
                    Math.pow(poses.get(i).poseMeters.getX() - poses.get(i - 1).poseMeters.getX(), 2) +
                    Math.pow(poses.get(i).poseMeters.getY() - poses.get(i - 1).poseMeters.getY(), 2));

            double maxAccelSpeed = Math.sqrt(Math.pow(previousSpeed, 2) + (2 * maxAccel * distanceBetweenWaypoint));
            double curveSpeed = (1 - Math.abs(poses.get(i).curvatureRadPerMeter / Math.PI)) * maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND);
            double stretchedValue = ((curveSpeed - maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND)) * cornerPercent) + maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND);

            if(stretchedValue < maxAccelSpeed)
                speeds.add(new Velocity(stretchedValue, Units.Unit.FEET_PER_SECOND));
            else
                speeds.add(new Velocity(maxAccelSpeed, Units.Unit.FEET_PER_SECOND));
        }

        speeds.set(speeds.size() - 1, new Velocity(0, Units.Unit.FEET_PER_SECOND));

        for(int i = poses.size() - 2; i >= 0; i--) {
            double previousSpeed = speeds.get(i + 1).getValue(Units.Unit.FEET_PER_SECOND);
            double maxAccel = maxDrivetrainAcceleration.getValue(Units.Unit.FEET_PER_SECOND2);
            double distanceBetweenWaypoint = Math.sqrt(
                    Math.pow(poses.get(i).poseMeters.getX() - poses.get(i + 1).poseMeters.getX(), 2) +
                            Math.pow(poses.get(i).poseMeters.getY() - poses.get(i + 1).poseMeters.getY(), 2));

            double maxAccelSpeed = Math.sqrt(Math.pow(previousSpeed, 2) + (2 * maxAccel * distanceBetweenWaypoint));
            double curveSpeed = (1 - Math.abs(poses.get(i).curvatureRadPerMeter / Math.PI)) * maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND);
            double stretchedValue = ((curveSpeed - maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND)) * cornerPercent) + maxDrivetrainVelocity.getValue(Units.Unit.FEET_PER_SECOND);
            Velocity speed;

            if(stretchedValue < maxAccelSpeed)
                speed = new Velocity(stretchedValue, Units.Unit.FEET_PER_SECOND);
            else
                speed = new Velocity(maxAccelSpeed, Units.Unit.FEET_PER_SECOND);

            if(speed.getValue(Units.Unit.FEET_PER_SECOND) < speeds.get(i).getValue(Units.Unit.FEET_PER_SECOND))
                speeds.set(i, speed);
        }
    }

    public List<Velocity> getSpeeds() {
        return speeds;
    }
}
