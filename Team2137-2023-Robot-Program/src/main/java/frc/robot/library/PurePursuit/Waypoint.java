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
import frc.robot.library.units.Distance2d;

public class Waypoint {
    private Distance2d x;
    private Distance2d y;
    private Rotation2d heading;
    private double curvature;
    private Distance2d distanceFromStart;

    public Waypoint(Distance2d _x, Distance2d _y, Rotation2d _w, double _curvature, Distance2d _distanceFromStart) {
        x = _x;
        y = _y;
        heading = _w;
        curvature = _curvature;
        distanceFromStart = _distanceFromStart;
    }

    public Distance2d getX() {
        return x;
    }

    public void setX(Distance2d x) {
        this.x = x;
    }

    public Distance2d getY() {
        return y;
    }

    public void setY(Distance2d y) {
        this.y = y;
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }

    public double getCurvature() {
        return curvature;
    }

    public void setCurvature(double curvature) {
        this.curvature = curvature;
    }

    public Distance2d getDistanceFromStart() {
        return distanceFromStart;
    }

    public void setDistanceFromStart(Distance2d distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }
}
