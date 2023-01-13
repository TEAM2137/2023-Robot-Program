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

package frc.robot.library.units;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.library.units.Distance2d.DistanceUnits;

public class Vector2d {
    double x;
    double y;

    public Vector2d(Distance2d _x, Distance2d _y) {
        x = _x.getValue(DistanceUnits.INCH);
        y = _y.getValue(DistanceUnits.INCH);
    }

    public Vector2d normalize() {
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new Vector2d(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x / length), Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y / length));
    }

    public Vector2d scale(double scale) {
        return new Vector2d(Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x * scale), Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y * scale));
    }

    public Distance2d getX() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, x);
    }

    public Distance2d getY() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, y);
    }

    /**
     * Will return in Inches
     * @param translation2d
     * @return
     */
    public Translation2d applyVector(Translation2d translation2d) {
        return new Translation2d(translation2d.getX() + this.x, translation2d.getY() + this.y);
    }

    public double slope() {
        return y / x;
    }
}
