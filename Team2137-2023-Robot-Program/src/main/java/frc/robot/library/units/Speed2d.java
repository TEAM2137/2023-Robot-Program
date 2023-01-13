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

public class Speed2d {

    private final Distance2d distance;
    private final Time2d time;

    public Speed2d(Distance2d.DistanceUnits distanceUnits, Time2d.TimeUnits timeUnits, double value) {
        distance = Distance2d.fromUnit(distanceUnits, value);
        time = Time2d.fromUnit(timeUnits, 1);
    }

    public Speed2d(Distance2d distance, Time2d time) {
        this(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, distance.getValue(Distance2d.DistanceUnits.FEET)/time.getValue(Time2d.TimeUnits.SECONDS));
    }

    public Speed2d(double value) {
        this(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, value);
    }

    public double getValue(Distance2d.DistanceUnits distanceUnit, Time2d.TimeUnits timeUnit) {
        return distance.getValue(distanceUnit) / time.getValue(timeUnit);
    }

    public double getValue() {
        return getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS);
    }

    public static Speed2d fromFeetPerSecond(double value) {
        return new Speed2d(value);
    }

    public double getCTREVelocityUnit(Distance2d wheelConversionFactor) {
        return getCTREVelocityUnit(wheelConversionFactor, 2048);
    }

    public double getCTREVelocityUnit(Distance2d wheelConversionFactor, double countsPerRev) {
        double rotationPerSec = (wheelConversionFactor.getValue(Distance2d.DistanceUnits.FEET) * getValue());
        return (rotationPerSec * countsPerRev) / 10; //Return counts per 100ms
    }

    @Override
    public String toString() {
        return distance.getValue(Distance2d.DistanceUnits.FEET)/time.getValue(Time2d.TimeUnits.SECONDS) + " F/S";
    }
}
