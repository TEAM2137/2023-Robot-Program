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

public class Distance2d {
    public enum DistanceUnits {
        //Metric
        METER (0.3048, "Meters"),
        CENTIMETER (30.48, "Centimeters"),
        MILLIMETER (304.8, "Millimeters"),

        //US Custom
        FEET (1.0, "Feet"),
        INCH (12.0, "Inches");

        private final double feetToUnit;
        private final String name;

        DistanceUnits(double _feetToUnit, String _name) {
            feetToUnit = _feetToUnit;
            name = _name;
        }

        public String getName() {
            return name;
        }
    }

    private double distanceFeet;

    private Distance2d(double value) {
        distanceFeet = value;
    }

    public static Distance2d fromUnit(DistanceUnits unit, double otherUnit) {
        return new Distance2d(otherUnit / unit.feetToUnit);
    }

    public static Distance2d fromFeet(double feet) {
        return new Distance2d(feet);
    }

    public double getValue(DistanceUnits unit) {
        return distanceFeet * unit.feetToUnit;
    }

    public Distance2d minus(Distance2d distance2d) {
        return Distance2d.fromFeet(distanceFeet - distance2d.getValue(DistanceUnits.FEET));
    }

    public void mutableMinus(Distance2d distance2d) {
        mutableMinus(distance2d.getValue(DistanceUnits.FEET));
    }

    public void mutableMinus(double valInFeet) {
        distanceFeet -= valInFeet;
    }

    public Distance2d plus(Distance2d distance2d) {
        return Distance2d.fromFeet(distanceFeet + distance2d.getValue(DistanceUnits.FEET));
    }

    public void mutablePlus(Distance2d distance2d) {
        mutablePlus(distance2d.getValue(DistanceUnits.FEET));
    }

    public void mutablePlus(double valInFeet) {
        distanceFeet += valInFeet;
    }
}
