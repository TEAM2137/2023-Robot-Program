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

public class Time2d {
    public enum TimeUnits {
        MILLISECONDS (1000.0, "Milliseconds"),
        SECONDS (1.0, "Seconds"),
        MINUTES (1.0/60.0, "Minutes"),
        HOURS (1.0/3600.0, "Hours");

        public double secondToUnit;
        private final String name;

        TimeUnits(double _secondsToUnit, String _name) {
            secondToUnit = _secondsToUnit;
            name = _name;
        }

        public String getName() {
            return name;
        }
    }

    private final double secondsValue;

    private Time2d(double value) {
        secondsValue = value;
    }

    public static Time2d fromUnit(TimeUnits unit, double otherUnit) {
        return new Time2d(otherUnit / unit.secondToUnit);
    }

    public static Time2d fromSeconds(double seconds) {
        return new Time2d(seconds);
    }

    public double getValue(TimeUnits unit) {
        return secondsValue * unit.secondToUnit;
    }
}
