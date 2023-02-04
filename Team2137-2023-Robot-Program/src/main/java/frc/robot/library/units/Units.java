package frc.robot.library.units;

import static frc.robot.library.units.Units.System.*;
import static frc.robot.library.units.Units.UnitType.*;

public class Units {
    public enum Unit {
        SCALAR          (Scalar, Generic, 1.0,"SCALAR"),

        RADIAN          (Angle, Generic, 1.0,"Rad"),
        DEGREE          (Angle, Generic,57.2957795, "Deg"),

        RADIAN_PER_SECOND (AngularVelocity, Generic, 1.0, "Rad/S"),
        DEGREE_PER_SECOND (AngularVelocity, Generic, 57.2957795, "Deg/S"),

        METER_PER_SECOND2 (Acceleration, Metric, 1.0,"M/S^2"),
        FEET_PER_SECOND2  (Acceleration, Imperial,3.280839895,"Ft/S^2"),

        METER_PER_SECOND (Velocity, Metric,1.0,"M/S"),
        FEET_PER_SECOND  (Velocity, Imperial,3.280839895,"F/S"),
        MILES_PER_HOUR   (Velocity, Imperial,2.23693629, "MPH"),
        CTRE_VELOCITY    (Velocity, Generic, Double.NaN, "CTRE_V"),

        METER           (Distance, Metric,1.0,"M"),
        CENTIMETER      (Distance, Metric,100.0,"CM"),
        MILLIMETER      (Distance, Metric, 1000.0,"MM"),

        FOOT            (Distance, Imperial, 3.2808399,"FT"),
        INCH            (Distance, Imperial,39.3700787, "IN"),

        SECOND          (Time, Generic, 1.0,"s"),
        MINUTE          (Time, Generic, 0.01666667,"m"),
        MILLISECOND     (Time, Generic, 1_000,"ms"),
        MICROSECOND     (Time, Generic, 1_000_000, "Ms"),
        NANOSECOND      (Time, Generic, 1_000_000_000,"ns")
        ;
        //to get primary divide by value

        UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        Unit(UnitType _unitType, System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        public double valueOf(double value, Unit unit) throws MissMatchedUnitTypeException {
            if(unitType == unit.unitType) {
                return (value / unit.unitsPerPrimary) * unitsPerPrimary;
            } else {
                throw new MissMatchedUnitTypeException();
            }
        }

        public static Unit getFromName(String name) {
            for(Unit a : Unit.values()) {
                for(String val : a.names) {
                    if(val.equals(name))
                        return a;
                }
            }

            return SCALAR;
        }

        public UnitType getUnitType() {
            return unitType;
        }
    }

    public enum System {
        Metric,
        Imperial,
        Generic
    }

    public enum UnitType {
        Angle,
        AngularVelocity,
        Acceleration,
        Velocity,
        Distance,
        Time,
        Scalar
    }

    public static class MissMatchedUnitTypeException extends Exception { }
}
