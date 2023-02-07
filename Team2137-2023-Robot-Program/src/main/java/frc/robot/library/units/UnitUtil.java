package frc.robot.library.units;

import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

public class UnitUtil {

    public enum System {
        Metric,
        Imperial,
        Generic
    }

    public enum UnitType {
        Angle,
        AngularVelocity,
        AngularAcceleration,
        Acceleration,
        Velocity,
        Distance,
        Time,
        Scalar
    }

    public static Class<? extends Unit<?, ?>> getUnitClass(UnitEnum unit) {
        switch(unit.getUnitType()) {
            case Angle:
                return Angle.class;
            case AngularVelocity:
                return AngularVelocity.class;
            case AngularAcceleration:
                return AngularAcceleration.class;
            case Acceleration:
                return Acceleration.class;
            case Velocity:
                return Velocity.class;
            case Distance:
                return Distance.class;
            case Time:
                return Time.class;
            default:
                return Number.class;
        }
    }

    public static Unit<?, ? extends UnitEnum> create(double val, UnitEnum unit) {
        switch(unit.getUnitType()) {
            case Angle:
                return new Angle(val, unit);
            case AngularVelocity:
                return new AngularVelocity(val, unit);
            case AngularAcceleration:
                return new AngularAcceleration(val, unit);
            case Acceleration:
                return new Acceleration(val, unit);
            case Velocity:
                return new Velocity(val, unit);
            case Distance:
                return new Distance(val, unit);
            case Time:
                return new Time(val, unit);
            default:
                return new Number(null, val);
        }
    }
}
