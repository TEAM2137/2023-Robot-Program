package frc.robot.library.units.TranslationalUnits;

import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Time;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.UnitUtil.UnitType.Velocity;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.CTRE_VELOCITY;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class Velocity implements TranslationUnit<Velocity, Velocity.VelocityUnits> {

    public enum VelocityUnits implements UnitEnum {
        METER_PER_SECOND (Velocity, METER, SECONDS,1.0,"M/S"),
        FEET_PER_SECOND  (Velocity, METER, SECONDS,3.280839895,"F/S"),
        MILES_PER_HOUR   (Velocity, METER, SECONDS,2.23693629, "MPH"),
        CTRE_VELOCITY    (Velocity, METER, SECONDS, 1.0, "CTRE_V");

        UnitUtil.UnitType unitType;
        Distance.DistanceUnits distanceUnits;
        Time.TimeUnits timeUnits;
        double unitsPerPrimary;
        String[] names;

        VelocityUnits(UnitUtil.UnitType _unitType, Distance.DistanceUnits distUnit, Time.TimeUnits timeUnit, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            distanceUnits = distUnit;
            timeUnits = timeUnit;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return METER_PER_SECOND;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for(frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits a : frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.values()) {
                for(String val : a.names) {
                    if(val.equals(name))
                        return a;
                }
            }

            return null;
        }

        @Override
        public UnitUtil.UnitType getUnitType() {
            return unitType;
        }

        public Distance.DistanceUnits getDistanceUnit() {
            return distanceUnits;
        }

        public Time.TimeUnits getTimeUnit() {
            return timeUnits;
        }
    }

    private double value;

    public Velocity(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    public Velocity(double x, double y, frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits unit) {
        this(Math.atan2(y, x), unit);
    }

    @Override
    public Velocity times(double scalar) {
        return new Velocity(getValue(METER_PER_SECOND) * scalar, METER_PER_SECOND);
    }
    @Override
    public Velocity times(Velocity other) {
        return new Velocity(getValue(METER_PER_SECOND) * other.getValue(METER_PER_SECOND), METER_PER_SECOND);
    }
    public Distance times(Time dt) {
        return new Distance(value * dt.getValue(SECONDS), METER);
    }

    @Override
    public Velocity divide(double scalar) {
        return new Velocity(getValue(METER_PER_SECOND) / scalar, METER_PER_SECOND);
    }
    @Override
    public Velocity divide(Velocity other) {
        return new Velocity(getValue(METER_PER_SECOND) / other.getValue(METER_PER_SECOND), METER_PER_SECOND);
    }
    public Acceleration divide(Time dt) {
        return new Acceleration(value / dt.getValue(SECONDS), METER);
    }
    public AngularVelocity divide(Distance dia) {
        return new AngularVelocity(value / (dia.getValue(METER) / 2), RADIAN_PER_SECOND);
    }
    public Time divide(Acceleration acceleration) {
        return new Time(value / acceleration.getValue(Acceleration.AccelerationUnits.METER_PER_SECOND2), SECONDS);
    }

    @Override
    public Velocity add(double scalar) {
        return new Velocity(getValue(METER_PER_SECOND) + scalar, METER_PER_SECOND);
    }
    @Override
    public Velocity add(Velocity other) {
        return new Velocity(getValue(METER_PER_SECOND) + other.getValue(METER_PER_SECOND), METER_PER_SECOND);
    }

    @Override
    public Velocity minus(double scalar) {
        return new Velocity(getValue(METER_PER_SECOND) - scalar, METER_PER_SECOND);
    }
    @Override
    public Velocity minus(Velocity other) {
        return new Velocity(getValue(METER_PER_SECOND) - other.getValue(METER_PER_SECOND), METER_PER_SECOND);
    }

    @Override
    public void setValue(double val, frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(METER_PER_SECOND);
    }

    @Override
    public VelocityUnits getPrimaryUnit() {
        return METER_PER_SECOND;
    }

    public Velocity getCTREVelocityUnit(Distance wheelConversionFactor) {
        return getCTREVelocityUnit(wheelConversionFactor, 2048);
    }

    public Velocity getCTREVelocityUnit(Distance wheelConversionFactor, double countsPerRev) {
        double rotationPerSec = (wheelConversionFactor.getValue(METER) * getValue(METER_PER_SECOND));
        return new Velocity((rotationPerSec * countsPerRev) / 10, CTRE_VELOCITY); //Return counts per 100ms
    }
}
