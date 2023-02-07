package frc.robot.library.units;

import frc.robot.library.units.AngleUnits.Angle;

import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.Time;

public class Time implements Unit<Time, frc.robot.library.units.Time.TimeUnits> {

    public enum TimeUnits implements UnitEnum {        
        SECONDS          (Time, Generic, 1.0,"s"),
        MINUTE          (Time, Generic, 0.01666667,"m"),
        MILLISECONDS     (Time, Generic, 1_000,"ms"),
        MICROSECONDS     (Time, Generic, 1_000_000, "Ms"),
        NANOSECONDS      (Time, Generic, 1_000_000_000,"ns");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        TimeUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return SECONDS;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for (TimeUnits a : TimeUnits.values()) {
                for (String val : a.names) {
                    if (val.equals(name))
                        return a;
                }
            }

            return null;
        }

        @Override
        public UnitUtil.UnitType getUnitType() {
            return unitType;
        }
    }

    private double value;

    public Time(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public Time times(double scalar) {
        return new Time(getValue(SECONDS) * scalar, SECONDS);
    }

    @Override
    public Time times(Time other) {
        return new Time(getValue(SECONDS) * other.getValue(SECONDS), SECONDS);
    }

    @Override
    public Time divide(double scalar) {
        return new Time(getValue(SECONDS) / scalar, SECONDS);
    }

    @Override
    public Time divide(Time other) {
        return new Time(getValue(SECONDS) / other.getValue(SECONDS), SECONDS);
    }

    @Override
    public Time add(double scalar) {
        return new Time(getValue(SECONDS) + scalar, SECONDS);
    }

    @Override
    public Time add(Time other) {
        return new Time(getValue(SECONDS) + other.getValue(SECONDS), SECONDS);
    }

    @Override
    public Time minus(double scalar) {
        return new Time(getValue(SECONDS) - scalar, SECONDS);
    }

    @Override
    public Time minus(Time other) {
        return new Time(getValue(SECONDS) - other.getValue(SECONDS), SECONDS);
    }

    @Override
    public void setValue(double val, frc.robot.library.units.Time.TimeUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(frc.robot.library.units.Time.TimeUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(SECONDS);
    }

    @Override
    public TimeUnits getPrimaryUnit() {
        return SECONDS;
    }
}
