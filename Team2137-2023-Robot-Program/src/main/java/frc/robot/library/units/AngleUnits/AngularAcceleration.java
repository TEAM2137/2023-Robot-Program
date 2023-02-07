package frc.robot.library.units.AngleUnits;

import frc.robot.library.units.Time;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.AngularAcceleration;

public class AngularAcceleration implements AngleUnit<AngularAcceleration, AngularAcceleration.AngularAccelerationUnits> {

    public enum AngularAccelerationUnits implements UnitEnum {
        RADIAN_PER_SECOND2 (AngularAcceleration, Generic, 1.0, "Rad/S^2"),
        DEGREE_PER_SECOND2 (AngularAcceleration, Generic, 57.2957795, "Deg/S^2");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        AngularAccelerationUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return RADIAN_PER_SECOND2;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits a : frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.values()) {
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
    }

    private double value;

    public AngularAcceleration(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    public AngularAcceleration(double x, double y, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits unit) {
        this(Math.atan2(y, x), unit);
    }

    @Override
    public AngularAcceleration times(double scalar) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) * scalar, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }
    @Override
    public AngularAcceleration times(frc.robot.library.units.AngleUnits.AngularAcceleration other) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) * other.getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2), frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }
    public Angle times(Time dt) {
        return new Angle(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) * dt.getValue(Time.TimeUnits.SECONDS), RADIAN);
    }

    @Override
    public AngularAcceleration divide(double scalar) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) / scalar, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public frc.robot.library.units.AngleUnits.AngularAcceleration divide(frc.robot.library.units.AngleUnits.AngularAcceleration other) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) / other.getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2), frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public AngularAcceleration add(double scalar) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) + scalar, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public AngularAcceleration add(AngularAcceleration other) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) + other.getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2), frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public AngularAcceleration minus(double scalar) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) - scalar, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public frc.robot.library.units.AngleUnits.AngularAcceleration minus(frc.robot.library.units.AngleUnits.AngularAcceleration other) {
        return new AngularAcceleration(getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2) - other.getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2), frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public void setValue(double val, frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2);
    }

    @Override
    public frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits getPrimaryUnit() {
        return frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2;
    }
}
