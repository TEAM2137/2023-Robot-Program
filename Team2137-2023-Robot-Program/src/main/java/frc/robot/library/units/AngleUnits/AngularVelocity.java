package frc.robot.library.units.AngleUnits;

import frc.robot.library.units.Time;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.AngularVelocity;

public class AngularVelocity implements AngleUnit<AngularVelocity, AngularVelocity.AngularVelocityUnits> {

    public enum AngularVelocityUnits implements UnitEnum {
        RADIAN_PER_SECOND (AngularVelocity, Generic, 1.0, "Rad/S"),
        ROTATION_PER_MINUTE (AngularVelocity, Generic, 9.5492965964254, "RPM"),
        DEGREE_PER_SECOND (AngularVelocity, Generic, 57.2957795, "Deg/S");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        AngularVelocityUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return RADIAN_PER_SECOND;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits a : frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.values()) {
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

    public AngularVelocity(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    public AngularVelocity(double x, double y, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits unit) {
        this(Math.atan2(y, x), unit);
    }

    @Override
    public AngularVelocity times(double scalar) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) * scalar, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }
    @Override
    public AngularVelocity times(frc.robot.library.units.AngleUnits.AngularVelocity other) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) * other.getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND), frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }
    public Angle times(Time dt) {
        return new Angle(getValue(AngularVelocityUnits.RADIAN_PER_SECOND) * dt.getValue(Time.TimeUnits.SECONDS), RADIAN);
    }

    @Override
    public AngularVelocity divide(double scalar) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) / scalar, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public frc.robot.library.units.AngleUnits.AngularVelocity divide(frc.robot.library.units.AngleUnits.AngularVelocity other) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) / other.getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND), frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public AngularVelocity add(double scalar) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) + scalar, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public AngularVelocity add(AngularVelocity other) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) + other.getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND), frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public AngularVelocity minus(double scalar) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) - scalar, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public frc.robot.library.units.AngleUnits.AngularVelocity minus(frc.robot.library.units.AngleUnits.AngularVelocity other) {
        return new AngularVelocity(getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND) - other.getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND), frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public void setValue(double val, frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND);
    }

    @Override
    public AngularVelocityUnits getPrimaryUnit() {
        return AngularVelocityUnits.RADIAN_PER_SECOND;
    }
}