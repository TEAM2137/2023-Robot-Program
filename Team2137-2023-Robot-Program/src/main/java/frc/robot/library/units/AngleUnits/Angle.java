package frc.robot.library.units.AngleUnits;

import frc.robot.library.units.Time;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.Angle;

public class Angle implements AngleUnit<Angle, Angle.AngleUnits> {

    public enum AngleUnits implements UnitEnum {
        RADIAN          (Angle, Generic, 1.0,"Rad"),
        DEGREE          (Angle, Generic,57.2957795, "Deg"),
        REVOLUTIONS     (Angle, Generic, 0.159155, "Rev");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        AngleUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return RADIAN;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for(AngleUnits a : AngleUnits.values()) {
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

    public Angle(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    public Angle(double y, double x) {
        this(Math.atan2(y, x), RADIAN);
    }

    @Override
    public Angle times(double scalar) {
        return new Angle(getValue(RADIAN) * scalar, RADIAN);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle times(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(RADIAN) * other.getValue(RADIAN), RADIAN);
    }

    @Override
    public Angle divide(double scalar) {
        return new Angle(getValue(RADIAN) / scalar, RADIAN);
    }

    public AngularVelocity divide(Time dt) {
        return new AngularVelocity(getValue(AngleUnits.DEGREE) / dt.getValue(Time.TimeUnits.SECONDS), DEGREE_PER_SECOND);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle divide(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(RADIAN) / other.getValue(RADIAN), RADIAN);
    }

    @Override
    public Angle add(double scalar) {
        return new Angle(getValue(RADIAN) + scalar, RADIAN);
    }

    @Override
    public Angle add(Angle other) {
        return new Angle(getValue(RADIAN) + other.getValue(RADIAN), RADIAN);
    }

    @Override
    public Angle minus(double scalar) {
        return new Angle(getValue(RADIAN) - scalar, RADIAN);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle minus(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(RADIAN) - other.getValue(RADIAN), RADIAN);
    }

    @Override
    public void setValue(double val, AngleUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(AngleUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(RADIAN);
    }

    @Override
    public AngleUnits getPrimaryUnit() {
        return RADIAN;
    }
}
