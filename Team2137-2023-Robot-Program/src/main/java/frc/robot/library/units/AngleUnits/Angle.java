package frc.robot.library.units.AngleUnits;

import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.Angle;

public class Angle implements AngleUnit<Angle, Angle.AngleUnits> {

    public enum AngleUnits implements UnitEnum {
        RADIAN          (Angle, Generic, 1.0,"Rad"),
        DEGREE          (Angle, Generic,57.2957795, "Deg");

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

    public Angle(double x, double y, AngleUnits unit) {
        this(Math.atan2(y, x), unit);
    }

    @Override
    public Angle times(double scalar) {
        return new Angle(getValue(AngleUnits.RADIAN) * scalar, AngleUnits.RADIAN);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle times(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(AngleUnits.RADIAN) * other.getValue(AngleUnits.RADIAN), AngleUnits.RADIAN);
    }

    @Override
    public Angle divide(double scalar) {
        return new Angle(getValue(AngleUnits.RADIAN) / scalar, AngleUnits.RADIAN);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle divide(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(AngleUnits.RADIAN) / other.getValue(AngleUnits.RADIAN), AngleUnits.RADIAN);
    }

    @Override
    public Angle add(double scalar) {
        return new Angle(getValue(AngleUnits.RADIAN) + scalar, AngleUnits.RADIAN);
    }

    @Override
    public Angle add(Angle other) {
        return new Angle(getValue(AngleUnits.RADIAN) + other.getValue(AngleUnits.RADIAN), AngleUnits.RADIAN);
    }

    @Override
    public Angle minus(double scalar) {
        return new Angle(getValue(AngleUnits.RADIAN) - scalar, AngleUnits.RADIAN);
    }

    @Override
    public frc.robot.library.units.AngleUnits.Angle minus(frc.robot.library.units.AngleUnits.Angle other) {
        return new Angle(getValue(AngleUnits.RADIAN) - other.getValue(AngleUnits.RADIAN), AngleUnits.RADIAN);
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
        return getValue(AngleUnits.RADIAN);
    }

    @Override
    public AngleUnits getPrimaryUnit() {
        return AngleUnits.RADIAN;
    }
}
