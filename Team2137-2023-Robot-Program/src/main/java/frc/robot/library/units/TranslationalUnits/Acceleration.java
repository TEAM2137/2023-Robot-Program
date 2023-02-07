package frc.robot.library.units.TranslationalUnits;

import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.Time;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.UnitUtil.System.*;
import static frc.robot.library.units.UnitUtil.UnitType.Acceleration;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class Acceleration implements TranslationUnit<Acceleration, frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits> {

    public enum AccelerationUnits implements UnitEnum {
        METER_PER_SECOND2 (Acceleration, Metric, 1.0,"M/S^2"),
        FEET_PER_SECOND2  (Acceleration, Imperial,3.280839895,"Ft/S^2");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        AccelerationUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return METER_PER_SECOND2;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for (AccelerationUnits a : AccelerationUnits.values()) {
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

    public Acceleration(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public Acceleration times(double scalar) {
        return new Acceleration(getValue(METER_PER_SECOND2) * scalar, METER_PER_SECOND2);
    }
    @Override
    public Acceleration times(Acceleration other) {
        return new Acceleration(getValue(METER_PER_SECOND2) * other.getValue(METER_PER_SECOND2), METER_PER_SECOND2);
    }
    public Velocity times(Time time) {
        return new Velocity(getValue(METER_PER_SECOND2) * time.getValue(Time.TimeUnits.SECONDS), METER_PER_SECOND);
    }

    @Override
    public Acceleration divide(double scalar) {
        return new Acceleration(getValue(METER_PER_SECOND2) / scalar, METER_PER_SECOND2);
    }

    @Override
    public Acceleration divide(Acceleration other) {
        return new Acceleration(getValue(METER_PER_SECOND2) / other.getValue(METER_PER_SECOND2), METER_PER_SECOND2);
    }

    @Override
    public Acceleration add(double scalar) {
        return new Acceleration(getValue(METER_PER_SECOND2) + scalar, METER_PER_SECOND2);
    }

    @Override
    public Acceleration add(Acceleration other) {
        return new Acceleration(getValue(METER_PER_SECOND2) + other.getValue(METER_PER_SECOND2), METER_PER_SECOND2);
    }

    @Override
    public Acceleration minus(double scalar) {
        return new Acceleration(getValue(METER_PER_SECOND2) - scalar, METER_PER_SECOND2);
    }

    @Override
    public Acceleration minus(Acceleration other) {
        return new Acceleration(getValue(METER_PER_SECOND2) - other.getValue(METER_PER_SECOND2), METER_PER_SECOND2);
    }

    @Override
    public void setValue(double val, AccelerationUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(AccelerationUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(METER_PER_SECOND2);
    }

    @Override
    public AccelerationUnits getPrimaryUnit() {
        return METER_PER_SECOND2;
    }
}
