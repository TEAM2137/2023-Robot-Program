package frc.robot.library.units.TranslationalUnits;

import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;
import org.w3c.dom.Element;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.UnitUtil.System.Imperial;
import static frc.robot.library.units.UnitUtil.System.Metric;
import static frc.robot.library.units.UnitUtil.UnitType.Distance;

public class Distance extends EntityImpl implements TranslationUnit<Distance, frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits> {

    public enum DistanceUnits implements UnitEnum {
        METER           (Distance, Metric,1.0,"M"),
        CENTIMETER      (Distance, Metric,100.0,"CM"),
        MILLIMETER      (Distance, Metric, 1000.0,"MM"),

        FOOT            (Distance, Imperial, 3.2808399,"FT"),
        INCH            (Distance, Imperial,39.3700787, "IN");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        DistanceUnits(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return METER;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for (DistanceUnits a : DistanceUnits.values()) {
                for (String val : a.names) {
                    if (val.equalsIgnoreCase(name))
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

    public Distance(double val, UnitEnum unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    public Distance(Element element) {
        super(element);

        UnitEnum elementUnit = DistanceUnits.INCH.getFromName(getNodeOrAttribute(element, "unit", null));

        value = Double.parseDouble(element.getTextContent()) / elementUnit.getUnitPerPrimaryUnit();
    }

    @Override
    public Distance times(double scalar) {
        return new Distance(getValue(METER) * scalar, METER);
    }

    @Override
    public Distance times(Distance other) {
        return new Distance(getValue(METER) * other.getValue(METER), METER);
    }

    @Override
    public Distance divide(double scalar) {
        return new Distance(getValue(METER) / scalar, METER);
    }

    @Override
    public Distance divide(Distance other) {
        return new Distance(getValue(METER) / other.getValue(METER), METER);
    }


    @Override
    public Distance add(double scalar) {
        return new Distance(getValue(METER) + scalar, METER);
    }

    @Override
    public Distance add(Distance other) {
        return new Distance(getValue(METER) + other.getValue(METER), METER);
    }

    @Override
    public Distance minus(double scalar) {
        return new Distance(getValue(METER) - scalar, METER);
    }

    @Override
    public Distance minus(Distance other) {
        return new Distance(getValue(METER) - other.getValue(METER), METER);
    }

    @Override
    public void setValue(double val, frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits unit) {
        value = val / unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValue(frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits unit) {
        return value * unit.getUnitPerPrimaryUnit();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue(METER);
    }

    @Override
    public DistanceUnits getPrimaryUnit() {
        return METER;
    }

    public boolean greaterThan(Distance other) {
        return getValue(FOOT) > other.getValue(FOOT);
    }
}
