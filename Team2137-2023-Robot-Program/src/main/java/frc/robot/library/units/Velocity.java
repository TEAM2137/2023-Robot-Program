package frc.robot.library.units;

import org.w3c.dom.Element;

import static frc.robot.library.units.Units.Unit.CTRE_VELOCITY;
import static frc.robot.library.units.Units.Unit.METER_PER_SECOND;

public class Velocity extends Number {
    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     *
     * @param name   - Name of the number
     * @param _value - Value of the Number
     */
    public Velocity(String name, double _value) {
        super(name, _value);
    }

    public Velocity(double _value, Units.Unit _unit) {
        super(_value, _unit);
    }

    public Velocity(String name, double _value, Units.Unit _unit) {
        super(name, _value, _unit);
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     *
     * @param element - Number element from XML file
     */
    public Velocity(Element element) {
        super(element);
    }

    @Override
    public Units.Unit getPrimaryUnit() {
        return METER_PER_SECOND;
    }

    public Velocity getCTREVelocityUnit(Distance wheelConversionFactor) {
        return getCTREVelocityUnit(wheelConversionFactor, 2048);
    }

    public Velocity getCTREVelocityUnit(Distance wheelConversionFactor, double countsPerRev) {
        double rotationPerSec = (wheelConversionFactor.getValue(Units.Unit.FOOT) * getValue());
        return new Velocity((rotationPerSec * countsPerRev) / 10, CTRE_VELOCITY); //Return counts per 100ms
    }
}
