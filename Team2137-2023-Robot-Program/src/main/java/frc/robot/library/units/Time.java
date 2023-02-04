package frc.robot.library.units;

import org.w3c.dom.Element;

public class Time extends Number {
    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     *
     * @param name   - Name of the number
     * @param _value - Value of the Number
     */
    public Time(String name, double _value) {
        super(name, _value);
    }

    public Time(double _value, Units.Unit _unit) {
        super(_value, _unit);
    }
    public Time(String name, double _value, Units.Unit _unit) {
        super(name, _value, _unit);
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     *
     * @param element - Number element from XML file
     */
    public Time(Element element) {
        super(element);
    }

    @Override
    public Units.Unit getPrimaryUnit() {
        return Units.Unit.SECOND;
    }
}
