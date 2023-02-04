package frc.robot.library.units;

import org.w3c.dom.Element;

public class Angle extends Number {
    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     *
     * @param name   - Name of the number
     * @param _value - Value of the Number
     */
    public Angle(String name, double _value) {
        super(name, _value);
    }

    public Angle(double val, Units.Unit unit) {
        super(val, unit);
    }

    public Angle(double x, double y, Units.Unit unit) {
        super(Math.atan2(y, x), unit);
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     *
     * @param element - Number element from XML file
     */
    public Angle(Element element) {
        super(element);
    }

    @Override
    public Units.Unit getPrimaryUnit() {
        return Units.Unit.RADIAN;
    }
}
