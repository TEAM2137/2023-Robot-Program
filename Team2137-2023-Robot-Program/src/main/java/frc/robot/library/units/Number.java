//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package frc.robot.library.units;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

import java.lang.reflect.InvocationTargetException;

/**
 * This class represents a double value that is mutable and
 * has the ability to be published to the Smartdashboard
 * and to the XML file
 */
public class Number extends Entity implements MathFunctions<Number> {

    private double value;
    private Units.Unit ogUnit;

    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     * @param name - Name of the number
     * @param _value - Value of the Number
     */
    protected Number(String name, double _value) {
        this(name, _value, Units.Unit.SCALAR);
    }

    protected Number(double _value, Units.Unit _unit) {
        this(null, _value, _unit);
    }

    protected Number(String name, double _value, Units.Unit _unit) {
        super(name);

        ogUnit = _unit;
        value = _value;
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     * @param element - Number element from XML file
     */
    public Number(Element element) {
        super(element);

        value = Double.parseDouble(element.getTextContent());

        if (element.hasAttribute("unit")) {
            ogUnit = Units.Unit.getFromName(element.getAttribute("unit"));
        } else {
            ogUnit = Units.Unit.SCALAR;
        }
    }

    public static Class<? extends Number> getUnitClass(Units.Unit unit) {
        switch(unit.getUnitType()) {
            case Angle:
                return Angle.class;
            case AngularVelocity:
                return AngularVelocity.class;
            case Acceleration:
                return Acceleration.class;
            case Velocity:
                return Velocity.class;
            case Distance:
                return Distance.class;
            case Time:
                return Time.class;
            default:
                return Number.class;
        }
    }

    public static Number create(double val, Units.Unit unit) {
        switch(unit.getUnitType()) {
            case Angle:
                return new Angle(val, unit);
            case AngularVelocity:
                return new AngularVelocity(val, unit);
            case Acceleration:
                return new Acceleration(val, unit);
            case Velocity:
                return new Velocity(val, unit);
            case Distance:
                return new Distance(val, unit);
            case Time:
                return new Time(val, unit);
            default:
                return new Number(val, unit);
        }
    }

    public Number clone() {
        try {
            return this.getClass().getDeclaredConstructor(String.class, double.class, Units.Unit.class).newInstance(getName(), value, ogUnit);
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            e.printStackTrace();
        }

        return null;
    }

    public Units.Unit getPrimaryUnit() {
        return Units.Unit.SCALAR;
    }

    public Units.Unit getOriginalUnit() {
        return ogUnit;
    }

    public synchronized double getValueInDefaultUnit() {
        return getValue(ogUnit);
    }

    public synchronized double getValueInPrimaryUnit() {
        return getValue(getPrimaryUnit());
    }

    /**
     * Gets the current stored local value
     * @return - Value of current number
     */
    protected synchronized double getValue() {
        return value;
    }

    public synchronized double getValue(Units.Unit unit) {
        try {
            return unit.valueOf(value, ogUnit);
        } catch (Units.MissMatchedUnitTypeException e) {
            return Double.NaN;
        }
    }

    /**
     * Sets the value of the local values but does NOT change network table value or element value
     * @param value - Value to set current number to
     */
    protected synchronized void setValue(double value) {
        this.value = value;
    }

    public synchronized void setValue(double value, Units.Unit unit) {
        try {
            this.value = ogUnit.valueOf(value, unit);
        } catch (Units.MissMatchedUnitTypeException e) {
            e.printStackTrace();
        }
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry entry = table.getEntry("Value");
        entry.setDouble(value);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();

        setValue(table.getEntry("Value").getDouble(getValue()));

        return table;
    }

    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = super.removeFromNetworkTable();

        table.getEntry("Value").unpublish();

        return table;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        buildStringTabbedData(builder, depth, "Value", String.valueOf(value));
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().setTextContent(String.valueOf(value));

        return getSavedElement();
    }

    @Override
    public Number times(double scalar) {
        return Number.create(value * scalar, ogUnit);
    }

    @Override
    public Number divide(double scalar) {
        return Number.create(value / scalar, ogUnit);
    }

    @Override
    public Number add(double scalar) {
        return Number.create(value + scalar, ogUnit);
    }

    @Override
    public Number add(Number num) {
        return Number.create(value + num.getValue(ogUnit), ogUnit);
    }

    @Override
    public Number minus(double scalar) {
        return Number.create(value - scalar, ogUnit);
    }
}
