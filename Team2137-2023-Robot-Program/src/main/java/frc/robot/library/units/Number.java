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
import frc.robot.library.units.AngleUnits.AngleUnit;
import frc.robot.library.units.TranslationalUnits.TranslationUnit;
import org.w3c.dom.Element;

import static frc.robot.library.units.Number.NumberUnit.SCALAR;
import static frc.robot.library.units.UnitUtil.System.Generic;
import static frc.robot.library.units.UnitUtil.UnitType.Scalar;

/**
 * This class represents a double value that is mutable and
 * has the ability to be published to the Smartdashboard
 * and to the XML file
 */
public class Number extends Entity implements AngleUnit<Number, Number.NumberUnit>, TranslationUnit<Number, Number.NumberUnit> {

    public enum NumberUnit implements UnitEnum {
        SCALAR          (Scalar, Generic, 1.0,"Scalar");

        UnitUtil.UnitType unitType;
        double unitsPerPrimary;
        String[] names;

        NumberUnit(UnitUtil.UnitType _unitType, UnitUtil.System unitSystem, double _UnitsPerPrimary, String... _names) {
            names = _names;
            unitType = _unitType;
            unitsPerPrimary = _UnitsPerPrimary;
        }

        @Override
        public UnitEnum getPrimaryUnit() {
            return SCALAR;
        }

        @Override
        public double getUnitPerPrimaryUnit() {
            return unitsPerPrimary;
        }

        @Override
        public UnitEnum getFromName(String name) {
            for(NumberUnit a : NumberUnit.values()) {
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

    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     * @param name - Name of the number
     * @param _value - Value of the Number
     */
    protected Number(String name, double _value) {
        super(name);
        value = _value;
    }

    public Number(double _value) {
        this(null, _value);
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     * @param element - Number element from XML file
     */
    public Number(Element element) {
        super(element);

        value = Double.parseDouble(element.getTextContent());
    }

    /**
     * Gets the current stored local value
     * @return - Value of current number
     */
    public synchronized double getValue() {
        return value;
    }

    /**
     * Sets the value of the local values but does NOT change network table value or element value
     * @param value - Value to set current number to
     */
    public synchronized void setValue(double value) {
        this.value = value;
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
        return new Number(getValue() - scalar);
    }

    @Override
    public Number times(Number other) {
        return new Number(getValue() - other.getValue());
    }

    @Override
    public Number divide(double scalar) {
        return new Number(getValue() - scalar);
    }

    @Override
    public Number divide(Number other) {
        return new Number(getValue() - other.getValue());
    }

    @Override
    public Number add(double scalar) {
        return new Number(getValue() + scalar);
    }

    @Override
    public Number add(Number other) {
        return new Number(getValue() + other.getValue());
    }

    @Override
    public Number minus(double scalar) {
        return new Number(getValue() - scalar);
    }

    @Override
    public Number minus(Number other) {
        return new Number(getValue() - other.getValue());
    }

    @Override
    public void setValue(double val, NumberUnit unit) {
        setValue(val);
    }

    @Override
    public double getValue(NumberUnit unit) {
        return getValue();
    }

    @Override
    public double getValueInPrimaryUnit() {
        return getValue();
    }

    @Override
    public NumberUnit getPrimaryUnit() {
        return SCALAR;
    }
}
