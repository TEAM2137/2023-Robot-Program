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

package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

/**
 * This class represents a double value that is mutable and
 * has the ability to be published to the Smartdashboard
 * and to the XML file
 */
public class Number extends Entity {

    private double value;

    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     * @param name - Name of the number
     * @param _value - Value of the Number
     */
    public Number(String name, double _value) {
        super(name);

        value = _value;
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
}
