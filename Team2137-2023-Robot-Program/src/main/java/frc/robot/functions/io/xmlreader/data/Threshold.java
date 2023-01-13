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

public class Threshold extends Entity {
    private double upperValue;
    private double lowerValue;

    public Threshold(String name, int _upperValue, int _lowerValue) {
        super(name);
        this.upperValue = _upperValue;
        this.lowerValue = _lowerValue;
    }

    public Threshold(Element element) {
        super(element);
        this.upperValue = Double.parseDouble(getNodeOrAttribute(element, "High", "0.0"));
        this.lowerValue = Double.parseDouble(getNodeOrAttribute(element, "Low", "0.0"));
    }

    public synchronized double getUpperValue() {
        return upperValue;
    }

    public synchronized void setUpperValue(double upperValue) {
        this.upperValue = upperValue;
    }

    public synchronized double getLowerValue() {
        return lowerValue;
    }

    public synchronized void setLowerValue(double lowerValue) {
        this.lowerValue = lowerValue;
    }

    public boolean valueWithin(double val) {
        return val >= lowerValue && val <= upperValue;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        buildStringTabbedData(builder, depth, "Upper", String.valueOf(upperValue));
        buildStringTabbedData(builder, depth, "Lower", String.valueOf(lowerValue));
    }


    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry highEntry = table.getEntry("High");
        highEntry.setNumber(upperValue);

        NetworkTableEntry lowEntry = table.getEntry("Low");
        lowEntry.setNumber(lowerValue);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable(NetworkTable instance) {
        NetworkTable table = super.pullFromNetworkTable(instance);

        setUpperValue(table.getEntry("High").getDouble(getUpperValue()));
        setLowerValue(table.getEntry("Low").getDouble(getLowerValue()));

        return table;
    }

    @Override
    public NetworkTable removeFromNetworkTable(NetworkTable instance) {
        NetworkTable table = super.removeFromNetworkTable(instance);

        table.getEntry("High").unpublish();
        table.getEntry("Low").unpublish();

        return table;
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().getElementsByTagName("High").item(0).setTextContent(String.valueOf(upperValue));
        getSavedElement().getElementsByTagName("Low").item(0).setTextContent(String.valueOf(lowerValue));

        return getSavedElement();
    }
}
