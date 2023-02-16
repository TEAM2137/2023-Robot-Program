package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import org.w3c.dom.Element;

public class Toggle extends EntityImpl {
    private boolean value;

    /**
     * Constructs Number with simple Name and Value but DOES NOT have reference to XML Element
     * @param name - Name of the number
     * @param _value - Value of the Number
     */
    public Toggle(String name, boolean _value) {
        super(name);

        value = _value;
    }

    /**
     * Constructs Number with element from XML file and uses the Tag name as the name and Text content as value
     * @param element - Number element from XML file
     */
    public Toggle(Element element) {
        super(element);

        value = Boolean.parseBoolean(element.getTextContent());
    }

    /**
     * Gets the current stored local value
     * @return - Value of current number
     */
    public synchronized boolean getValue() {
        return value;
    }

    /**
     * Sets the value of the local values but does NOT change network table value or element value
     * @param value - Value to set current number to
     */
    public synchronized void setValue(boolean value) {
        this.value = value;
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry entry = table.getEntry("Value");
        entry.setBoolean(value);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();

        setValue(table.getEntry("Value").getBoolean(getValue()));

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
        Entity.buildStringTabbedData(builder, depth, "Value", String.valueOf(value));
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().setTextContent(String.valueOf(value));

        return getSavedElement();
    }
}
