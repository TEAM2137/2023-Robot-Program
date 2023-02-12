package frc.robot.functions.io.xmlreader.objects;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

public class CANifierObj extends Entity {

    int id;

    public CANifierObj(String _name, int _id) {
        super(_name);
        this.id = _id;
    }

    public CANifierObj(Element element) {
        super(element);

        this.id = Integer.parseInt(getOrDefault(element, "ID", "0"));
    }

    public int getID() { return id; }

    public void setID(int _id) { id = _id;}

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(id));

        return getSavedElement();
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry entryID = table.getEntry("ID");
        entryID.setDouble(id);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();

        setID((int) table.getEntry("ID").getDouble(id));

        return table;
    }
    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = super.removeFromNetworkTable();

        table.getEntry("ID").unpublish();

        return table;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        buildStringTabbedData(builder, depth, "ID", String.valueOf(id));
    }
}
