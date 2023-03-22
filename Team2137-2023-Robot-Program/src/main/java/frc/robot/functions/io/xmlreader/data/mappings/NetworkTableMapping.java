package frc.robot.functions.io.xmlreader.data.mappings;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.objects.canifier.DIO;
import org.w3c.dom.Element;

public class NetworkTableMapping extends EntityImpl implements Mapping {

    private final boolean inverted;
    private final String valueName;
    private NetworkTableEntry entry;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public NetworkTableMapping(Element element) {
        super(element);

        String id = getNodeOrAttribute(element, "id", "limelight-rtape/tx");
        valueName = getNodeOrAttribute(element, "value", "value");
        inverted = Boolean.parseBoolean(getNodeOrAttribute(element, "inverted", "false"));

        entry = NetworkTableInstance.getDefault().getEntry(id);
    }

    @Override
    public double getValue() {
        return entry.getDouble(0);
    }

    @Override
    public boolean getBooleanValue() {
        return entry.getBoolean(false);
    }

    @Override
    public Boolean isBooleanValue() {
        return entry.getType() == NetworkTableType.kBoolean;
    }

    @Override
    public String getPseudoName() {
        return valueName;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        Entity.buildStringTabbedData(builder, depth, "EntryName", entry.getName());
        Entity.buildStringTabbedData(builder, depth, "ValueName", valueName);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getPseudoName());

        NetworkTableEntry entryName = table.getEntry("EntryName");
        entryName.setString(entry.getName());

        NetworkTableEntry entryInverted = table.getEntry("ValueName");
        entryInverted.setString(valueName);

        return table;
    }
}
