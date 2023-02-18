package frc.robot.functions.io.xmlreader.data.mappings;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.objects.canifier.DIO;
import org.w3c.dom.Element;

import java.util.ArrayList;

public class DIOMapping extends EntityImpl implements Mapping {

    private DIO digitalInput;
    private final String valueName;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public DIOMapping(Element element) {
        super(element);

        String id = getNodeOrAttribute(element, "id", "X");
        valueName = getNodeOrAttribute(element, "value", "value");

        for(Entity a : Robot.allEntities) {
            if(a instanceof DIO && a.getName().equalsIgnoreCase(id)) {
                digitalInput = (DIO) a;
            }
        }

        System.out.println("DIO not found");
    }

    @Override
    public double getValue() {
        return digitalInput.get() ? 1 : 0;
    }

    @Override
    public boolean getBooleanValue() {
        return digitalInput.get();
    }

    @Override
    public String getPseudoName() {
        return valueName;
    }

    @Override
    public boolean isBooleanValue() {
        return true;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        Entity.buildStringTabbedData(builder, depth, "ValueName", valueName);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getPseudoName());

        NetworkTableEntry entryInverted = table.getEntry("ValueName");
        entryInverted.setString(valueName);

        return table;
    }
}
