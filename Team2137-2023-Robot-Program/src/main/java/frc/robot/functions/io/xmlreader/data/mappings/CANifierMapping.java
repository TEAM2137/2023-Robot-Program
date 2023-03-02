package frc.robot.functions.io.xmlreader.data.mappings;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.objects.canifier.CANifierObj;
import org.w3c.dom.Element;

public class CANifierMapping extends EntityImpl implements Mapping {

    private CANifierObj canifier;
    private final CANifier.GeneralPin pin;
    private final String mCANifierName;
    private final String value;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public CANifierMapping(Element element) {
        super(element);

        mCANifierName = getNodeOrAttribute(element, "CANifierName", "none");
        pin = CANifier.GeneralPin.valueOf(getNodeOrAttribute(element, "id", "LIMF"));
        value = getNodeOrAttribute(element, "value", "none");

        for(Entity entity : Robot.allEntities) {
            if(entity instanceof CANifierObj && entity.getName().equalsIgnoreCase(mCANifierName)){
                canifier = (CANifierObj) entity;
                break;
            }
        }
    }

    @Override
    public double getValue() {
        return canifier.getGeneralInput(pin) ? 1.0 : 0.0;
    }

    @Override
    public boolean getBooleanValue() {
        return canifier.getGeneralInput(pin);
    }

    @Override
    public Boolean isBooleanValue() {
        return true;
    }

    @Override
    public String getPseudoName() {
        return value;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        Entity.buildStringTabbedData(builder, depth, "CANifierName", mCANifierName);
        Entity.buildStringTabbedData(builder, depth, "Pin", pin.toString());

        Entity.buildStringTabbedData(builder, depth, "ValueName", value);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getPseudoName());

        NetworkTableEntry caniferNameEntry = table.getEntry("CANifierName");
        caniferNameEntry.setString(mCANifierName);

        NetworkTableEntry pinValueEntry = table.getEntry("Pin");
        pinValueEntry.setString(pin.toString());

        NetworkTableEntry entryInverted = table.getEntry("ValueName");
        entryInverted.setString(value);

        return table;
    }
}
