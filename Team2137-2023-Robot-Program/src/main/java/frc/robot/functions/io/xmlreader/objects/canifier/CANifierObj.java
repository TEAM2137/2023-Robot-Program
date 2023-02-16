package frc.robot.functions.io.xmlreader.objects.canifier;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import org.w3c.dom.Element;

public class CANifierObj extends CANifier implements Entity {

    private NetworkTable savedNetworkTableInstance;
    private org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    int id;

    public CANifierObj(Element element) {
        super(Integer.parseInt(Entity.getOrDefault(element, "ID", "0")));

        savedElement = element;

        this.id = super.getDeviceID();
    }

    public int getID() { return id; }

    public void setID(int _id) { id = _id;}

    @Override
    public Element updateElement() {
        getSavedElement().getElementsByTagName("Name").item(0).setTextContent(name);
        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(id));

        return getSavedElement();
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = getCurrentNetworkInstance();

        NetworkTableEntry entryID = table.getEntry("ID");
        entryID.setDouble(id);

        NetworkTableEntry entryName = table.getEntry("Name");
        entryName.setString(name);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        name = table.getEntry("Name").getString(getName());
        setID((int) table.getEntry("ID").getDouble(id));

        return table;
    }
    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        table.getEntry("Name").unpublish();
        table.getEntry("ID").unpublish();

        return table;
    }

    @Override
    public NetworkTable getCurrentNetworkInstance() {
        return savedNetworkTableInstance;
    }

    @Override
    public void setCurrentNetworkInstance(NetworkTable instance) {
        savedNetworkTableInstance = instance;
    }

    /**
     * Returns the linked Element object
     *
     * @return - Linked Element object
     */
    @Override
    public Element getSavedElement() {
        return savedElement;
    }

    /**
     * Gets the set name of the Entity
     *
     * @return - If no name is present return "Default"
     */
    @Override
    public String getName() {
        return name;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        Entity.buildStringTabbedData(builder, depth, "Name", name);
        Entity.buildStringTabbedData(builder, depth, "ID", String.valueOf(id));
    }

    @Override
    public void setOnImplementCallback(Runnable run) {
        onImplement = run;
    }

    @Override
    public Runnable getOnImplementCallback() {
        return onImplement;
    }
}
