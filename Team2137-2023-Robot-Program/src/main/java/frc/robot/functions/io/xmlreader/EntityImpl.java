package frc.robot.functions.io.xmlreader;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class EntityImpl implements Entity {

    private NetworkTable savedNetworkTableInstance;
    private org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    public EntityImpl(String _name) {
        name = _name;
    }

    public EntityImpl(org.w3c.dom.Element element) {
        savedElement = element;
        this.name = getNodeOrAttribute(element, "Name", element.getTagName());
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
    }

    @Override
    public void setOnImplementCallback(Runnable run) {
        onImplement = run;
    }

    @Override
    public Runnable getOnImplementCallback() {
        return onImplement;
    }

    @Override
    public org.w3c.dom.Element updateElement() {
        getSavedElement().getElementsByTagName("Name").item(0).setTextContent(name);

        return getSavedElement();
    }

    @Override
    public void OnImplement() {
        onImplement.run();
    }


    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getName());

        NetworkTableEntry entryName = table.getEntry("Name");
        entryName.setString(name);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        name = table.getEntry("Name").getString(getName());

        return table;
    }

    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = getCurrentNetworkInstance();

        table.getEntry("Name").unpublish();

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

}
