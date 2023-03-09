package frc.robot.functions.io.xmlreader.objects.powerdist;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

public class REVPDH extends PowerDistribution implements Entity {
    private NetworkTable savedNetworkTableInstance;
    private org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    private final int channelCount;

    public REVPDH(Element element) {
        super();

        savedElement = element;
        this.name = getNodeOrAttribute(element, "Name", element.getTagName());

        channelCount = super.getNumChannels();


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

    @Override
    public Element getSavedElement() {
        return savedElement;
    }
}
