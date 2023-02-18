package frc.robot.functions.io.xmlreader.objects.solenoid;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

public class DoubleSolenoid extends edu.wpi.first.wpilibj.DoubleSolenoid implements Entity, SimpleSolenoid {
    private NetworkTable savedNetworkTableInstance;
    private org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    public DoubleSolenoid(org.w3c.dom.Element element) {
        super(PneumaticsModuleType.REVPH, Integer.parseInt(Entity.getOrDefault(element, "IDF", element.getTagName())), Integer.parseInt(Entity.getOrDefault(element, "IDR", element.getTagName())));

        savedElement = element;
        this.name = getNodeOrAttribute(element, "Name", element.getTagName());
    }

    @Override
    public void set(edu.wpi.first.wpilibj.DoubleSolenoid.Value value) {
        super.set(value);
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
        Entity.buildStringTabbedData(builder, depth, "IDF", String.valueOf(this.getFwdChannel()));
        Entity.buildStringTabbedData(builder, depth, "IDR", String.valueOf(this.getRevChannel()));
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
        getSavedElement().getElementsByTagName("IDF").item(0).setTextContent(String.valueOf(getFwdChannel()));
        getSavedElement().getElementsByTagName("IDR").item(0).setTextContent(String.valueOf(getRevChannel()));

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

        NetworkTableEntry fID = table.getEntry("IDF");
        fID.setInteger(getFwdChannel());

        NetworkTableEntry rID = table.getEntry("IDR");
        rID.setInteger(getRevChannel());

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
        table.getEntry("IDF").unpublish();
        table.getEntry("IDR").unpublish();

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
