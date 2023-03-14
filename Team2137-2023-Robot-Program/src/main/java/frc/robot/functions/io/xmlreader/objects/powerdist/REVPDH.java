package frc.robot.functions.io.xmlreader.objects.powerdist;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class REVPDH extends PowerDistribution implements Entity {

    public enum PowerDistributionFaultType {
        CHANNEL0FAULT  (0),
        CHANNEL1FAULT  (1),
        CHANNEL2FAULT  (2),
        CHANNEL3FAULT  (3),
        CHANNEL4FAULT  (4),
        CHANNEL5FAULT  (5),
        CHANNEL6FAULT  (6),
        CHANNEL7FAULT  (7),
        CHANNEL8FAULT  (8),
        CHANNEL9FAULT  (9),
        CHANNEL10FAULT (10),
        CHANNEL11FAULT (11),
        CHANNEL12FAULT (12),
        CHANNEL13FAULT (13),
        CHANNEL14FAULT (14),
        CHANNEL15FAULT (15),
        CHANNEL16FAULT (16),
        CHANNEL17FAULT (17),
        CHANNEL18FAULT (18),
        CHANNEL19FAULT (19),
        CHANNEL20FAULT (20),
        CHANNEL21FAULT (21),
        CHANNEL22FAULT (22),
        CHANNEL23FAULT (23),
        HARDWARE   (null),
        BROWNOUT   (null),
        CANWARNING (null);

        private Integer slotNumber;

        PowerDistributionFaultType(Integer _slotNumber) {
            slotNumber = _slotNumber;
        }

        public boolean hasSlot() {
            return slotNumber != null;
        }

        public int getSlot() {
            return slotNumber;
        }

        public static List<PowerDistributionFaultType> faults(PowerDistributionFaults fault) {
            List<PowerDistributionFaultType> returner = new ArrayList<>();

            if(fault.Channel0BreakerFault)
                returner.add(CHANNEL0FAULT);
            if(fault.Channel1BreakerFault)
                returner.add(CHANNEL1FAULT);
            if(fault.Channel2BreakerFault)
                returner.add(CHANNEL2FAULT);
            if(fault.Channel3BreakerFault)
                returner.add(CHANNEL3FAULT);
            if(fault.Channel4BreakerFault)
                returner.add(CHANNEL4FAULT);
            if(fault.Channel5BreakerFault)
                returner.add(CHANNEL5FAULT);
            if(fault.Channel6BreakerFault)
                returner.add(CHANNEL6FAULT);
            if(fault.Channel7BreakerFault)
                returner.add(CHANNEL7FAULT);
            if(fault.Channel8BreakerFault)
                returner.add(CHANNEL8FAULT);
            if(fault.Channel9BreakerFault)
                returner.add(CHANNEL9FAULT);
            if(fault.Channel10BreakerFault)
                returner.add(CHANNEL10FAULT);
            if(fault.Channel11BreakerFault)
                returner.add(CHANNEL11FAULT);
            if(fault.Channel12BreakerFault)
                returner.add(CHANNEL12FAULT);
            if(fault.Channel13BreakerFault)
                returner.add(CHANNEL13FAULT);
            if(fault.Channel14BreakerFault)
                returner.add(CHANNEL14FAULT);
            if(fault.Channel15BreakerFault)
                returner.add(CHANNEL15FAULT);
            if(fault.Channel16BreakerFault)
                returner.add(CHANNEL16FAULT);
            if(fault.Channel17BreakerFault)
                returner.add(CHANNEL17FAULT);
            if(fault.Channel18BreakerFault)
                returner.add(CHANNEL18FAULT);
            if(fault.Channel19BreakerFault)
                returner.add(CHANNEL19FAULT);
            if(fault.Channel20BreakerFault)
                returner.add(CHANNEL20FAULT);
            if(fault.Channel21BreakerFault)
                returner.add(CHANNEL21FAULT);
            if(fault.Channel22BreakerFault)
                returner.add(CHANNEL22FAULT);
            if(fault.Channel23BreakerFault)
                returner.add(CHANNEL23FAULT);
            if(fault.Brownout)
                returner.add(BROWNOUT);
            if(fault.CanWarning)
                returner.add(CANWARNING);

            return returner;
        }
    }

    private NetworkTable savedNetworkTableInstance;
    private org.w3c.dom.Element savedElement;
    private String name;
    private Runnable onImplement;

    private FileLogger fileLogger;

    private String[] slotNames;

    private final int channelCount;

    public REVPDH(Element element) {
        super();

        savedElement = element;
        this.name = getNodeOrAttribute(element, "Name", element.getTagName());

        fileLogger = Robot.fileLogger;

        channelCount = super.getNumChannels();
        slotNames = new String[channelCount];

        NodeList nodeList = element.getElementsByTagName("Slot");
        for(int idx = 0; idx < nodeList.getLength(); idx++) {
            Element childNode = (Element) nodeList.item(idx);

            String slotNumber = getNodeOrAttribute(childNode, "SlotNumber", null);
            if(slotNumber != null) {
                slotNames[Integer.parseInt(slotNumber)] = getNodeOrAttribute(childNode, "Name", "Default");
            }
        }

        String notifyPeriod = getNodeOrAttribute(element, "Period", null);

        if(notifyPeriod != null) {
            Robot.threadPoolExecutor.scheduleAtFixedRate(this::NotifyFaults, 0, Integer.parseInt(notifyPeriod), TimeUnit.MILLISECONDS);
        }
    }

    public void NotifyFaults() {
        List<PowerDistributionFaultType> faults = PowerDistributionFaultType.faults(super.getFaults());

        for(PowerDistributionFaultType fault : faults) {
            String message;

            if(fault.hasSlot()) {
                message = "PDH Fault on Slot " + fault.getSlot() + (slotNames[fault.getSlot()] != null ? " aka " + slotNames[fault.getSlot()] : "");
            } else {
                message = "PDH Fault ";

                switch(fault) {
                    case BROWNOUT:
                        message += "Brownout";
                        break;
                    case HARDWARE:
                        message += "Hardware";
                        break;
                    case CANWARNING:
                        message += "CAN Warning";
                        break;
                }
            }

            DriverStation.reportWarning(message, false);
            fileLogger.writeEvent(1, FileLogger.EventType.Warning, message);
        }
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
