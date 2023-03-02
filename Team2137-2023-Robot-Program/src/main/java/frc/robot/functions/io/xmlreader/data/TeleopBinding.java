package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.library.Constants;
import org.w3c.dom.Element;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

import static edu.wpi.first.wpilibj.DriverStation.isTeleop;

public class TeleopBinding extends EntityGroup {

    private final ArrayList<Step> steps = new ArrayList<>();

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public TeleopBinding(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        for(Entity entry : getEntities()) {
            if(entry.getName().equalsIgnoreCase("Step")) {
                Step step = (Step) entry;

                steps.add(step);
            }
        }

        Robot.persistenceTeleopSteps.addAll(steps);
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        return table;
    }
}
