package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.library.Constants;
import org.w3c.dom.Element;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class Binding extends EntityGroup {

    private HashMap<String, ControllerMapping> mappings = new HashMap<String, ControllerMapping>();
    private ArrayList<ControllerMapping> buttons = new ArrayList<>();
    private ArrayList<Step> steps = new ArrayList<>();

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public Binding(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        for(Entity entry : getEntities()) {
            if(entry.getName().equalsIgnoreCase("Map")) {
                ControllerMapping map = (ControllerMapping) entry;
                mappings.put(map.getValueName(), map);
            }
        }

        for(Entity entry : getEntities()) {
            if(entry.getName().equalsIgnoreCase("Step")) {
                Step step = (Step) entry;
                step.registerMappings(mappings);

                steps.add(step);
            }
        }

        boolean flag = true;
        for(ControllerMapping entry : mappings.values()) {
            if(entry.isButton()) {
                buttons.add(entry);
            } else if(flag) {
                Robot.currentActiveSteps.addAll(steps);
                flag = false;
            }
        }
    }

    @Override
    public void periodic() {
        for (ControllerMapping button : buttons) {
            if(button.getButtonValue()) {
                for (Step step : steps)
                    step.changeStepState(Constants.StepState.STATE_INIT);
                Robot.currentActiveSteps.addAll(steps);
            }
        }

//        if(isAutonomous() || isTeleop() || isTest()) {
//            Robot.currentActiveSteps.addAll(steps);
//        }
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        for(ControllerMapping mapping : mappings.values()) {
            mapping.constructTreeItemPrintout(builder, depth + 1);
        }
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        for(ControllerMapping mapping : mappings.values()) {
            mapping.addToNetworkTable(table);
        }

        return table;
    }
}
