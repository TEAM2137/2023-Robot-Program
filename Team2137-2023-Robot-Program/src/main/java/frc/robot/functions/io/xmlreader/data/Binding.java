package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.mappings.ControllerMapping;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.library.Constants;
import org.w3c.dom.Element;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.wpilibj.DriverStation.isTeleop;
import static edu.wpi.first.wpilibj.DriverStation.isTest;

public class Binding extends EntityGroup {

    private final HashMap<String, Mapping> mappings = new HashMap<String, Mapping>();
    private final ArrayList<Mapping> booleanEntries = new ArrayList<>();
    private final ArrayList<Mapping> numericEntries = new ArrayList<>();
    private final ArrayList<Step> steps = new ArrayList<>();

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
                Mapping map = (Mapping) entry;
                mappings.put(map.getPseudoName(), map);
            }
        }

        for(Entity entry : getEntities()) {
            if(entry.getName().equalsIgnoreCase("Step")) {
                Step step = (Step) entry;
                step.registerMappings(mappings);

                steps.add(step);
            }
        }

        for(Mapping entry : mappings.values()) {
            if(entry.isBooleanValue()) {
                booleanEntries.add(entry);
            } else {
                numericEntries.add(entry);
                break;
            }
        }

        if(numericEntries.size() > 0) {
            for (Step step : steps)
                step.changeStepState(Constants.StepState.STATE_INIT);

            Robot.currentActiveSteps.addAll(steps);
        }
    }

    @Override
    public void periodic() {
            for (Mapping button : booleanEntries) {
//                SmartDashboard.putBoolean(getName() + "-" + button.getPseudoName(), button.getBooleanValue());

                if (button.getBooleanValue()) {
                    if(button instanceof ControllerMapping && (!isTeleop() || !isTest()))
                        continue;

                    for (Step step : steps) {
                        step.changeStepState(Constants.StepState.STATE_INIT);

                        Robot.currentActiveSteps.add(step);
                    }
                }
            }
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        for(Mapping mapping : mappings.values()) {
            mapping.constructTreeItemPrintout(builder, depth + 1);
        }
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        for(Mapping mapping : mappings.values()) {
            mapping.addToNetworkTable(table);
        }

        return table;
    }
}
