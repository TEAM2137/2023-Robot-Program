package frc.robot.functions.io.xmlreader.data.mappings;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import org.w3c.dom.Element;

import java.util.ArrayList;

public class ControllerMapping extends EntityImpl implements Mapping {

    private XboxController[] controllers;
    private final int controllerNumber;
    private String valueName;
    private XboxController.Axis controllerAxis;
    private XboxController.Button controllerButton;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public ControllerMapping(Element element) {
        super(element);

        controllers = new XboxController[2];
        controllers[0] = Robot.primaryController;
        controllers[1] = Robot.secondaryController;

        controllerNumber = Integer.parseInt(getNodeOrAttribute(element, "controller", "0"));
        String id = getNodeOrAttribute(element, "id", "X");
        valueName = getNodeOrAttribute(element, "value", "value");

        for (XboxController.Axis axis : XboxController.Axis.values()) {
            if(axis.toString().contains(id)) {
                controllerAxis = axis;
            }
        }

        for (XboxController.Button button : XboxController.Button.values()) {
            if(button.toString().equalsIgnoreCase(id)) {
                controllerButton = button;
            }
        }
    }

    @Override
    public double getValue() {
        if(controllerAxis != null) {
            return controllers[controllerNumber].getRawAxis(controllerAxis.value);
        } else if (controllerButton != null) {
            return controllers[controllerNumber].getRawButton(controllerButton.value) ? 1 : 0;
        } else {
            return 0;
        }
    }

    @Override
    public boolean getBooleanValue() {
        if(controllerAxis != null) {
            return controllers[controllerNumber].getAxisType(controllerAxis.value) == 1;
        } else if (controllerButton != null) {
            return controllers[controllerNumber].getRawButtonPressed(controllerButton.value);
        } else {
            return false;
        }
    }

    @Override
    public String getPseudoName() {
        return valueName;
    }

    @Override
    public boolean isBooleanValue() {
        return controllerButton != null;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        Entity.buildStringTabbedData(builder, depth, "Controller", String.valueOf(controllerNumber));
        if(controllerAxis != null)
            Entity.buildStringTabbedData(builder, depth, "Axis", controllerAxis.toString());
        if(controllerButton != null)
            Entity.buildStringTabbedData(builder, depth, "Button", controllerButton.toString());

        Entity.buildStringTabbedData(builder, depth, "ValueName", valueName);
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = dashboard.getSubTable(getPseudoName());

        NetworkTableEntry entryGearRatio = table.getEntry("Controller");
        entryGearRatio.setInteger(controllerNumber);

        if(controllerAxis != null) {
            NetworkTableEntry entryRampRate = table.getEntry("Axis");
            entryRampRate.setString(controllerAxis.toString());
        }

        if(controllerButton != null) {
            NetworkTableEntry entryCurrentLimit = table.getEntry("Button");
            entryCurrentLimit.setString(controllerButton.toString());
        }

        NetworkTableEntry entryInverted = table.getEntry("ValueName");
        entryInverted.setString(valueName);

        return table;
    }
}