package frc.robot.functions.io.xmlreader.data.mappings;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.library.Gamepad;
import org.w3c.dom.Element;

import frc.robot.library.Gamepad.Axis;
import frc.robot.library.Gamepad.Button;
import frc.robot.library.Gamepad;

import java.sql.Driver;
import java.util.ArrayList;

public class ControllerMapping extends EntityImpl implements Mapping {

    public enum DPADValues {
        DPAD_UP ("DPAD-UP", 0),
        DPAD_DOWN ("DPAD-DOWN", 2),
        DPAD_LEFT ("DPAD-LEFT", 3),
        DPAD_RIGHT ("DPAD-RIGHT", 1);

        final String val;
        final int dir;

        DPADValues(String value, int pos) {
            val = value;
            dir = pos;
        }

        @Override
        public String toString() {
            return val;
        }

        public int getDirection() {
            return dir;
        }
    }

    private final Gamepad[] controllers;
    private final int controllerNumber;
    private final double deadband;
    private final String valueName;
    private Axis controllerAxis;
    private Button controllerButton;
    private DPADValues controllerDPAD;

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     *
     * @param element
     */
    public ControllerMapping(Element element) {
        super(element);

        controllers = new Gamepad[2];
        controllers[0] = Robot.primaryController;
        controllers[1] = Robot.secondaryController;

        controllerNumber = Integer.parseInt(getNodeOrAttribute(element, "controller", "0"));
        String id = getNodeOrAttribute(element, "id", "X");
        valueName = getNodeOrAttribute(element, "value", "value");
        deadband = Double.parseDouble(getNodeOrAttribute(element, "deadband", "0"));

        for (Axis axis : Axis.values()) {
            if(axis.toString().equalsIgnoreCase(id)) {
                controllerAxis = axis;
            }
        }

        for (Button button : Button.values()) {
            if(button.toString().equalsIgnoreCase(id)) {
                controllerButton = button;
            }
        }

        for (DPADValues dpad : DPADValues.values()) {
            if(dpad.toString().equalsIgnoreCase(id)) {
                controllerDPAD = dpad;
            }
        }
    }

    @Override
    public double getValue() {
        if(controllerAxis != null) {
            double value = controllers[controllerNumber].getRawAxis(controllerAxis.value);

            if(controllerAxis == Axis.kLeftY || controllerAxis == Axis.kRightY)
                value *= -1;

            if(Math.abs(value) > deadband)
                return value;
            else
                return 0;

        } else if (controllerButton != null) {
            return controllers[controllerNumber].getRawButton(controllerButton.value) ? 1 : 0;
        } else if (controllerDPAD != null) {
            return controllers[controllerNumber].getPOV();
        }else {
            return 0;
        }
    }

    @Override
    public boolean getBooleanValue() {
        if(controllerAxis != null) {
            return controllers[controllerNumber].getAxisType(controllerAxis.value) == 1;
        } else if (controllerButton != null) {
            return controllers[controllerNumber].getRawButtonPressed(controllerButton.value);
        } else if (controllerDPAD != null) {
            return getDPADValues(controllers[controllerNumber].getPOV())[controllerDPAD.getDirection()];
        } else {
            return false;
        }
    }

    @Override
    public String getPseudoName() {
        return valueName;
    }

    @Override
    public Boolean isBooleanValue() {
        return controllerButton != null || controllerDPAD != null;
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

    //Up, Right, Down, Left
    public boolean[] getDPADValues(double angle) {
        if(angle >= 0 && angle <= 22.5)
            return new boolean[] { true,    false,  false,  false };
        else if(angle > 22.5 && angle <= 67.5)
            return new boolean[] { true,    true,   false,  false };
        else if(angle > 67.5 && angle <= 112.5)
            return new boolean[] { false,   true,   false,  false };
        else if(angle > 112.5 && angle <= 157.5)
            return new boolean[] { false,   true,   true,   false };
        else if(angle > 157.5 && angle <= 202.5)
            return new boolean[] { false,   false,  true,   false };
        else if(angle > 202.5 && angle <= 247.5)
            return new boolean[] { false,   false,  true,   true  };
        else if(angle > 247.5 && angle <= 292.5)
            return new boolean[] { false,   false,  false,  true  };
        else if(angle > 292.5 && angle <= 337.5)
            return new boolean[] { true,    false,  false,  true  };
        else if(angle > 337.5 && angle <= 360)
            return new boolean[] { true,    false,  false,  false };
        else
            return new boolean[] { false,   false,  false,  false };
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