//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package frc.robot.functions.io.xmlreader.objects;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class Motor extends Entity {

    public static final int NUMBEROFPIDSLOTS = 2;

    private MotorTypes type = MotorTypes.NEO;
    private boolean inverted = false;
    private int currentLimit = 0;
    private double gearRatio = 0;
    private double rampRate = 0;
    private PID[] pidValues = new PID[NUMBEROFPIDSLOTS];
    private int id = 0;

    /**
     * Create a new Motor Object for debug and better storage
     *
     * @param _name
     * @param _id
     * @param _type
     * @param _invert
     * @param _currentLimit
     * @param _gearRatio
     */
    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _ramp) {
        super(_name);
        this.type = _type;
        this.inverted = _invert;
        this.currentLimit = _currentLimit;
        this.gearRatio = _gearRatio;
        this.rampRate = _ramp;
        this.id = _id;
    }

    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _ramp, PID[] pid) {
        this(_name, _id, _type, _invert, _currentLimit, _gearRatio, _ramp);
        this.pidValues = pid;
    }

    public Motor(Element element) {
        super(element);
        this.type = MotorTypes.valueOf(getOrDefault(element, "Type", "NEO").toUpperCase());
        this.inverted = Boolean.parseBoolean(getOrDefault(element, "Inverted", "false").toLowerCase());
        this.currentLimit = Integer.parseInt(getOrDefault(element, "CurrentLimit", "-1"));
        this.gearRatio = Double.parseDouble(getOrDefault(element, "GearRatio", "1"));
        this.rampRate = Double.parseDouble(getOrDefault(element, "RampRate", "0"));
        this.id = Integer.parseInt(getOrDefault(element, "ID", "0"));

        NodeList tmpList = element.getElementsByTagName("PID");

        for (int i = 0; i < tmpList.getLength(); i++) {
            String value = ((Element) tmpList.item(i)).getAttribute("Slot");
            int slotNumber = 0;

            if (!value.equals(""))
                slotNumber = Integer.parseInt(value);
            //TODO add multiplatform debug (DriverStation doesnt work on desktop)
//            else
//                DriverStation.reportError("Null PID Slot Given", false);

            if (slotNumber >= NUMBEROFPIDSLOTS) {
//                throw new PIDSlotIDOutOfRangeException(getName());
                DriverStation.reportError("PID Slot too large", false);
            } else {
                this.pidValues[slotNumber] = new PID((Element) tmpList.item(i));
            }
        }
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public void setGearRatio(double ratio) { this.gearRatio = ratio; }

    public MotorTypes getMotorType() {
        return this.type;
    }

    public void setMotorType(MotorTypes _type) { this.type = _type; }

    public boolean inverted() {
        return this.inverted;
    }

    public void setInverted(boolean value) { this.inverted = value; }

    public int getCurrentLimit() {
        return this.currentLimit;
    }

    public void setCurrentLimit(int limit) { this.currentLimit = limit; }

    public PID getPID(int slotID) {
        return this.pidValues[slotID];
    }

    public void setPID(int soltID, PID pid) {
        this.pidValues[soltID] = pid;
    }

    public PID getPID() {
        return this.pidValues[0];
    }

    public void setPID(PID pid) {
        this.pidValues[0] = pid;
    }

    public double getRampRate() {
        return rampRate;
    }

    public void setRampRate(double _rampRate) { this.rampRate = _rampRate; }

    public int getID() { return id; }

    public void setID(int _id) { id = _id; }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        buildStringTabbedData(builder, depth, "ID", String.valueOf(id));
        buildStringTabbedData(builder, depth, "Type", type.toString());
        buildStringTabbedData(builder, depth, "Inverted", String.valueOf(inverted));
        buildStringTabbedData(builder, depth, "Current Limit", String.valueOf(currentLimit));
        buildStringTabbedData(builder, depth, "Gear Ratio", String.valueOf(gearRatio));
        buildStringTabbedData(builder, depth, "Ramp Rate", String.valueOf(rampRate));

        for(int i = 0; i < NUMBEROFPIDSLOTS; i++) {
            if(pidValues[i] != null)
                pidValues[i].constructTreeItemPrintout(builder, depth + 1);
        }
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(id));
        getSavedElement().getElementsByTagName("GearRatio").item(0).setTextContent(String.valueOf(gearRatio));
        getSavedElement().getElementsByTagName("Type").item(0).setTextContent(type.toString());
        getSavedElement().getElementsByTagName("Inverted").item(0).setTextContent(String.valueOf(inverted));
        getSavedElement().getElementsByTagName("CurrentLimit").item(0).setTextContent(String.valueOf(currentLimit));
        getSavedElement().getElementsByTagName("RampRate").item(0).setTextContent(String.valueOf(rampRate));

        //TODO readdPID
//        for (PID pid : pidValues) {
//            if(pid != null) pid.updateElement();
//        }

        return getSavedElement();
    }


    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        //TODO readdPID
//        for (PID pid : pidValues) {
//            if(pid != null) pid.addToNetworkTable(dashboard);
//        }

        NetworkTableEntry entryGearRatio = table.getEntry("GearRatio");
        entryGearRatio.setDouble(gearRatio);

        NetworkTableEntry entryRampRate = table.getEntry("RampRate");
        entryRampRate.setDouble(rampRate);

        NetworkTableEntry entryCurrentLimit = table.getEntry("CurrentLimit");
        entryCurrentLimit.setDouble(currentLimit);

        NetworkTableEntry entryInverted = table.getEntry("Inverted");
        entryInverted.setBoolean(inverted);

        //None mutable
        NetworkTableEntry entryType = table.getEntry("Type");
        entryType.setString(type.toString());

        NetworkTableEntry entryID = table.getEntry("ID");
        entryID.setDouble(id);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();

        //TODO add type
        setInverted(table.getEntry("Inverted").getBoolean(inverted()));
        setID((int) table.getEntry("ID").getDouble(id));
        setCurrentLimit((int) table.getEntry("CurrentLimit").getDouble(getCurrentLimit())); //TODO convert current limit to double
        setRampRate(table.getEntry("RampRate").getDouble(getRampRate()));
        setGearRatio(table.getEntry("GearRatio").getDouble(getGearRatio()));

        return table;
    }
    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = super.removeFromNetworkTable();

        table.getEntry("Inverted").unpublish();
        table.getEntry("ID").unpublish();
        table.getEntry("CurrentLimit").unpublish();
        table.getEntry("RampRate").unpublish();
        table.getEntry("GearRatio").unpublish();
        table.getEntry("Type").unpublish();

        return table;
    }

    public enum MotorClass {
        Brushless,
        Brushed
    }

    public enum MotorTypes {
        NEO(MotorClass.Brushless, "NEO"),
        NEO550(MotorClass.Brushless, "NEO550"),
        FALCON(MotorClass.Brushless, "FALCON"),
        BAG(MotorClass.Brushed, "BAG"),
        CIM(MotorClass.Brushed, "CIM"),
        WINDOW(MotorClass.Brushed, "WINDOW");

        MotorClass type;
        String value;

        MotorTypes(MotorClass revType, String name) {
            type = revType;
            value = name;
        }

        public MotorClass getMotorClass() {
            return this.type;
        }

        public CANSparkMaxLowLevel.MotorType getREVType() {
            return (getMotorClass() == Motor.MotorClass.Brushless ? CANSparkMaxLowLevel.MotorType.kBrushless : CANSparkMaxLowLevel.MotorType.kBrushed);
        }

        @Override
        public String toString() {
            return value;
        }
    }

    public enum ControllerType {
        SPARK("Spark"),
        TALONFX("TalonFX"),
        TALON("Talon");

        String value;

        ControllerType(String name) {
            value = name;
        }

        public static ControllerType getControllerFromMotor(MotorTypes type) {
            switch (type) {
                case NEO:
                case NEO550:
                    return SPARK;
                case FALCON:
                    return TALONFX;
                default:
                    return TALON;
            }
        }

        @Override
        public String toString() {
            return value;
        }
    }
}