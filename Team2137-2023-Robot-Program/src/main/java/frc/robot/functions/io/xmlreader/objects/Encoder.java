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

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.EntityImpl;
import org.w3c.dom.Element;

public class Encoder extends EntityImpl {

    public enum EncoderTypes {
        CTRE_CAN_ABS ("CTRE_CAN_ABS", 2048, null),
        REV_INTEGRATED ("REV_INTEGRATED", 2048, SparkMaxRelativeEncoder.Type.kHallSensor),
        REV_BORE_ABS ("REV_BORE_ABS", 4096, null),
        REV_BORE_REL ("REV_BORE_REL", 4096, SparkMaxRelativeEncoder.Type.kHallSensor);

        final String name;
        final int cpr;
        final SparkMaxRelativeEncoder.Type type;

        EncoderTypes (String value, int _cpr, SparkMaxRelativeEncoder.Type _type) {
            name = value;
            cpr = _cpr;
            type = _type;
        }

        @Override
        public String toString() {
            return name;
        }

        public int getCPR() {
            return cpr;
        }

        public SparkMaxRelativeEncoder.Type getType() {
            return type;
        }
    }

    EncoderTypes type;
    String canLoopName;
    boolean inverted;
    boolean absolute;
    double offset;
    int cpr;
    int id;

    public Encoder(String _name, int _id, EncoderTypes _type, boolean _invert, double offset, boolean _absoluteMode) {
        super(_name);
        this.id = _id;
        this.type = _type;
        this.inverted = _invert;
        this.offset = offset;
        this.canLoopName = "rio";
        this.cpr = _type.getCPR();
        this.absolute = _absoluteMode;
    }

    public Encoder(Element element) {
        super(element);

        this.id = Integer.parseInt(Entity.getOrDefault(element, "ID", "0"));
        this.type = EncoderTypes.valueOf(Entity.getOrDefault(element, "Type", "CTRE_CAN_ABS").toUpperCase());
        this.inverted = Boolean.parseBoolean(Entity.getOrDefault(element, "Inverted", "false").toLowerCase());
        this.offset = Double.parseDouble(Entity.getOrDefault(element, "Offset", "0"));
        this.canLoopName = Entity.getOrDefault(element, "CANLoop", "rio");
        this.cpr = Integer.parseInt(Entity.getOrDefault(element, "CPR", String.valueOf(type.cpr)));
        this.absolute = Boolean.parseBoolean(Entity.getOrDefault(element, "Absolute", "false"));
    }

    public String getCANLoopName() {
        return canLoopName;
    }

    public EncoderTypes getEncoderType() {
        return this.type;
    }

    public void setEncoderTypes(EncoderTypes _type) { type = _type; }

    public boolean inverted() {
        return this.inverted;
    }

    public void setInverted(boolean val) { inverted = val; }

    public double getOffset() {
        return offset;
    }

    public void setOffset(double _offset) { offset = _offset; }

    public int getID() { return id; }

    public void setID(int _id) { id = _id;}

    public void setCPR(int _cpr) {
        cpr = _cpr;
    }

    public int getCPR() {
        return cpr;
    }

    public boolean isAbsolute() {
        return absolute;
    }

    public void setAbsolute(boolean ab) {
        absolute = ab;
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().getElementsByTagName("Type").item(0).setTextContent(String.valueOf(type.name));
        getSavedElement().getElementsByTagName("Inverted").item(0).setTextContent(String.valueOf(inverted));
        getSavedElement().getElementsByTagName("Offset").item(0).setTextContent(String.valueOf(offset));
        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(id));
        getSavedElement().getElementsByTagName("CPR").item(0).setTextContent(String.valueOf(cpr));

        return getSavedElement();
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry entryInverted = table.getEntry("Inverted");
        entryInverted.setBoolean(inverted);

        //None mutable
        NetworkTableEntry entryType = table.getEntry("Type");
        entryType.setString(type.toString());

        NetworkTableEntry entryID = table.getEntry("ID");
        entryID.setDouble(id);

        NetworkTableEntry entryOffset = table.getEntry("Offset");
        entryOffset.setDouble(offset);

        NetworkTableEntry entryCPR = table.getEntry("CPR");
        entryCPR.setInteger(cpr);

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();


        //TODO add type
        setInverted(table.getEntry("Inverted").getBoolean(inverted()));
        setID((int) table.getEntry("ID").getDouble(id));
        //setOffset(table.getEntry("Offset").getDouble(getOffset()));
        setOffset(table.getEntry("Offset").getDouble(offset));
        setCPR((int) table.getEntry("CPR").getInteger(cpr));

        return table;
    }
    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = super.removeFromNetworkTable();

        table.getEntry("Inverted").unpublish();
        table.getEntry("ID").unpublish();
        table.getEntry("Offset").unpublish();
        table.getEntry("Type").unpublish();
        table.getEntry("CPR").unpublish();

        return table;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        Entity.buildStringTabbedData(builder, depth, "ID", String.valueOf(id));
        Entity.buildStringTabbedData(builder, depth, "Type", type.toString());
        Entity.buildStringTabbedData(builder, depth, "Inverted", String.valueOf(inverted));
        Entity.buildStringTabbedData(builder, depth, "Offset", String.valueOf(offset));
        Entity.buildStringTabbedData(builder, depth, "CPR", String.valueOf(cpr));
    }
}