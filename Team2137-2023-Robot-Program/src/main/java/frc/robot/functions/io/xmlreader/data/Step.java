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

package frc.robot.functions.io.xmlreader.data;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.library.Constants;
import frc.robot.library.units.Time;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.script.ScriptException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import static frc.robot.library.units.Time.TimeUnits.MILLISECONDS;
import static frc.robot.library.units.Time.TimeUnits.SECONDS;


public class Step extends EntityImpl {
    public enum StepValues {
        COMMAND ("COMMAND"),
        TIMEOUT ("TIMEOUT"),
        SPEED ("SPEED"),
        //DISTANCE ("DISTANCE"),
        XDISTANCE ("XDISTANCE"),
        YDISTANCE ("YDISTANCE"),
        PARALLEL ("PARALLEL");

        String name = "";

        StepValues(String str) {
            this.name = str;
        }

        public String toString() { return name; }
    }

    private class ValueEntry {
        public String value;
        public boolean genericType;
        public Mapping mappings;

        public ValueEntry(String _value) {
            value = _value;
            genericType = false;
        }

        public ValueEntry(String _value, boolean _genericType) {
            value = _value;
            genericType = _genericType;
        }
    }

    private final HashMap<String, ValueEntry> values = new HashMap<String, ValueEntry>();
    private Constants.StepState mCurrentStepState = Constants.StepState.STATE_INIT;
    private Constants.StepState mDriveSpecificState = null;

    private Double timeout;
    private Long startTime;

    public Step(Element element) {
        super(element);
        NodeList list = element.getChildNodes();
        for (int i = 0; i < list.getLength(); i++) {
            Element tmp;
            try {
                tmp = (Element) list.item(i); //TODO tmp solution
            } catch(Exception e) {
                continue;
            }

            Node genericElement = tmp.getFirstChild();

            if (genericElement.getNodeType() == Node.TEXT_NODE) {
                this.values.put(tmp.getNodeName().toUpperCase(), new ValueEntry(genericElement.getTextContent()));
            } else if (genericElement.getNodeType() == Node.ELEMENT_NODE){
                this.values.put(tmp.getNodeName().toUpperCase(), new ValueEntry(genericElement.getNodeName(), true));
//                this.values.put(tmp.getNodeName().toUpperCase(), new ValueEntry(genericElement.getTextContent(), true));
            }
        }

        if(this.values.containsKey(StepValues.TIMEOUT.toString()))
            timeout = Double.parseDouble(this.values.get(StepValues.TIMEOUT.toString()).value);
        else
            timeout = null;
    }

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _XDISTANCE, String _YDISTANCE, String _PARALLEL, String... _PARMs) {
        super("Step");
        this.values.put(StepValues.COMMAND.toString(),  new ValueEntry(_COMMAND));
        this.values.put(StepValues.TIMEOUT.toString(), new ValueEntry(_TIMEOUT));
        timeout = Double.parseDouble(_TIMEOUT);
        this.values.put(StepValues.SPEED.toString(),  new ValueEntry(_SPEED));
        this.values.put(StepValues.XDISTANCE.toString(),  new ValueEntry(_XDISTANCE));
        this.values.put(StepValues.YDISTANCE.toString(),  new ValueEntry(_YDISTANCE));
        this.values.put(StepValues.PARALLEL.toString(),  new ValueEntry(_PARALLEL));
        for (int i = 0; i < _PARMs.length; i++) {
            this.values.put("PARM" + i,  new ValueEntry(_PARMs[i]));
        }
    }

    public Step (String _COMMAND, double _TIMEOUT, double _SPEED, double _XDISTANCE, double _YDISTANCE, boolean _PARALLEL, String... _PARMs) {
        this(_COMMAND, String.valueOf(_TIMEOUT), String.valueOf(_SPEED), String.valueOf(_XDISTANCE), String.valueOf(_YDISTANCE), String.valueOf(_PARALLEL), _PARMs);
    }

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _DISTANCE, String _PARALLEL, String[] _PARMs) {
        this(_COMMAND, _TIMEOUT, _SPEED, _DISTANCE, _DISTANCE, _PARALLEL, _PARMs);
    }

    public Step() {
        super("Step");
    }

    public void registerMappings(HashMap<String, Mapping> suppliedMappings) {
        values.forEach((a, b) -> {
            if(b.genericType) {
                if(suppliedMappings.containsKey(b.value))
                    b.mappings = suppliedMappings.get(b.value);
                else
                    System.out.println("Failed to register for genericType: (" + b.value + ")");
            }
        });
    }

    public void setValue(StepValues key, Object val) {
        this.values.replace(key.toString(), new ValueEntry(String.valueOf(val)));
    }

    public void setValue(Pair<StepValues, String> pair) {
        this.values.put(pair.getFirst().toString(), new ValueEntry(pair.getSecond()));
    }

    public String getValue(String key) {
        key = key.toUpperCase();

        if(values.containsKey(key)) {
            ValueEntry entry = values.get(key);

            if(entry.genericType) {
                return String.valueOf(entry.mappings.getValue());
            } else {
                return entry.value;
            }
        } else {
            return "0";
        }
    }

    public boolean hasValue(String key) {
        return values.containsKey(key.toUpperCase());
    }

    public String getCommand() {
        return getValue(StepValues.COMMAND.toString());
    }

    public double getSpeed() {
        return Double.parseDouble(getValue(StepValues.SPEED.toString()));
    }

    public double getXDistance() {
        return Double.parseDouble(getValue(StepValues.XDISTANCE.toString()));
    }

    public double getYDistance() {
        return Double.parseDouble(getValue(StepValues.YDISTANCE.toString()));
    }

    public double getDistance() {
        return (getXDistance() + getYDistance()) / 2;
    }

    public boolean isParallel() {
        return Boolean.parseBoolean(getValue(StepValues.PARALLEL.toString()));
    }

    public void setParm(String parm, String value) {
        values.put(parm.toUpperCase(), new ValueEntry(value));
    }

    public Double getParm(int PARM) {
        return Double.parseDouble(getValue("PARM" + PARM));
    }

    public Double getParm(int PARM, Double falseReturn) {
        if (this.values.containsKey("PARM" + PARM)) {
            return Double.parseDouble(getValue("PARM" + PARM));
        } else {
            return falseReturn;
        }
    }

    public Integer getParmInt(int PARM) {
        if (this.values.containsKey("PARM" + PARM)) {
            return Integer.valueOf("PARM" + PARM);
        } else {
            return null;
        }
    }

    public boolean checkParm(int PARM) {
        return this.values.containsKey("PARM" + PARM);
    }

    public void changeStepState(Constants.StepState newStepState) {
        this.mCurrentStepState = newStepState;
    }

    public Constants.StepState getStepState() {
        return this.mCurrentStepState;
    }

    public void StartTimer() {
        startTime = System.currentTimeMillis();
    }

    public boolean hasTimeoutElapsed() {
        if(timeout != null && startTime != null)
            return hasTimeElapsed(new Time(timeout, MILLISECONDS));
        else
            return false;
    }

    public Time getTime() {
        return new Time(System.currentTimeMillis() - startTime, MILLISECONDS);
    }

    public boolean hasTimeElapsed(Time time) {
        return (System.currentTimeMillis() - startTime) >= time.getValue(MILLISECONDS);
    }

    private void propagateBlankValues() {
        for(StepValues a : StepValues.values()) {
            this.values.putIfAbsent(a.toString(), new ValueEntry("0"));
        }

        for(int i = 1; i < 8; i++) {
            this.values.putIfAbsent("PARM" + i, new ValueEntry("0"));
        }
    }

    public Element getXMLFileElement(Document doc) {
        propagateBlankValues();
        Element xmlFileElement = doc.createElement("Step");

        for(StepValues a : StepValues.values()) {
            Element tmp = doc.createElement(a.toString().toLowerCase());
            tmp.setTextContent(getValue(a.toString()));
            xmlFileElement.appendChild(tmp);
        }

        for(int i = 1; i < 8; i++) {
            Element tmp = doc.createElement("parm" + i);
            tmp.setTextContent(getValue("PARM" + i));
            xmlFileElement.appendChild(tmp);
        }

//
//            values.forEach((a, b) -> {
//                Element tmp = doc.createElement(a);
//                tmp.setTextContent(b);
//                xmlFileElement.appendChild(tmp);
//            });

        return xmlFileElement;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);

        for(Map.Entry<String, ValueEntry> val : values.entrySet()) {
            Entity.buildStringTabbedData(builder, depth, val.getKey(), val.getValue().value);
        }
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

//        for(ControllerMapping mapping : mappings.values()) {
//            mapping.addToNetworkTable(table);
//        }

        return table;
    }
}