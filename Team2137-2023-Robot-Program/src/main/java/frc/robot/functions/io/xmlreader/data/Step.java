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
import frc.robot.library.Constants;
import frc.robot.library.units.Time;
import frc.robot.library.units.Units;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.util.HashMap;

import static frc.robot.library.units.Units.Unit.MILLISECOND;

public class Step {
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

    private final HashMap<String, String> values = new HashMap<String, String>();
    private Constants.StepState mCurrentStepState = Constants.StepState.STATE_INIT;

    private double startTime;

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _XDISTANCE, String _YDISTANCE, String _PARALLEL, String[] _PARMs) {
        this.values.put(StepValues.COMMAND.toString(),  _COMMAND);
        this.values.put(StepValues.TIMEOUT.toString(), _TIMEOUT);
        this.values.put(StepValues.SPEED.toString(),  (_SPEED));
        this.values.put(StepValues.XDISTANCE.toString(),  (_XDISTANCE));
        this.values.put(StepValues.YDISTANCE.toString(),  (_YDISTANCE));
        this.values.put(StepValues.PARALLEL.toString(),  (_PARALLEL));
        for (int i = 0; i < _PARMs.length; i++) {
            this.values.put("PARM" + i,  _PARMs[i]);
        }
    }

    public Step (String _COMMAND, String _TIMEOUT, String _SPEED, String _DISTANCE, String _PARALLEL, String[] _PARMs) {
        this(_COMMAND, _TIMEOUT, _SPEED, _DISTANCE, _DISTANCE, _PARALLEL, _PARMs);
    }

    public Step(String[] _PARMs) {
        for (int i = 0; i < _PARMs.length; i++) {
            this.values.put("PARM" + i,  _PARMs[i]);
        }
    }

    public Step() {

    }

    public void setValue(StepValues key, Object val) {
        this.values.replace(key.toString(), String.valueOf(val));
    }

    public void setValue(Pair<StepValues, String> pair) {
        this.values.put(pair.getFirst().toString(), pair.getSecond());
    }

    public String getCommand() {
        return this.values.get(StepValues.COMMAND.toString());
    }

    public double getSpeed() {
        return Double.parseDouble(this.values.get(StepValues.SPEED.toString()));
    }

    public double getXDistance() {
        return Double.parseDouble(this.values.get(StepValues.XDISTANCE.toString()));
    }

    public double getYDistance() {
        return Double.parseDouble(this.values.get(StepValues.YDISTANCE.toString()));
    }

    public double getDistance() {
        return (getXDistance() + getYDistance()) / 2;
    }

    public boolean isParallel() {
        return Boolean.parseBoolean(this.values.get(StepValues.PARALLEL.toString()));
    }

    public void setParm(String parm, String value) {
        values.put(parm.toUpperCase(), value);
    }

    public Double getParm(int PARM) {
        return Double.parseDouble(values.get("PARM" + PARM));
    }

    public Double getParm(int PARM, Double falseReturn) {
        if (this.values.containsKey("PARM" + PARM)) {
            return Double.parseDouble(values.get("PARM" + PARM));
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

    public Time getTime() {
        return new Time(System.currentTimeMillis() - startTime, Units.Unit.SECOND);
    }

    public boolean hasTimeElapsed(Time time) {
        return (System.currentTimeMillis() - startTime) >= time.getValue(MILLISECOND);
    }

    private void propagateBlankValues() {
        for(StepValues a : StepValues.values()) {
            this.values.putIfAbsent(a.toString(), "0");
        }

        for(int i = 1; i < 8; i++) {
            this.values.putIfAbsent("PARM" + i, "0");
        }
    }

    public Element getXMLFileElement(Document doc) {
            propagateBlankValues();
            Element xmlFileElement = doc.createElement("Step");

            for(StepValues a : StepValues.values()) {
                Element tmp = doc.createElement(a.toString().toLowerCase());
                tmp.setTextContent(values.get(a.toString()));
                xmlFileElement.appendChild(tmp);
            }

            for(int i = 1; i < 8; i++) {
                Element tmp = doc.createElement("parm" + i);
                tmp.setTextContent(values.get("PARM" + i));
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
}