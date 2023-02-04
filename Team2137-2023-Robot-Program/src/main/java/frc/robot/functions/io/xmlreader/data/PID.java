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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import org.w3c.dom.Element;

public class PID extends Entity {

    private Double P, I, D, IZ, FF;
    private Double S, V, A;

    public PID (double _P, double _I, double _D, String name) {
        super(name);
        P = _P;
        I = _I;
        D = _D;
    }

    public PID (Double _P, Double _I, Double _D, Double _FF, Double _IZ, String name) {
        this(_P, _I, _D, name);
        IZ = _IZ;
        FF = _FF;
    }

    public PID (Double _P, Double _I, Double _D, Double _S, Double _V, Double _A, String name) {
        this(_P, _I, _D, name);
        S = _S;
        V = _V;
        A = _A;
    }

    public PID (Double _P, Double _I, Double _D, Double _FF, Double _IZ, Double _S, Double _V, Double _A, String name) {
        this(_P, _I, _D, _S, _V, _A, name);
        FF = _FF;
        IZ = _IZ;
    }

    public PID(Element element) {
        super(element);

        if (element != null) {
            var nodes = element.getElementsByTagName("P");
            if (nodes.getLength() > 0)
                P = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("I");
            if (nodes.getLength() > 0)
                I = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("D");
            if (nodes.getLength() > 0)
                D = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("IZ");
            if (nodes.getLength() > 0)
                IZ = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("FF");
            if (nodes.getLength() > 0)
                FF = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("S");
            if (nodes.getLength() > 0)
                S = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("V");
            if (nodes.getLength() > 0)
                V = Double.parseDouble(nodes.item(0).getTextContent());

            nodes = element.getElementsByTagName("A");
            if (nodes.getLength() > 0)
                A = Double.parseDouble(nodes.item(0).getTextContent());
        }
    }

    public double getP() {
        return P;
    }

    public void setP(Double p) {
        P = p;
    }

    public double getI() {
        return I;
    }

    public void setI(Double i) {
        I = i;
    }

    public double getD() {
        return D;
    }

    public void setD(Double d) {
        D = d;
    }

    public double getIZ() {
        return IZ;
    }

    public void setIZ(Double IZ) {
        IZ = IZ;
    }

    public double getFF() {
        return FF;
    }

    public void setFF(Double FF) {
        FF = FF;
    }

    public double getS() {
        return S;
    }

    public void setS(Double s) {
        S = s;
    }

    public double getV() {
        return V;
    }

    public void setV(Double v) {
        V = v;
    }

    public double getA() {
        return A;
    }

    public void setA(Double a) {
        A = a;
    }

    public Double[] getPIDArray() {
        return new Double[] {P, I, D};
    }

    public PIDController getWPIPIDController() {
        return new PIDController(P, I, D);
    }

    public SimpleMotorFeedforward getWPIFeedForwardController() {
        return new SimpleMotorFeedforward(S, V, A);
    }

    public void addToLogger(FileLogger logger, String tag) {
        logger.writeEvent(0, tag + " P", String.valueOf(P));
        logger.writeEvent(0, tag + " I", String.valueOf(I));
        logger.writeEvent(0, tag + " D", String.valueOf(D));
    }

    @Override
    public NetworkTable addToNetworkTable(NetworkTable dashboard) {
        NetworkTable table = super.addToNetworkTable(dashboard);

        NetworkTableEntry pValue = table.getEntry("P");
        pValue.setDouble(getP());

        NetworkTableEntry iValue = table.getEntry("I");
        iValue.setDouble(getI());

        NetworkTableEntry dValue = table.getEntry("D");
        dValue.setDouble(getD());

        return table;
    }

    @Override
    public NetworkTable pullFromNetworkTable() {
        NetworkTable table = super.pullFromNetworkTable();

        //TODO add type
        setP(table.getEntry("P").getDouble(getP()));
        setI(table.getEntry("I").getDouble(getI()));
        setD(table.getEntry("D").getDouble(getD()));

        return table;
    }
    @Override
    public NetworkTable removeFromNetworkTable() {
        NetworkTable table = super.removeFromNetworkTable();

        table.getEntry("P").unpublish();
        table.getEntry("I").unpublish();
        table.getEntry("D").unpublish();

        return table;
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        buildStringTabbedData(builder, depth, "P", String.valueOf(P));
        buildStringTabbedData(builder, depth, "I", String.valueOf(I));
        buildStringTabbedData(builder, depth, "D", String.valueOf(D));
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        if(P != null)
            getSavedElement().getElementsByTagName("P").item(0).setTextContent(String.valueOf(P));
        if(I != null)
            getSavedElement().getElementsByTagName("I").item(0).setTextContent(String.valueOf(I));
        if(D != null)
            getSavedElement().getElementsByTagName("D").item(0).setTextContent(String.valueOf(D));
        if(IZ != null)
            getSavedElement().getElementsByTagName("IZ").item(0).setTextContent(String.valueOf(IZ));
        if(FF != null)
            getSavedElement().getElementsByTagName("FF").item(0).setTextContent(String.valueOf(FF));
        if(S != null)
            getSavedElement().getElementsByTagName("S").item(0).setTextContent(String.valueOf(S));
        if(V != null)
            getSavedElement().getElementsByTagName("V").item(0).setTextContent(String.valueOf(V));
        if(A != null)
            getSavedElement().getElementsByTagName("A").item(0).setTextContent(String.valueOf(A));

        return getSavedElement();
    }
}
