package frc.robot.functions.io.xmlreader.data.mappings;

import edu.wpi.first.networktables.NetworkTable;

public interface Mapping {
    double getValue();
    boolean getBooleanValue();
    boolean isBooleanValue();

    String getPseudoName();

    void constructTreeItemPrintout(StringBuilder builder, int depth);
    NetworkTable addToNetworkTable(NetworkTable dashboard);
}
