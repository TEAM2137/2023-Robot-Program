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

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.functions.io.xmlreader.Entity;
import org.w3c.dom.Element;

public class Camera extends Entity {
    private int deviceID;

    public Camera(Element element) {
        super(element);

        deviceID = Integer.parseInt(getOrDefault(element, "ID", "0"));
    }

    @Override
    public void constructTreeItemPrintout(StringBuilder builder, int depth) {
        super.constructTreeItemPrintout(builder, depth);
        buildStringTabbedData(builder, depth, "ID", String.valueOf(deviceID));
    }

    public int getDeviceID() {
        return deviceID;
    }

    public void setDeviceID(int deviceID) {
        this.deviceID = deviceID;
    }

    @Override
    public Element updateElement() {
        super.updateElement();

        getSavedElement().getElementsByTagName("ID").item(0).setTextContent(String.valueOf(deviceID));

        return getSavedElement();
    }

    @Override
    public NetworkTable removeFromNetworkTable(NetworkTable instance) {
        NetworkTable table = super.removeFromNetworkTable(instance);

        table.getEntry("ID").unpublish();

        return table;
    }
}
