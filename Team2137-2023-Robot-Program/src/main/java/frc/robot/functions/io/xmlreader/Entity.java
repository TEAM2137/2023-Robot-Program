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

package frc.robot.functions.io.xmlreader;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.util.concurrent.Callable;

public interface Entity {

//    private final String strName;
//    private Element savedElement;
//    private NetworkTable currentInstance;
//    private Callable<Boolean> onDestroyCallback;
//    private Runnable onImplementCallback;

    /**
     * Constructs a new Entity with only a name value (Not linked to XML Element)
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number.
     * @param name - Name of the Entity
     */
//    public Entity(String name) {
//        strName = name;
//        onDestroyCallback = () -> false;
//        Robot.allEntities.add(this);
//        Robot.deviceCallList.put(strName, this);
//    }

//    public Entity(String name, String defaultValue) {
//        if(name == null) {
//            strName = defaultValue;
//        } else {
//            strName = name;
//        }
//        Robot.allEntities.add(this);
//
//        onDestroyCallback = () -> false;
//    }

    /**
     * Constructs a new Entity with Element linkage
     * Usage: Entity is the main type of the XML File and represents any of the elements whether it is hardware or
     * just a number
     * @param element
     */
//    public Entity(Element element) {
//        this(getNodeOrAttribute(element, "name", null), element.getTagName());
//
//        savedElement = element;
//    }

    /**
     * Safe method that tried to retrieve a string from SubElement with given name
     * @param element - Element to search
     * @param name - Name of the SubElement to search for
     * @param defaultReturn - Default string value to return if it does not exist
     * @return - Returns String value from XML file or on fail the defaultReturn value
     */
    public static String getOrDefault(Element element, String name, String defaultReturn) {
        NodeList childNodes = element.getChildNodes();
        for(int i = 0; i < childNodes.getLength(); i++) {
            Node a = childNodes.item(i);

            if (Node.ELEMENT_NODE != a.getNodeType()) {
                continue;
            }

            if(((Element) a).getTagName().equalsIgnoreCase(name)) {
                return childNodes.item(i).getTextContent();
            }
        }

        return defaultReturn;
    }

    /**
     * Safe method that tried to retrieve a string from SubElement with given name set by an attribute
     * @param element - Element to search
     * @param name - Name of the SubElement to search for
     * @param defaultReturn - Default string value to return if it does not exist
     * @return - Returns String value from XML file or on fail the defaultReturn value
     */
    public default String getAttributeOrDefault(Element element, String name, String defaultReturn) {
        String value = element.getAttribute(name);
        if(value.equals(""))
            return defaultReturn;
        else
            return value;
    }

    /**
     * Safe method that tried to retrieve a string from SubElement with given name set by a Name Node
     * @param element - Element to search
     * @param name - Name of the SubElement to search for
     * @param defaultReturn - Default string value to return if it does not exist
     * @return - Returns String value from XML file or on fail the defaultReturn value
     */
    public default String getNodeOrAttribute(Element element, String name, String defaultReturn) {
        String capitalizedFirstLetter = String.valueOf(name.charAt(0)).toUpperCase() + name.substring(1).toLowerCase();

        String nodeResult = getOrDefault(element, capitalizedFirstLetter, null);
        String attributeResult = getAttributeOrDefault(element, name.toLowerCase(), null);

        if(nodeResult != null)
            return nodeResult;
        else if (attributeResult != null)
            return attributeResult;
        else
            return defaultReturn;
    }

    /**
     * Gets the set name of the Entity
     * @return - If no name is present return "Default"
     */
    public String getName();

    /**
     * Appends the Entity values into a {@see StringBuilder} and is meant to be Overwritten by Child classes in order
     * to add more values to the Entity print out (ie. CAN ID for a motor controller)
     * @param builder - StringBuilder to append values to
     * @param depth - Current depth (amount of tabs to add)
     */
    public default void constructTreeItemPrintout(StringBuilder builder, int depth) {
        builder.append("\n");
        buildStringTabbedData(builder, Math.max(0, depth - 1), "Name", getName());
    }

    /**
     * Write the value of the Entity with correct tabs
     * @param builder - Builder to append to
     * @param number - Number of tabs to add
     * @param title - Title of the value
     * @param message - Value or message about value
     */
    public static void buildStringTabbedData(StringBuilder builder, int number, String title, String message) {
        builder.append("\t".repeat(number));
        builder.append(title);
        builder.append(": ");
        builder.append(message);
        builder.append("\n");
    }

    public void setOnImplementCallback(Runnable run);
    public Runnable getOnImplementCallback();

    public default void OnImplement() {
        if(getOnImplementCallback() != null)
            this.getOnImplementCallback().run();
    }

    /**
     * Adds this Entity to the Network Tables as a SubTable
     * @param instance - Parent Network table instance
     * @return - SubTable instance
     */
    public default NetworkTable addToNetworkTable(NetworkTable instance) {
        setCurrentNetworkInstance(instance.getSubTable(getName()));
        return getCurrentNetworkInstance();
    }

    /**
     * Gets the value from the Network Table and sets it to the objects
     * @return - SubTable instance
     */
    public default NetworkTable pullFromNetworkTable() {
        return getCurrentNetworkInstance();
    }

    /**
     * To be implemented but removes this Entity SubTable in the Network Tables
     * Child classes should extend this and add all XML values
     */
    public default NetworkTable removeFromNetworkTable() {
        return getCurrentNetworkInstance();
    }

    /**
     * Adds this Entity to the Network Tables as a SubTable with different name value
     * Child classes should extend this and add all XML values
     * @param instance - Parent Network table instance
     * @return - SubTable instance
     */
    public default NetworkTable addToNetworkTable(String name, NetworkTable instance) {
        setCurrentNetworkInstance(instance.getSubTable(name));
        return getCurrentNetworkInstance();
    }

    public NetworkTable getCurrentNetworkInstance();
    public void setCurrentNetworkInstance(NetworkTable instance);

    /**
     * Returns the linked Element object
     * @return - Linked Element object
     */
    Element getSavedElement();

    /**
     * Updates the linked Element's (if present) values to the stored one
     * Child class should extend this and update all XML values
     * @return - Returns the linked element
     */
    public default Element updateElement() {
        if (getSavedElement() == null || this.getName() == null)
            return getSavedElement();

        if(getSavedElement().hasAttribute(getName().toLowerCase()))
            getSavedElement().setAttribute("name", getName());
        else if (getSavedElement().getElementsByTagName("Name").getLength() > 0)
            getSavedElement().getElementsByTagName("Name").item(0).setTextContent(getName());

        return getSavedElement();
    }
}