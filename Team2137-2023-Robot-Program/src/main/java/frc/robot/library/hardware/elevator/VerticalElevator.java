package frc.robot.library.hardware.elevator;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import org.w3c.dom.Element;

public class VerticalElevator extends EntityGroup {

    

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public VerticalElevator(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);
    }
}
