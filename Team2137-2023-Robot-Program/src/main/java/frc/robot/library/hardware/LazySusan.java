package frc.robot.library.hardware;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.hardware.simpleMotorControl.SimpleMotor;
import frc.robot.library.units.AngleUnits.Angle;
import org.w3c.dom.Element;

import java.util.function.Consumer;

public class LazySusan extends EntityGroup {

    private Motor rotationMotor;

    private SimpleMotor simpleMotor;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public LazySusan(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        simpleMotor = SimpleMotor.createMotor((Motor) getEntity("SpinMotor"));
    }

    public void setPosition(Angle angle) {

    }

    public void homeLazySusan(Consumer<Boolean> sensorState) {

    }
}
