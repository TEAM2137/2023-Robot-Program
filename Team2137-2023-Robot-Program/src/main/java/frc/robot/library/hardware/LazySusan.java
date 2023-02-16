package frc.robot.library.hardware;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.units.AngleUnits.Angle;
import org.w3c.dom.Element;

import java.util.function.Consumer;

public class LazySusan extends EntityGroup {

    private SimpleMotorControl simpleMotor;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public LazySusan(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        simpleMotor = (SimpleMotorControl) getEntity("SpinMotor");
    }

    public void setPosition(Angle angle) {
        simpleMotor.setPosition(angle);
    }

    public void homeLazySusan(Consumer<Boolean> sensorState) {

    }
}
