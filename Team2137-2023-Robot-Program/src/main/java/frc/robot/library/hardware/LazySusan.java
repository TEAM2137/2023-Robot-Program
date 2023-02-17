package frc.robot.library.hardware;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.Constants;
import frc.robot.library.units.AngleUnits.Angle;
import org.w3c.dom.Element;

import java.util.function.Consumer;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;

public class LazySusan extends EntityGroup {

    private final SimpleMotorControl simpleMotor;

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

        this.addSubsystemCommand(getName() + "SetPosition", this::setPosition);
        this.addSubsystemCommand(getName() + "SetSpeed", this::setSpeed);
    }

    public void setPosition(Angle angle) {
        simpleMotor.setPosition(angle);
    }

    public void setPosition(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT) {
            setPosition(new Angle(step.getParm(1), DEGREE));
        }
    }

    public void setSpeed(double speed) {
        simpleMotor.set(speed);
    }

    public void setSpeed(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT && step.getParm(2) < Math.abs(step.getParm(1))) {
            simpleMotor.set(step.getParm(1));
        }
    }

    public void homeLazySusan(Consumer<Boolean> sensorState) {

    }
}
