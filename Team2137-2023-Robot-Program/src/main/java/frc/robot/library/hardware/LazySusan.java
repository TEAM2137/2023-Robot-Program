package frc.robot.library.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.Constants;
import frc.robot.library.units.AngleUnits.Angle;
import org.w3c.dom.Element;

import java.util.function.Consumer;

import static frc.robot.library.Constants.StepState.STATE_INIT;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;

public class LazySusan extends EntityGroup {

    private final SimpleMotorControl simpleMotor;

    private boolean rawSpeedControl = false;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     */
    public LazySusan(Element element, EntityGroup parent) {
        super(element, parent, false);

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
        if(step.getStepState() == STATE_INIT) {
            if(step.getParm(1) != 0) {
                rawSpeedControl = true;
                simpleMotor.set(step.getParm(1));
                SmartDashboard.putNumber(getName() + "-Speed", step.getParm(1));
            } else if (rawSpeedControl) {
                rawSpeedControl = false;
                simpleMotor.set(0);
                SmartDashboard.putNumber(getName() + "-Speed", 0);
            }
        }
    }

    public void homeLazySusan(Consumer<Boolean> sensorState) {

    }
}
