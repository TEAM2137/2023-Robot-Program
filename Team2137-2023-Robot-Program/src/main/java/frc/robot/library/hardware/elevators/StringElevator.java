package frc.robot.library.hardware.elevators;

import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.library.Constants;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;

import javax.swing.*;
import java.util.Collection;
import java.util.concurrent.Callable;

import static frc.robot.library.Constants.StepState.STATE_INIT;
import static frc.robot.library.Constants.StepState.STATE_NOT_STARTED;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class StringElevator extends EntityGroup implements Elevator {

    private SimpleMotorControl[] mLiftMotors;

    private FileLogger logger;

    private Distance mGoalPosition;
    private Distance mSpoolDiameter;

    private Constants.StepState mHomingStepState = STATE_NOT_STARTED;
    private Callable<Boolean> mHomingSensorState;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public StringElevator(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        fileLogger.writeEvent(4, "Entering String Elevator constructor");

        Collection<Entity> keys = getEntities();

        int count = 0;
        for (Entity a : keys) {
            if(a.getName() != null && a.getName().contains("Lift Motor"))
                count++;
        }

        fileLogger.writeEvent(4, "Found motor keys attempting to get control object");

        mLiftMotors = new SimpleMotorControl[count];
        for(Entity a : keys) {
            String name = a.getName();

            if(name.contains("Lift Motor")) {
                fileLogger.writeEvent(0, "Found lift motor with name: " + name);
                String[] parts = name.split(" ");
                int idx = Integer.parseInt(parts[parts.length - 1]) - 1;
                mLiftMotors[idx] = (SimpleMotorControl) a;
            }
        }

        fileLogger.writeEvent(0, "Attempting to create following relationships");
        for(int i = 0; i < mLiftMotors.length; i++) {
            if(i != 0)
                mLiftMotors[i].follow(mLiftMotors[0]);
        }

        if(this.getEntity("SpoolDiameter") != null) {
            mSpoolDiameter = new Distance(((Number) getEntity("SpoolDiameter")).getValue(), INCH);
        } else {
            mSpoolDiameter = new Distance(1, INCH);
        }
        mLiftMotors[0].setDistancePerRevolution(new Distance(mSpoolDiameter.getValue(INCH) * Math.PI, INCH));

        logger = fileLogger;

        this.addSubsystemCommand(getName() + "-SetRawPosition", this::setRawPosition);
        this.addSubsystemCommand(getName() + "-HomeElevator", this::homeElevator);
    }

    @Override
    public void periodic() {
        switch (mHomingStepState) {
            case STATE_INIT:
                if(mHomingSensorState == null) {
                    mHomingStepState = Constants.StepState.STATE_FINISH;
                } else {
                    setSpeed(-0.5);
                }
                break;
            case STATE_RUNNING:
                try {
                    if(mHomingSensorState.call()) {
                        setSpeed(0.0);
                        mLiftMotors[0].setIntegratedSensorPosition(0);
                        mHomingStepState = Constants.StepState.STATE_FINISH;
                    }
                } catch (Exception e) {
                    setSpeed(0.0);
                }
                break;
        }
    }

    @Override
    public void setSpeed(double speed) {
        mLiftMotors[0].set(speed);
    }

    public void homeElevator(Step step) {
        if(step.getStepState() == STATE_INIT) {
            homeElevator(mHomingSensorState);
            mHomingStepState = STATE_INIT;
        }
    }

    @Override
    public void homeElevator(Callable<Boolean> homeSensorState) {
        mHomingStepState = STATE_INIT;
        mHomingSensorState = homeSensorState;
    }

    public void setRawPosition(Step step) {
        if (step.getStepState() == STATE_INIT) {
            setPosition(new Distance(step.getParm(1), INCH));

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    @Override
    public void setPosition(Distance distance) {
        logger.writeEvent(2, FileLogger.EventType.Debug, "Setting " + getName() + " Position to " + distance.getValue(FOOT) + "ft");
        mGoalPosition = distance;
        mLiftMotors[0].setPosition(distance);
    }
}
