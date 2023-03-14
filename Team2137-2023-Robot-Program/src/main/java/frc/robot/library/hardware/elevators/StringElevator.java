package frc.robot.library.hardware.elevators;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.InstantCommand;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.functions.io.xmlreader.data.mappings.PersistentCommand;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.Constants;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;

import java.util.Collection;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import static frc.robot.library.Constants.StepState.*;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class StringElevator extends EntityGroup implements Elevator {

    private final SimpleMotorControl[] mLiftMotors;
    private DoubleSolenoid mRatchet;

    private boolean rawSpeedControl = false;

    private final FileLogger logger;

    private Distance mGoalPosition;
    private final Distance mMaxTravel;
    private final Distance mSpoolDiameter;
    private final Distance mZeroPosition;
    private Distance mCurrentPosition;
    private Distance mTolerance;
    private Number mGravityGain;

    private ScheduledThreadPoolExecutor threadPoolExecutor = new ScheduledThreadPoolExecutor(1);

    private Constants.StepState mHomingStepState = STATE_NOT_STARTED;

    private final Mapping mHomingSensorMap;

    private final PIDController pidController;

    private final FileLogger fileLogger;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent -
     */
    public StringElevator(Element element, EntityGroup parent) {
        super(element, parent, true);
        fileLogger = getLogger();

        fileLogger.writeEvent(4, "Entering String Elevator constructor");

        Collection<Entity> keys = getEntities();

        int countLift = 0;
        for (Entity a : keys) {
            if(a.getName() != null)
                if(a.getName().contains("Lift Motor")) {
                    countLift++;
                }
        }

        fileLogger.writeEvent(4, "Found motor keys attempting to get control object");
        mLiftMotors = new SimpleMotorControl[countLift];

        for(Entity a : keys) {
            String name = a.getName();

            if(name == null)
                continue;

            if(name.contains("Lift Motor")) {
                fileLogger.writeEvent(0, "Found lift motor with name: " + name);
                String[] parts = name.split(" ");
                int idx = Integer.parseInt(parts[parts.length - 1]) - 1;
                mLiftMotors[idx] = (SimpleMotorControl) a;
            } else if(name.contains("Ratchet")) {
                fileLogger.writeEvent(0, "Found Double Solenoid Ratchet with name: " + name);
                mRatchet = (DoubleSolenoid) a;
            }
        }

        fileLogger.writeEvent(0, "Attempting to create following relationships");
        for(int i = 0; i < mLiftMotors.length; i++) {
            if(i != 0)
                mLiftMotors[i].follow(mLiftMotors[0]);
        }

        if(this.getEntity("ZeroPosition") != null) {
            mZeroPosition = (Distance) getEntity("ZeroPosition");
        } else {
            mZeroPosition = new Distance(0, INCH);
        }

        if(this.getEntity("SpoolDiameter") != null) {
            mSpoolDiameter = (Distance) getEntity("SpoolDiameter");
        } else {
            mSpoolDiameter = new Distance(1, INCH);
        }

        if(this.getEntity("MaxTravel") != null) {
            mMaxTravel = (Distance) getEntity("MaxTravel");
        } else {
            mMaxTravel = new Distance(32.0, INCH);
        }

        if(this.getEntity("GravityGain") != null) {
            mGravityGain = (Number) getEntity("GravityGain");
        } else {
            mGravityGain = new Number(0);
        }

        if(this.getEntity("Tolerance") != null) {
            mTolerance = (Distance) getEntity("Tolerance");
        } else {
            mTolerance = new Distance(1, INCH);
        }

        mLiftMotors[0].setDistancePerRevolution(new Distance(mSpoolDiameter.getValue(INCH) * Math.PI, INCH));
        mLiftMotors[0].setIntegratedSensorDistance(mZeroPosition);

        if(mHomingStepState == STATE_FINISH) {
            mLiftMotors[0].enableReverseLimit();
            mLiftMotors[0].enableForwardLimit();

            mLiftMotors[0].configureForwardLimit(mMaxTravel);
            mLiftMotors[0].configureReverseLimit(new Distance(0, INCH));
        }

        setSpeed(0.0);

        PID pid = mLiftMotors[0].getPID();
        pidController = new PIDController(pid.getP(), pid.getI(), pid.getD());
        pidController.setTolerance(mTolerance.getValue(INCH));

        mHomingSensorMap = (Mapping) this.getEntity("HomingMap");
        
        logger = fileLogger;

//        Robot.robotStateSubscribers.add((state) -> {
//            if(state == Constants.RobotState.TELEOP && mHomingStepState == STATE_NOT_STARTED) {
//                mHomingStepState = STATE_INIT;
//            }
//        });

        this.addSubsystemCommand(getName() + "-SetRawPosition", this::setRawPosition);
        this.addSubsystemCommand(getName() + "-SetRawSpeed", this::setSpeed);
        this.addSubsystemCommand(getName() + "-SetSingleRawSpeed", this::setSingleSpeed);
        this.addSubsystemCommand(getName() + "-HomeElevator", this::homeElevator);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(getName() + "-" + mHomingSensorMap.getPseudoName(), mHomingSensorMap.getBooleanValue());
        mCurrentPosition = mLiftMotors[0].getPosition();
        SmartDashboard.putNumber(getName() + "-CurrentPosition", mCurrentPosition.getValue(FOOT));
        SmartDashboard.putNumber(getName() + "-RawOutput", mLiftMotors[0].get());

        switch (mHomingStepState) {
            case STATE_INIT:
                if(mHomingSensorMap == null) {
                    mHomingStepState = Constants.StepState.STATE_FINISH;
                } else {
                    mLiftMotors[0].disableForwardLimit();
                    mLiftMotors[0].disableReverseLimit();
                    DriverStation.reportError("Starting the Home seqence for " + getName(), false);
                    setSpeed(-0.375);
                    mGoalPosition = null;
                    mHomingStepState = STATE_RUNNING;
                }
                break;
            case STATE_RUNNING:
                if(mHomingSensorMap.getBooleanValue()) {
                    setSpeed(0.0);

                    mLiftMotors[0].setIntegratedSensorDistance(mZeroPosition);
                    mLiftMotors[0].configureForwardLimit(mMaxTravel);
                    mLiftMotors[0].configureReverseLimit(new Distance(0, INCH));
//                    mLiftMotors[0].enableForwardLimit();
//                    mLiftMotors[0].enableReverseLimit();

                    mHomingStepState = Constants.StepState.STATE_FINISH;
                }
                break;

            default:
                if(!rawSpeedControl && mGoalPosition != null) {
                    double desiredSpeed = pidController.calculate(mCurrentPosition.getValue(INCH));
                    setSpeed(desiredSpeed + ((desiredSpeed > 0) ? mGravityGain.getValue() : 0));
                }
                break;
        }
    }

    @Override
    public void setSpeed(final double speed) {

        if(mRatchet != null) {
            if (speed > 0) {
                mRatchet.set(DoubleSolenoid.Value.kReverse);
            } else {
                mRatchet.set(DoubleSolenoid.Value.kForward);
            }

            threadPoolExecutor.schedule(() -> {
                mLiftMotors[0].set(speed);
            }, 100, TimeUnit.MILLISECONDS);
        } else {
            mLiftMotors[0].set(speed);
        }
    }

    @InstantCommand
    public void setSingleSpeed(Step step) {
        if(step.getStepState() == STATE_INIT){
            setSpeed(step.getSpeed());

            step.changeStepState(STATE_FINISH);
        }
    }

    @PersistentCommand
    public void setSpeed(Step step) {
        if(step.getStepState() == STATE_INIT) {
            if(step.getSpeed() != 0) {
                rawSpeedControl = true;
                setSpeed(step.getSpeed());
                mGoalPosition = null;
                SmartDashboard.putNumber(getName() + "-Speed", step.getSpeed());
            } else if (rawSpeedControl) {
                rawSpeedControl = false;
                setSpeed(0.0);

                if(mRatchet != null)
                    mGoalPosition = null;
                else
                    mGoalPosition = mLiftMotors[0].getPosition();

                SmartDashboard.putNumber(getName() + "-Speed", 0.0);
            }
        }
    }

    @InstantCommand
    public void homeElevator(Step step) {
        switch(step.getStepState()) {
            case STATE_INIT:
                mHomingStepState = STATE_INIT;
                DriverStation.reportWarning("Started Homing...", false);
                step.changeStepState(STATE_RUNNING);
                break;
            case STATE_RUNNING:
                if(mHomingStepState == STATE_FINISH)
                    step.changeStepState(STATE_FINISH);
                break;
        }
    }

    @Override
    public void homeElevator() {
        mHomingStepState = STATE_INIT;
    }

    @InstantCommand
    public void setRawPosition(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:

                if(step.hasValue("delay")) {
                    Robot.threadPoolExecutor.schedule(() -> {
                        setPosition(new Distance(step.getParm(1), INCH));
                    }, Integer.parseInt(step.getValue("delay")), TimeUnit.MILLISECONDS);
                } else {
                    setPosition(new Distance(step.getParm(1), INCH));
                }

                step.changeStepState(STATE_RUNNING);
                break;
            case STATE_RUNNING:
                if(pidController.atSetpoint())
                    step.changeStepState(STATE_FINISH);
                break;
        }
    }

    @Override
    public void setPosition(Distance distance) {
        DriverStation.reportWarning(getName() + "-SetPosition to " + distance.getValue(FOOT), false);
        logger.writeEvent(2, FileLogger.EventType.Debug, "Setting " + getName() + " Position to " + distance.getValue(FOOT) + "ft");

        mGoalPosition = distance;

        pidController.setSetpoint(mGoalPosition.getValue(INCH));
    }
}
