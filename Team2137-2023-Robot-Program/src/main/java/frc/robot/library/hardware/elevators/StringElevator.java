package frc.robot.library.hardware.elevators;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.functions.io.xmlreader.objects.motor.SimpleMotorControl;
import frc.robot.library.Constants;
import frc.robot.library.units.TranslationalUnits.Distance;
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

    private ScheduledThreadPoolExecutor threadPoolExecutor = new ScheduledThreadPoolExecutor(1);

    private Constants.StepState mHomingStepState = STATE_FINISH;

    private final Mapping mHomingSensorMap;

    private final PIDController pidController;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent -
     * @param fileLogger -
     */
    public StringElevator(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

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
            mMaxTravel = new Distance(38.875, INCH);
        }

        mLiftMotors[0].setDistancePerRevolution(new Distance(mSpoolDiameter.getValue(INCH) * Math.PI, INCH));
        mLiftMotors[0].setIntegratedSensorDistance(mZeroPosition);

        mLiftMotors[0].enableReverseLimit();
        mLiftMotors[0].enableForwardLimit();

        mLiftMotors[0].configureForwardLimit(mMaxTravel);
        mLiftMotors[0].configureReverseLimit(new Distance(0, INCH));

        setSpeed(0.0);

        PID pid = mLiftMotors[0].getPID();
        pidController = new PIDController(pid.getP(), pid.getI(), pid.getD());
        pidController.setTolerance(0.1);

        mHomingSensorMap = (Mapping) this.getEntity("HomingMap");
        
        logger = fileLogger;

        this.addSubsystemCommand(getName() + "-SetRawPosition", this::setRawPosition);
        this.addSubsystemCommand(getName() + "-SetRawSpeed", this::setSpeed);
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
                    setSpeed(-0.25, false);
                    mGoalPosition = null;
                    mHomingStepState = STATE_RUNNING;
                }
                break;
            case STATE_RUNNING:
                if(mHomingSensorMap.getBooleanValue()) {
                    setSpeed(0.0);

                    mLiftMotors[0].enableForwardLimit();
                    mLiftMotors[0].enableReverseLimit();
                    mLiftMotors[0].setIntegratedSensorDistance(mZeroPosition);

                    mHomingStepState = Constants.StepState.STATE_FINISH;
                }
                break;

            default:
                if(!rawSpeedControl && mGoalPosition != null) {
                    setSpeed(pidController.calculate(mCurrentPosition.getValue(INCH)));
                }
                break;
        }
    }

    @Override
    public void setSpeed(double speed) {
        setSpeed(speed, true);
    }

    public void setSpeed(final double speed, boolean limit) {
//        if(limit)
//            if(mCurrentPosition.greaterThan(mMaxTravel) && speed > 0) {
//                setSpeed(0, false);
//                return;
//            } else if(0 > mCurrentPosition.getValue(FOOT) && speed < 0) {
//                setSpeed(speed, false);
//                return;
//            }

        if(speed > 0) {
            mRatchet.set(DoubleSolenoid.Value.kReverse);
        } else {
            mRatchet.set(DoubleSolenoid.Value.kForward);
        }

        threadPoolExecutor.schedule(() -> {
            mLiftMotors[0].set(speed);
        }, 100, TimeUnit.MILLISECONDS);
    }

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
                mGoalPosition = null;
                SmartDashboard.putNumber(getName() + "-Speed", 0.0);
            }
        }
    }

    public void homeElevator(Step step) {
        if(step.getStepState() == STATE_INIT) {
            mHomingStepState = STATE_INIT;

            DriverStation.reportWarning("Started Homing...", false);

            step.changeStepState(STATE_FINISH);
        }
    }

    @Override
    public void homeElevator() {
        mHomingStepState = STATE_INIT;
    }

    public void setRawPosition(Step step) {
        if (step.getStepState() == STATE_INIT) {
            setPosition(new Distance(step.getParm(1), INCH));

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    @Override
    public void setPosition(Distance distance) {
        DriverStation.reportWarning(getName() + "-SetPosition to " + distance.getValue(FOOT), false);
        logger.writeEvent(2, FileLogger.EventType.Debug, "Setting " + getName() + " Position to " + distance.getValue(FOOT) + "ft");

        if(distance.getValue(FOOT) < mLiftMotors[0].getPosition().getValue(FOOT)) {
            mRatchet.set(DoubleSolenoid.Value.kForward);
        } else {
            mRatchet.set(DoubleSolenoid.Value.kReverse);
        }

        mGoalPosition = distance;
        pidController.setSetpoint(mGoalPosition.getValue(INCH));
        //mLiftMotors[0].setPosition(distance);
    }
}
