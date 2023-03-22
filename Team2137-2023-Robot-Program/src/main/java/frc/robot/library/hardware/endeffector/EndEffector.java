package frc.robot.library.hardware.endeffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.InstantCommand;
import frc.robot.functions.io.xmlreader.data.mappings.Mapping;
import frc.robot.functions.io.xmlreader.data.mappings.PersistentCommand;
import frc.robot.functions.io.xmlreader.objects.motor.NeoMotor;
import frc.robot.functions.io.xmlreader.objects.solenoid.DoubleSolenoid;
import frc.robot.library.Constants;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;

import java.util.concurrent.TimeUnit;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;

/**
 * Comments by Wyatt 2.7.2023 (Sorry it is a lot)
 * [DONE] At line 36 maybe use the PID class inorder to store the values instead of the double so that we can tune them easier.
 * By using the class we can enable and disable when we want to tune the pid values look at @see SwerveFALCONDriveModule.class
 *
 * [DONE] Possible rename getClose() to isJawClosed() or something a little more descriptive.
 *
 * To create the motor try to name them and use the EntityGroup static function inorder to get a Motor.class object which
 * contains all the information.
 * Gage: ^^ The above function doesn't exist
 *
 * [DONE] The closed loop error needs to be changed to allow for more otherwise the wrist will be unpredictable
 *
 * [DONE] Pitch Encoder needs to be a Absolute Encoder not relative so we can track the position of the wrist no matter starting
 * position. (Maybe use Encoder.class to hold values also) << Gage: I don't see much point to this. Why use two objects when I can get all the info from just one?
 *
 * [DONE] You do not need to set position in the periodic loop. That will take up the CAN utilization. The value that you set is
 * saved on the motor controller. It only need to be set in a setPosition function.
 *
 * Not sure the getRPM math is correct but overall might not be needed.
 * Gage: ^^ RPM math was taken from last year's code, relevant values have been modified accordingly.
 *
 * [DONE] The main branch has a class called Angle.class which might be useful for storing the angles or you could use the FIRST
 * version of this Rotation2d.class
 *
 * [DONE] Probably add FileLogging in the intialization block of code.
 */
public class EndEffector extends EntityGroup {

    private final FileLogger logger;

    private Angle pitchTarget = new Angle(180, DEGREE);
    private long lastPitchSpeedTime;
    private boolean rawPitchControl = true;
    private boolean mLastAutoCloseSensorValue = false;
    private Timer mLastManualControlTime = new Timer();

    private DoubleSolenoid jaw1;

    private final NeoMotor pitchMotor;
    private Number mTolerance;
    private Mapping autoCloseMapping;
    private final double pitchAllowedErr = 0.5;
    private final double feedforward;

    public EndEffector(Element element, EntityGroup parent){
        super(element, parent, true);

        logger = getLogger();

        jaw1 = (DoubleSolenoid) getEntity("JawSolenoid");
        setEffectorState(true);
        logger.writeLine("ENDEFFECTOR: Jaw Solenoid Initialized");

        pitchMotor = (NeoMotor) getEntity("WristMotor");
        feedforward = pitchMotor.getPID().getFF();
        logger.writeLine("ENDEFFECTOR: Pitch Motor Initialized");

        if(this.getEntity("Tolerance") != null) {
            mTolerance = (Number) getEntity("Tolerance");
        } else {
            mTolerance = new Number(4);
        }

        pitchMotor.configureForwardLimit(new Angle(158, DEGREE));
        pitchMotor.configureReverseLimit(new Angle(0, DEGREE));

        autoCloseMapping = (Mapping) getEntity("AutoCloseSensorMap");

        this.addSubsystemCommand(getName() + "-SetRawClaw", this::setRawClaw);
        this.addSubsystemCommand(getName() + "-RawWristPosition", this::setWristPosition);
        this.addSubsystemCommand(getName() + "-RawWristSpeed", this::setWristSpeed);
    }

    //20 deg/sec
    @PersistentCommand
    public void setWristSpeed(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT) {
            if(step.getParm(1) != 0) {
                if(!rawPitchControl) {
                    lastPitchSpeedTime = System.currentTimeMillis();
                    rawPitchControl = true;
                } else {
                    double dP = Math.pow(Math.sin(step.getParm(1) * (Math.PI / 2)), 3) * 45 * ((System.currentTimeMillis() - lastPitchSpeedTime) / 1000.0);
                    Angle newTarget = getPitch().add(new Angle(dP, DEGREE));

                    setTargetPitch(newTarget);
                }
            } else if (rawPitchControl) {
                rawPitchControl = false;
//                pitchMotor.set(0.0);
                //setTargetPitch(getPitch());
            }
        }
    }

    @InstantCommand
    public void setWristPosition(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                if(step.hasValue("delay")) {
                    Robot.threadPoolExecutor.schedule(() -> {
                        setTargetPitch(new Angle(step.getParm(1), DEGREE));
                    }, Integer.parseInt(step.getValue("delay")), TimeUnit.MILLISECONDS);
                } else {
                    setTargetPitch(new Angle(step.getParm(1), DEGREE));
                }

                step.changeStepState(Constants.StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                if(atTargetPitch())
                    step.changeStepState(Constants.StepState.STATE_FINISH);
                break;
        }
    }

    @InstantCommand
    public void setRawClaw(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT) {
            setEffectorState(step.getParm(1) > 0);

            mLastManualControlTime.restart();

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    @Override
    public void periodic() {
        if(autoCloseMapping != null) {
            if (mLastManualControlTime.hasElapsed(1) && !mLastAutoCloseSensorValue && autoCloseMapping.getBooleanValue()) {
                setEffectorState(true);
            }

            mLastAutoCloseSensorValue = autoCloseMapping.getBooleanValue();
        }


        SmartDashboard.putNumber("Claw Current", pitchMotor.getOutputAmperage());

        // Update dashboard stats
        SmartDashboard.putBoolean("End Effector Closed", isJawClosed());
        SmartDashboard.putNumber("Pitch (Degrees)", getPitch().getValue(DEGREE));
        SmartDashboard.putNumber("Target Pitch (Degrees)", pitchTarget.getValue(DEGREE));
        SmartDashboard.putNumber("Pitch Allowed Error", pitchAllowedErr);
    }

    /**
     * Gets the close state of the end effector.
     * @return Returns true if closed, and false if open.
     */
    public boolean isJawClosed() {
        return jaw1.get() == edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
    }

    /**
     * Toggles the end effector state. Useful for mapping button presses to the end effector.
     */
    public void toggleEffectorState() {
        setEffectorState(!isJawClosed());
    }

    /**
     * Sets the end effector closed state.
     * @param isClosed New state of the end effector. True is closed and false is open.
     */
    public void setEffectorState(boolean isClosed){
        jaw1.set(isClosed ? edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward : edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse);
    }

    /**
     * Sets the speed of the pitch motor
     * @param speed Speed of the motor in range from -1 to 1
     */
//    public void setPitchMotorSpeed(double speed) {
//        pitchSpeed = speed;
//        pitchPIDController.setReference(pitchSpeed, CANSparkMax.ControlType.kVelocity);
//    }

    /**
     * Sets the speed of the pitch motor in RPM
     * @param rpm Speed of the motor in RPM
     */
//    public void setPitchMotorRPM(double rpm){
//        setPitchMotorSpeed(rpm / 60.0 * 2048 / 10.0);
//    }

    /**
     * Uses PID to move to the target pitch
     * @param degrees Position in degrees to set the pitch
     */
    public void setTargetPitch(Angle degrees){
        pitchTarget = degrees;
        pitchMotor.setPosition(pitchTarget, -Math.sin((Math.PI) - degrees.getValue(RADIAN)) * feedforward);
        //pitchPIDController.setReference(pitchTarget.getDegrees() * 360 / 4096, CANSparkMax.ControlType.kSmartMotion);
    }

    /**
     * Gets the current pitch in encoder units of the pitch motor
     * @return Pitch in encoder counts
     */
    public Angle  getPitch(){
        return pitchMotor.getAnglePosition();
    }

    public boolean atTargetPitch() {
        return pitchTarget.minus(getPitch()).getValue(DEGREE) <= mTolerance.getValue();
    }

    /**
     * Gets the current pitch in degrees
     * @return Pitch in degrees
     */
//    public double getPitchDegrees(){
//        return getPitch() / 4096 * 360;
//    }

    /**
     * Logs the state of the end effector and writes it to file
     */
//    public void logEndEffectorState(){
//        StringBuilder builder = new StringBuilder();
//
//        builder.append("Q~EES~"); // Starting string mirroring Q~SWDSE
//        builder.append(isJawClosed()).append(" ");
//
//        logger.writeLine(builder.toString());
//    }

    /*
    Slows down the CAN Frame periods for optimization
     */
    public void limitCANFrames(){
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 255);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 255);
    }

    /*
    Sets the CAN Frame periods back to default values
     */
    public void resetCANFrames(){
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 50);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 20);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 200);
//        pitchMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 200);
    }
}