package frc.robot.library.hardware.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import org.w3c.dom.Element;

/**
 * Comments by Wyatt 2.7.2023 (Sorry it is a lot)
 * At line 36 maybe use the PID class inorder to store the values instead of the double so that we can tune them easier.
 * By using the class we can enable and disable when we want to tune the pid values look at @see SwerveFALCONDriveModule.class
 *
 * Possible rename getClose() to isJawClosed() or something a little more descriptive.
 *
 * To create the motor try to name them and use the EntityGroup static function inorder to get a Motor.class object which
 * contains all the information.
 *
 * The closed loop error needs to be changed to allow for more otherwise the wrist will be unpredictable
 *
 * Pitch Encoder needs to be a Absolute Encoder not relative so we can track the position of the wrist no matter starting
 * position. (Maybe use Encoder.class to hold values also)
 *
 * You do not need to set position in the periodic loop. That will take up the CAN utilization. The value that you set is
 * saved on the motor controller. It only need to be set in a setPosition function.
 *
 * Not sure the getRPM math is correct but overall might not be needed.
 *
 * The main branch has a class called Angle.class which might be useful for storing the angles or you could use the FIRST
 * version of this Rotation2d.class
 *
 * Probably add FileLogging in the intialization block of code.
 */
public class EndEffector extends EntityGroup {

    private final FileLogger logger;

    private double pitchTarget = 0;
    private double pitchSpeed = 0;
    private boolean targetMode = false;

    private final Solenoid jaw;

    private final CANSparkMax pitchMotor;

    private final RelativeEncoder pitchEncoder;

    private final SparkMaxPIDController pitchPIDController;
    private final double kP, kI, kD, kFF; // PID constants

    private final double maxVel, maxAccel;

    public EndEffector(Element element, int depth, boolean printProcess, FileLogger fileLogger){
        super(element, depth, printProcess, fileLogger);

        kP = 0.5;
        kI = 0.5;
        kD = 0.5;
        kFF = 0.00015;

        maxVel = 60;
        maxAccel = 45;

        logger = fileLogger;

        jaw = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        pitchMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
        pitchEncoder = pitchMotor.getEncoder();
        pitchPIDController = pitchMotor.getPIDController();

        // PID for pitchMotor
        pitchPIDController.setP(kP);
        pitchPIDController.setI(kI);
        pitchPIDController.setD(kD);
        pitchPIDController.setFF(kFF);
        pitchPIDController.setOutputRange(-1.0, 1.0);

        int smartMotionSlot = 0;
        pitchPIDController.setSmartMotionMaxAccel(maxAccel, smartMotionSlot);
        pitchPIDController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        pitchPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pitchPIDController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    }

    @Override
    public void periodic(){
        // Move to pitch logic
        if(targetMode) {
            pitchPIDController.setReference(pitchTarget, CANSparkMax.ControlType.kSmartMotion);
        }else{
            pitchPIDController.setReference(pitchSpeed, CANSparkMax.ControlType.kVelocity);
        }
        // Update dashboard stats
        SmartDashboard.putBoolean("End Effector Closed", getClosed());
        SmartDashboard.putNumber("Pitch (Degrees)", getPitchDegrees());
        SmartDashboard.putNumber("Pitch (Ticks)", getPitch());
        SmartDashboard.putNumber("Target Pitch (Ticks)", pitchTarget);

        // PID stats
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Feed Forward", kFF);
    }

    /**
     * Gets the close state of the end effector.
     * @return Returns true if closed, and false if open.
     */
    public boolean getClosed() {
        return jaw.get();
    }

    /**
     * Toggles the end effector state. Useful for mapping button presses to the end effector.
     */
    public void toggleEffectorState() {
        setEffectorState(!getClosed());
    }

    /**
     * Sets the end effector closed state.
     * @param isClosed New state of the end effector. True is closed and false is open.
     */
    public void setEffectorState(boolean isClosed){
        jaw.set(isClosed);
    }

    /**
     * Sets the speed of the pitch motor
     * @param speed Speed of the motor in range from -1 to 1
     */
    public void setPitchMotorSpeed(double speed) {
        pitchSpeed = speed;
        targetMode = false;
    }

    /**
     * Sets the speed of the pitch motor in RPM
     * @param rpm Speed of the motor in RPM
     */
    public void setPitchMotorRPM(double rpm){
        setPitchMotorSpeed(rpm / 60.0 * 2048 / 10.0);
    }

    /**
     * Uses PID to move to the target pitch
     * @param degrees Position in degrees to set the pitch
     */
    public void setTargetPitch(double degrees){
        pitchTarget = degrees / 360 * 4096;
        targetMode = true;
    }

    /**
     * Gets the current pitch in encoder units of the pitch motor
     * @return Pitch in encoder counts
     */
    public double getPitch(){
        return pitchEncoder.getPosition();
    }

    /**
     * Gets the current pitch in degrees
     * @return Pitch in degrees
     */
    public double getPitchDegrees(){
        return getPitch() / 4096 * 360;
    }

    /**
     * Logs the state of the end effector and writes it to file
     */
    public void logEndEffectorState(){
        StringBuilder builder = new StringBuilder();

        builder.append("Q~EES~"); // Starting string mirroring Q~SWDSE
        builder.append(getClosed()).append(" ");

        logger.writeLine(builder.toString());
    }
}