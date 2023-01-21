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

public class EndEffector extends EntityGroup {

    private final FileLogger logger;

    private double pitchTarget = 0;

    private final Solenoid leftJaw;
    private final Solenoid rightJaw;

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

        leftJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        rightJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

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
        pitchPIDController.setReference(pitchTarget, CANSparkMax.ControlType.kSmartMotion);

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
        return leftJaw.get() && rightJaw.get();
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
        leftJaw.set(isClosed);
        rightJaw.set(isClosed);
    }

    /**
     * Sets the speed of the pitch motor
     * @param speed Speed of the motor in range from -1 to 1
     */
    public void setPitchMotorSpeed(double speed) {
        pitchMotor.set(speed);
    }

    /**
     * Sets the speed of the pitch motor in RPM
     * @param rpm Speed of the motor in RPM
     */
    public void setPitchMotorRPM(double rpm){
        setPitchMotorSpeed(rpm / 60.0 * 2048 / 10.0);
    }

    public void setTargetPitch(double degrees, double rpm){
        pitchTarget = degrees / 360 * 4096;
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