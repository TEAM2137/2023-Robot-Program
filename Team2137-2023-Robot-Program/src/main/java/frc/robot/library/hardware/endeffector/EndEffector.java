package frc.robot.library.hardware.endeffector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import org.w3c.dom.Element;

public class EndEffector extends EntityGroup {

    private FileLogger logger;

    private double pitchTarget = 0;

    private Solenoid leftJaw;
    private Solenoid rightJaw;

    private CANSparkMax pitchMotor;

    // TODO: PID for pitchMotor
    private PID pitchPID = new PID(0.25, 0.25, 0.0, "Pitch Motor");

    public EndEffector(Element element, int depth, boolean printProcess, FileLogger fileLogger){
        super(element, depth, printProcess, fileLogger);

        logger = fileLogger;

        leftJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        rightJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

        pitchMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);

        // TODO: PID for pitchMotor
//        pitchMotor.getPIDController().setP(pitchPID.getP());
//        pitchMotor.getPIDController().setI(pitchPID.getI());
//        pitchMotor.getPIDController().setD(pitchPID.getD());
    }

    @Override
    public void periodic(){
        // TODO: Move to pitch logic

        // Update dashboard stats
        SmartDashboard.putBoolean("End Effector Closed", getClosed());
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
        return pitchMotor.getEncoder().getPosition();
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