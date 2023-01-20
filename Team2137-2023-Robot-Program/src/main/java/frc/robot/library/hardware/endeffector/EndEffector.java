package frc.robot.library.hardware.endeffector;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import org.w3c.dom.Element;

public class EndEffector extends EntityGroup {

    private FileLogger logger;

    private Solenoid leftJaw;
    private Solenoid rightJaw;

    public EndEffector(Element element, int depth, boolean printProcess, FileLogger fileLogger){
        super(element, depth, printProcess, fileLogger);

        logger = fileLogger;

        leftJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        rightJaw = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    }

    @Override
    public void periodic(){
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
     * Logs the state of the end effector and writes it to file
     */
    public void logEndEffectorState(){
        StringBuilder builder = new StringBuilder();

        builder.append("Q~EES~"); // Starting string mirroring Q~SWDSE
        builder.append(getClosed()).append(" ");

        logger.writeLine(builder.toString());
    }
}