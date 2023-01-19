package frc.robot.library.hardware.endeffector;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import org.w3c.dom.Element;

public class EndEffector extends EntityGroup {

    private FileLogger logger;
    private boolean closed = false; // Informational value to see if the end effector is closed

    // TODO: Add pneumatics here

    public EndEffector(Element element, int depth, boolean printProcess, FileLogger fileLogger){
        super(element, depth, printProcess, fileLogger);

        logger = fileLogger;

        // TODO: Initialize pneumatics here
    }

    @Override
    public void periodic(){
        // Update dashboard stats
        SmartDashboard.putBoolean("End Effector Closed", closed);
    }

    /**
     * Gets the close state of the end effector.
     * @return Returns true if closed, and false if open.
     */
    public boolean getClosed() {
        return closed;
    }

    /**
     * Toggles the end effector state. Useful for mapping button presses to the end effector.
     */
    public void toggleEffectorState() {
        setEffectorState(!closed);
    }

    /**
     * Sets the end effector closed state.
     * @param isClosed New state of the end effector. True is closed and false is open.
     */
    public void setEffectorState(boolean isClosed){
        if(closed == isClosed) return; // Early leave to not waste time opening an open claw

        closed = isClosed; // Set to new state

        // TODO: Change pneumatics to new state
    }

    /**
     * Logs the state of the end effector and writes it to file
     */
    public void logEndEffectorState(){
        StringBuilder builder = new StringBuilder();

        builder.append("Q~EES~"); // Starting string mirroring Q~SWDSE
        builder.append(Boolean.toString(closed)).append(" ");

        logger.writeLine(builder.toString());
    }
}