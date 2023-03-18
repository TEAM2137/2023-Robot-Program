package frc.robot.vision.limelight;

import java.util.ArrayList;
import org.opencv.core.Point;

import frc.robot.vision.limelight.field.FieldArea;

public class AreaDetection {

    private ArrayList<FieldArea> fieldAreas = new ArrayList<>(20);

    private ArrayList<Point> leftSideTags = new ArrayList<>(3);
    private ArrayList<Point> rightSideTags = new ArrayList<>(3);

    private double xPos;
    private double yPos;
    private double rw;
    private double rh;

    /*
     * POINTS (INCHES)
     * 
     * Left community, blue: 56.25
     * Right community, blue: 190.93
     * Left community, red: 458.10
     * Right community, red:
     * Community width, large: 132.38
     * Community width, small: 74.75
     * Community width difference: 57.63
     * Charge station, blue: (116.94, 59.39)
     * AprilTag to center line X: 285.16
     * Half field size X: 325.61
     * Half field size Y: 159.50
     * Community to center line: 269.36
     * Community edge to center line: 136.98
     * 
     * These are just so I can calculate the positions more easily
     */

    public AreaDetection() {
        // Adds the coordinates of the apriltags
        leftSideTags.add(new Point(40.45, 42.19));
        leftSideTags.add(new Point(40.45, 108.19));
        leftSideTags.add(new Point(40.45, 174.19));

        rightSideTags.add(new Point(610.7, 174.19));
        rightSideTags.add(new Point(610.7, 108.19));
        rightSideTags.add(new Point(610.7, 42.19));

        addFieldAreas();
        addRightSideTagAreas();
        addLeftSideTagAreas();
    }

    private synchronized void addFieldAreas() {
        fieldAreas.add(new FieldArea("community-blue1", 56.26, 0, 132.38, 159.50));
        fieldAreas.add(new FieldArea("community-blue2", 56.26, 0, 74.75, 216.03));
        fieldAreas.add(new FieldArea("community-red1", 458.10, 0, 132.38, 159.50));
        fieldAreas.add(new FieldArea("community-red2", 515.73, 0, 74.75, 216.03));
        fieldAreas.add(new FieldArea("chargestation-blue", 116.94, 59.39, 148.125, 193.25));
        fieldAreas.add(new FieldArea("chargestation-red", 458.10, 59.39, 148.125, 193.25));
    }

    private synchronized void addLeftSideTagAreas() {
        for (int i = 0; i < leftSideTags.size(); i++) {
            Point position = new Point(leftSideTags.get(i).x, leftSideTags.get(i).y - 30);
            fieldAreas.add(new FieldArea("AprilTagArea#" + i, position.x, position.y, 60, 60));
        }
    }

    private synchronized void addRightSideTagAreas() {
        for (int i = 0; i < rightSideTags.size(); i++) {
            Point position = new Point(rightSideTags.get(i).x - 60, rightSideTags.get(i).y - 30);
            fieldAreas.add(new FieldArea("AprilTagArea#" + i, position.x, position.y, 60, 60));
        }
    }

    /**
     * Updates the position of the robot.
     * @param x the X position of the robot, in inches, with (0,0) as the bottom left corner of the field.
     * @param y the Y position of the robot, in inches, with (0,0) as the bottom left corner of the field.
     */
    public synchronized void setPosition(double x, double y) {
        xPos = x;
        yPos = y;
    }

    /**
     * @return a list of all the areas that the robot is currently in
     */
    public synchronized FieldArea[] getAreas() {
        for (int i = 0; i < fieldAreas.size(); i++) {
            if (fieldAreas.get(i).insideArea(xPos, yPos, rw, rh)) {

            }
        }
        return null;
    }

    /**
     * Checks if the robot is within the specified field area.
     * <p> List of all the valid names to check:
     * <p><code>"communityblue-1"
     * <p>"communityblue-2"
     * <p>"communityred-1"
     * <p>"communityred-2"
     * <p>"chargestation-blue"
     * <p>"chargestation-red"
     * </code>
     * @param name the name of the area to check (use one of the above)
     * @return true if the robot is in the desired area, false if it isn't.
     */
    public synchronized boolean isInArea(String name) {
        FieldArea[] areas = getAreas();
        for (int i = 0; i < areas.length; i++) {
            if (areas[i].id == name) {
                return true;
            }
        }
        return false;
    }

    /**
     * Checks if the robot is within ANY of the specified field areas.
     * <p>NOTE: This function should be used instead of <code>isInArea()</code> if you are
     * checking if the robot is in a red community/blue community, by checking for both community
     * names of each color.
     * <p>List of all the valid names to check:
     * <p><code>"communityblue-1"
     * <p>"communityblue-2"
     * <p>"communityred-1"
     * <p>"communityred-2"
     * <p>"chargestation-blue"
     * <p>"chargestation-red"
     * </code>
     * @param name the name of the area to check (use one of the above)
     * @return true if the robot is in any of the desired areas, false if it isn't.
     */
    public synchronized boolean isInAreas(String[] names) {
        FieldArea[] areas = getAreas();
        for (int i = 0; i < areas.length; i++) {
            for (int j = 0; j < names.length; j++) {
                if (areas[i].id == names[i]) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Checks if the robot is in any of the preset field areas
     * @return true if the robot is in any area, false if it isn't.
     */
    public synchronized boolean inAnyAreas() {
        if (getAreas().length > 0) {
            return true;
        } else {
            return false;
        }
    }

}