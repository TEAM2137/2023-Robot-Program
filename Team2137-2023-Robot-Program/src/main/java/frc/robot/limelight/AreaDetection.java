package frc.robot.limelight;

import java.util.ArrayList;
import org.opencv.core.Point;
import frc.robot.limelight.field.RectArea;

public class AreaDetection {

    // A list of rectangular areas around the left side of the field
    ArrayList<RectArea> leftSideAreas = new ArrayList<>(3);

    // A list of the positions of the aprilTags on the left side of the field
    ArrayList<Point> leftSideTags = new ArrayList<>(3);

    public void init() {
        leftSideTags.add(new Point(40.45, 174.19));
        leftSideTags.add(new Point(40.45, 108.19));
        leftSideTags.add(new Point(40.45, 42.19));

        addLeftSideAreas();
    }

    public double xDstToNearestTag() {
        Point p = dstToNearestTag();
        if (p == null) {
            return 0;
        } else {
            return p.x;
        }
    }

    public double yDstToNearestTag() {
        Point p = dstToNearestTag();
        if (p == null) {
            return 0;
        } else {
            return p.y;
        }
    }

    public Point dstToNearestTag() {
        int id = getAreaId();
        if (id >= 0) {
            Point tagPos = leftSideTags.get(id);
            return new Point(tagPos.x - AprilTags.getX(), tagPos.y = AprilTags.getY());
        } else {
            return null;
        }
    }

    private void addLeftSideAreas() {
        for (int i = 0; i < leftSideTags.size(); i++) {
            Point position = new Point(leftSideTags.get(i).x, leftSideTags.get(i).y - 30);
            leftSideAreas.add(new RectArea(i, position.x, position.y, 60, 60));        
        }
    }

    public int getAreaId() {
        if (getArea() == null) {
            return -1;
        } else {
            return getArea().id;
        }
    }

    public RectArea getArea() {

        // if on left side
        if (AprilTags.getX() < AprilTags.fieldSizeX) {
            for (int i = 0; i < leftSideAreas.size(); i++) {
                if (leftSideAreas.get(i).insideArea(AprilTags.getX(), AprilTags.getY())) {
                    // in this area
                    return leftSideAreas.get(i);
                }
            }
        }

        // not in any area
        return null;
    }

    public boolean inAnyAreas() {

        // if on left side
        if (AprilTags.getX() < AprilTags.fieldSizeX) {
            for (int i = 0; i < leftSideAreas.size(); i++) {
                if (leftSideAreas.get(i).insideArea(AprilTags.getX(), AprilTags.getY())) {
                    // in an area
                    return true;
                }
            }
        }

        // not in any area
        return false;
    }

}