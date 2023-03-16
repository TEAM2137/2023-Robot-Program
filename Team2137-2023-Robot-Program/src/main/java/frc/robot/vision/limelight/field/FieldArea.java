package frc.robot.vision.limelight.field;

public class FieldArea {
    // x and y represent the bottom left corner of the area
    public double x;
    public double y;

    public double width;
    public double height;

    public String id;

    public FieldArea(String id, double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.id = id;
    }

    public synchronized boolean insideArea(double px, double py, double robotWidth, double robotHeight) {
        boolean insideX = px > x && px < x + width;
        boolean insideY = py > y && py < y + height;
        return insideX && insideY;
    }
}
