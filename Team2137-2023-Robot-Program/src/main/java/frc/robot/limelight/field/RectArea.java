package frc.robot.limelight.field;

public class RectArea {

    // x and y represent the bottom left corner of the area
    public double x;
    public double y;

    public double width;
    public double height;

    public int id;

    public RectArea(int id, double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.id = id;
    }

    public boolean insideArea(double px, double py) {
        boolean insideX = px > x && px < x + width;
        boolean insideY = py > y && py < y + height;
        return insideX && insideY;
    }
}
