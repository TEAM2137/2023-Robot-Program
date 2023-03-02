package frc.robot.library.units.UnitContainers;

import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;

import static java.lang.Math.*;

public class Rectangle3d {

    /**
     * Stores the x position of the volume.
     */
    public Distance x;
    /**
     * Stores the y position of the volume.
     */
    public Distance y;
    /**
     * Stores the z position of the volume.
     */
    public Distance z;

    public Distance width;
    public Distance height;
    public Distance depth;

    /**
     * Rotation in radians on the y-axis.
     */
    public Angle rotation;

    private final Distance midPointX;
    private final Distance midPointZ;
    private Point3d[] points;

    public Rectangle3d(Distance x, Distance y, Distance z, Distance width, Distance height, Distance depth){
        this.x = x;
        this.y = y;
        this.z = z;

        this.width = width;
        this.height = height;
        this.depth = depth;

        this.rotation = new Angle(0, Angle.AngleUnits.RADIAN);

        midPointX = width.minus(x);
        midPointZ = depth.minus(z);
    }

    public Rectangle3d(Distance x, Distance y, Distance z, Distance width, Distance height, Distance depth, Angle rotation){
        this.x = x;
        this.y = y;
        this.z = z;

        this.width = width;
        this.height = height;
        this.depth = depth;

        this.rotation = rotation;

        midPointX = this.width.minus(this.x);
        midPointZ = this.depth.minus(this.z);

        points = new Point3d[]{
                // Rotate Lower Layer
                rotatePoint(new Point3d<>(this.x, this.y, this.z), this.rotation),
                rotatePoint(new Point3d<>(this.width, this.y, this.z), this.rotation),
                rotatePoint(new Point3d<>(this.width, this.y, this.depth), this.rotation),
                rotatePoint(new Point3d<>(this.x, this.y, this.depth), this.rotation),

                // Rotate Upper Layer
                rotatePoint(new Point3d<>(this.x, this.height, this.z), this.rotation),
                rotatePoint(new Point3d<>(this.width, this.height, this.z), this.rotation),
                rotatePoint(new Point3d<>(this.width, this.height, this.depth), this.rotation),
                rotatePoint(new Point3d<>(this.x, this.height, this.depth), this.rotation),
        };
    }

    /**
     * Gets the values for both points of the area used to initialize, with rotation applied.
     * @return
     */
    public Point3d<Distance>[] getPoints(){

        return points;
    }

    private Point3d<Distance> rotatePoint(Point3d<Distance> point, Angle rotate){
        return new Point3d<Distance>(
                midPointX.minus(point.x).times(cos(rotate.getValue(Angle.AngleUnits.RADIAN))).minus(midPointZ.minus(point.z).times(sin(rotate.getValue(Angle.AngleUnits.RADIAN)))).getValueInPrimaryUnit(),
                point.y.getValueInPrimaryUnit(),
                midPointZ.minus(point.z).times(cos(rotate.getValue(Angle.AngleUnits.RADIAN))).add(midPointX.minus(point.x).times(sin(rotate.getValue(Angle.AngleUnits.RADIAN)))).getValueInPrimaryUnit(),
                point.x.getPrimaryUnit()
        );
    }
}
