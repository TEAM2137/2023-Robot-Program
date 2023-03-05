package frc.robot.library.units.UnitContainers;

import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

// TODO: Class incomplete. Math required later. Should be fine for now though.
public class Point3d <T extends Unit<T, ? extends UnitEnum>> {
    T x;
    T y;
    T z;

    public Point3d(T _x, T _y, T _z){
        x = _x;
        y = _y;
        z = _z;
    }
    public Point3d(double _x, double _y, double _z, UnitEnum unit){
        x = (T) UnitUtil.create(_x, unit);
        y = (T) UnitUtil.create(_y, unit);
        z = (T) UnitUtil.create(_z, unit);
    }

    public T getX() {
        return x;
    }

    public T getY() {
        return y;
    }

    public T getZ() {
        return z;
    }

    /**
     * Gets the X value in the primary unit.
     * @return Returns the X value in the dimension's primary unit.
     */
    public double getXPrimary() {
        return x.getValueInPrimaryUnit();
    }

    /**
     * Gets the Y value in the primary unit.
     * @return Returns the Y value in the dimension's primary unit.
     */
    public double getYPrimary() {
        return y.getValueInPrimaryUnit();
    }

    /**
     * Gets the Z value in the primary unit.
     * @return Returns the Z value in the dimension's primary unit.
     */
    public double getZPrimary() {
        return z.getValueInPrimaryUnit();
    }
}
