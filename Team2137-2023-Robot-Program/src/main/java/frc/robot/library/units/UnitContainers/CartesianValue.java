package frc.robot.library.units.UnitContainers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;

public class CartesianValue<T extends Unit<?, ? extends UnitEnum>> {
    private T x;
    private T y;
    private T z;

    public CartesianValue(T _x, T _y, T _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    public T getX() {
        return x;
    }

    public void setX(T x) {
        this.x = x;
    }

    public T getY() {
        return y;
    }

    public void setY(T y) {
        this.y = y;
    }

    public T getZ() {
        return z;
    }

    public void setZ(T z) {
        this.z = z;
    }

}
