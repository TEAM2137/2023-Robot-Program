package frc.robot.library.units.UnitContainers;

import frc.robot.library.units.Number;

public class Vector3d<T extends Number> {
    T x;
    T y;
    T z;

    public Vector3d(T _x, T _y, T _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    public Vector3d scale(double scale) {
        return new Vector3d(x.times(scale), y.times(scale), z.times(scale));
    }

    public Vector3d add(Vector3d other) {
        return new Vector3d(x.times(other.x), y.times(other.y), z.times(other.z));
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

    public CartesianValue<T> getCartesianValues() {
        return new CartesianValue<T>(getX(), getY(), getZ());
    }

    public void print() {
        System.out.println("X: " + getX().getValue());
        System.out.println("Y: " + getY().getValue());
        System.out.println("Z: " + getZ().getValue());
    }
}
