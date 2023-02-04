package frc.robot.library.units;

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
        T newX = (T) x.clone();
        newX.setValue(x.getValue() * scale);
        T newY = (T) y.clone();
        newY.setValue(y.getValue() * scale);
        T newZ = (T) z.clone();
        newY.setValue(z.getValue() * scale);

        return new Vector3d(newX, newY, newZ);
    }

    public Vector3d add(Vector3d other) {
        T newX = (T) x.clone();
        newX.setValue(x.getValue() + other.x.getValue());
        T newY = (T) y.clone();
        newY.setValue(y.getValue() + other.y.getValue());
        T newZ = (T) z.clone();
        newY.setValue(z.getValue() + other.z.getValue());

        return new Vector3d(newX, newY, newZ);
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
