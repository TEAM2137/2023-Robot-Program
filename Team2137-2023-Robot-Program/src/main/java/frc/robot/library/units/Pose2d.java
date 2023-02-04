package frc.robot.library.units;

public class Pose2d<T extends Number> {
    T x;
    T y;
    Angle theta;

    public Pose2d(T _x, T _y, Angle theta) {
        x = _x;
        y = _y;
    }

    public T getX() {
        return x;
    }

    public T getY() {
        return y;
    }

    public Angle getTheta() {
        return theta;
    }
}
