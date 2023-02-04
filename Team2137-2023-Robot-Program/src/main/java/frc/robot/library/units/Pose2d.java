package frc.robot.library.units;

import edu.wpi.first.math.geometry.Rotation2d;

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

    public void mutableAddVector(Vector2d<T> vector, Angle angle) {
        x = (T) x.add(vector.x);
        y = (T) y.add(vector.y);

        theta = (Angle) theta.add(angle);
    }
}
