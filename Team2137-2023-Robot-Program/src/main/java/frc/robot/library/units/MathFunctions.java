package frc.robot.library.units;

public interface MathFunctions<T> {
    T times(double scalar);
    T divide(double scalar);

    T add(double scalar);

    T add(T other);
    T minus(double scalar);
}
