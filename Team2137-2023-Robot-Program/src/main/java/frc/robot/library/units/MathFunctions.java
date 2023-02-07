package frc.robot.library.units;

public interface MathFunctions<T> {
    T times(double scalar);

    T times(T other);

    T divide(double scalar);

    T divide(T other);

    T add(double scalar);

    T add(T other);
    T minus(double scalar);

    T minus(T other);
}
