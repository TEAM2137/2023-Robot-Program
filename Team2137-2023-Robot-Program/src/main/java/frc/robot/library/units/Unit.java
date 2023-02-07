package frc.robot.library.units;

public interface Unit<T, V extends UnitEnum> extends MathFunctions<T> {
    void setValue(double val, V unit);
    double getValue(V unit);

    double getValueInPrimaryUnit();

    V getPrimaryUnit();
}
