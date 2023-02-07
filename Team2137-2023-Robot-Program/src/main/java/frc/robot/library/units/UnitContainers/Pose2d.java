package frc.robot.library.units.UnitContainers;

import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.TranslationUnit;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;

public class Pose2d<T extends Unit<T, ?>> {
    T x;
    T y;
    Angle theta;

    public Pose2d(T _x, T _y, Angle _theta) {
        x = _x;
        y = _y;
        theta = _theta;
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

    @Override
    public String toString() {
        return "X: " + x.getValueInPrimaryUnit() + " " + x.getPrimaryUnit().toString() + " Y: " + y.getValueInPrimaryUnit() + " " + y.getPrimaryUnit().toString() + " A: " + theta.getValueInPrimaryUnit() + theta.getPrimaryUnit().toString();
    }
}
