package frc.robot.library.units.UnitContainers;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngleUnit;
import frc.robot.library.units.TranslationalUnits.TranslationUnit;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;

public class Pose2d<T extends Unit<T, ?>, V extends AngleUnit<V, ?>> {
    T x;
    T y;
    V theta;

    public Pose2d(T _x, T _y, V _theta) {
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

    public V getTheta() {
        return theta;
    }

    public void mutableAddVector(Vector2d<T> vector, V angle) {
        x = x.add(vector.x);
        y = y.add(vector.y);

        theta = theta.add(angle);
    }

    @Override
    public String toString() {
        return "X: " + x.getValueInPrimaryUnit() + " " + x.getPrimaryUnit().toString() + " Y: " + y.getValueInPrimaryUnit() + " " + y.getPrimaryUnit().toString() + " A: " + theta.getValueInPrimaryUnit() + theta.getPrimaryUnit().toString();
    }
}
