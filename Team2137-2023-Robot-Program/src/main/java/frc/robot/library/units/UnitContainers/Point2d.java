package frc.robot.library.units.UnitContainers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngleUnit;
import frc.robot.library.units.Time;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;

public class Point2d <T extends Unit<T, ? extends UnitEnum>> {
    T x;
    T y;

    public Point2d(T _x, T _y) {
        x = _x;
        y = _y;
    }

    public Point2d(T _m, AngleUnit<?, ?> theta) {
        double xVal = _m.getValueInPrimaryUnit() * Math.cos(theta.getValueInPrimaryUnit());
        double yVal = _m.getValueInPrimaryUnit() * Math.sin(theta.getValueInPrimaryUnit());
        x = (T) UnitUtil.create(xVal, _m.getPrimaryUnit());
        y = (T) UnitUtil.create(yVal, _m.getPrimaryUnit());
    }

    public Point2d(double _x, double _y, UnitEnum unit) {
        x = (T) UnitUtil.create(_x, unit);
        y = (T) UnitUtil.create(_y, unit);
    }

    public Point2d(T _m, Rotation2d theta) {
        double xVal = _m.getValueInPrimaryUnit() * Math.cos(theta.getRadians());
        double yVal = _m.getValueInPrimaryUnit() * Math.sin(theta.getRadians());
        x = (T) UnitUtil.create(xVal, _m.getPrimaryUnit());
        y = (T) UnitUtil.create(yVal, _m.getPrimaryUnit());
    }

    public Point2d normalize() {
        double length = Math.sqrt(Math.pow(x.getValueInPrimaryUnit(), 2) + Math.pow(y.getValueInPrimaryUnit(), 2));

        return new Point2d(x.divide(length), y.divide(length));
    }

    public Point2d normalize(double val) {
        double length = Math.sqrt(Math.pow(x.getValueInPrimaryUnit(), 2) + Math.pow(y.getValueInPrimaryUnit(), 2));

        return new Point2d(x.divide(length).times(val), y.divide(length).times(val));
    }

    public Point2d scale(double scale) {

        T newX = x.times(scale);
        T newY = y.times(scale);

        return new Point2d(newX, newY);
    }

    public Angle getAngle() {
        return new Angle(Math.atan2(getY().getValueInPrimaryUnit(), getX().getValueInPrimaryUnit()), Angle.AngleUnits.RADIAN);
    }

    public T getX() {
        return x;
    }

    public T getY() {
        return y;
    }

    public T getMagnitude() {
        double d = Math.sqrt(Math.pow(getX().getValueInPrimaryUnit(), 2) + Math.pow(getY().getValueInPrimaryUnit(), 2));
        return (T) UnitUtil.create(d, getX().getPrimaryUnit());
    }

    public Point2d<?> times(Time dt) {
        if(x instanceof Velocity) {
            return new Point2d<Distance>(((Velocity) x).times(dt), ((Velocity) y).times(dt));
        } else if (y instanceof Acceleration) {
            return new Point2d<Velocity>(((Acceleration) x).times(dt), ((Acceleration) y).times(dt));
        } else {
            return null;
        }
    }
//    /**
//     * Will return in Inches
//     * @param translation2d
//     * @return
//     */
//    public Translation2d applyVector(Translation2d translation2d) {
//        return new Translation2d(translation2d.getX() + this.x, translation2d.getY() + this.y);
//    }

    public double slope() {
        return y.getValueInPrimaryUnit() / x.getValueInPrimaryUnit();
    }

    public void mutableAdd(Point2d<T> other) {
        x = x.add(other.x);
        y = y.add(other.y);
    }
}
