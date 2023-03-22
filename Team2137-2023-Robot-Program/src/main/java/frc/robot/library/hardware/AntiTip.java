package frc.robot.library.hardware;

import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Time;
import frc.robot.library.units.UnitContainers.CartesianValue;
import org.w3c.dom.Element;

import static frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.DEGREE_PER_SECOND2;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.Time.TimeUnits.MILLISECONDS;

public class AntiTip {

    private CartesianValue<Angle> mAngle;
    private CartesianValue<Angle> mLastAngle;
    private CartesianValue<AngularVelocity> mAngularVelocity;
    private CartesianValue<AngularVelocity> mLastVelocity;
    private CartesianValue<AngularAcceleration> mAngularAcceleration;

    private Angle mInitialAngle;

    private static AngularVelocity mTippingAngularVelocity;
    private static AngularAcceleration mTippingAngularAcceleration;

    private long lastTime;
    private DriveTrain mDrivetrain;

    public AntiTip() {

        this.mDrivetrain = (SwerveDrivetrain) Robot.robotEntityGroup.getEntityGroupByType("DriveTrain");
        lastTime = System.currentTimeMillis();
    }

    public boolean update() {
        mLastAngle = mAngle;
        mLastVelocity = mAngularVelocity;

        Time dt = new Time(System.currentTimeMillis() - lastTime, MILLISECONDS);

        mAngle = this.mDrivetrain.getCartesianAngles();

        AngularVelocity xAngularVelocity = mAngle.getX().minus(mLastAngle.getX()).divide(dt);
        AngularVelocity yAngularVelocity = mAngle.getY().minus(mLastAngle.getY()).divide(dt);
        mAngularVelocity = new CartesianValue<>(xAngularVelocity, yAngularVelocity, null);

        AngularAcceleration xAngularAcceleration = mLastVelocity.getX().minus(mLastVelocity.getX()).divide(dt);
        AngularAcceleration yAngularAcceleration = mLastVelocity.getY().minus(mLastVelocity.getY()).divide(dt);
        mAngularAcceleration = new CartesianValue<>(xAngularAcceleration, yAngularAcceleration, null);

        double angularVelocityMagnitude = Math.sqrt(Math.pow(xAngularVelocity.getValue(DEGREE_PER_SECOND), 2) + Math.pow(yAngularVelocity.getValue(DEGREE_PER_SECOND), 2));
        double angularVelocityDirection = Math.atan2(yAngularVelocity.getValue(DEGREE_PER_SECOND), xAngularVelocity.getValue(DEGREE_PER_SECOND));

        double angularAccelerationMagnitude = Math.sqrt(Math.pow(xAngularAcceleration.getValue(DEGREE_PER_SECOND2), 2) + Math.pow(yAngularAcceleration.getValue(DEGREE_PER_SECOND2), 2));
        double angularAccelerationDirection = Math.atan2(yAngularAcceleration.getValue(DEGREE_PER_SECOND2), xAngularAcceleration.getValue(DEGREE_PER_SECOND2));

        if(angularVelocityMagnitude > mTippingAngularVelocity.getValue(DEGREE_PER_SECOND) && angularVelocityDirection - angularAccelerationDirection < Math.toRadians(45)) {
            return true;
        }
        return false;
    }
}
