package frc.robot.functions.splines;

import frc.robot.library.units.Time;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class TrapezoidalVelocity {
    private Acceleration mMaxAcceleration;
    private Acceleration mMaxStopAcceleration;
    private Velocity mMaxVelocity;

    private Distance mTotalDistance;

    private Distance mAccelerationDistance;
    private Distance mStartDecelerationDistance;

    private Velocity mMaxAchievableVelocity;

    public TrapezoidalVelocity(Distance totalDistance) {
        mMaxAcceleration = new Acceleration(4, METER_PER_SECOND2);
        mMaxStopAcceleration = new Acceleration(-6, METER_PER_SECOND2);
        mMaxVelocity = new Velocity(4.6, METER_PER_SECOND);
        mTotalDistance = totalDistance;

        Time mAccelerationTime = mMaxVelocity.divide(mMaxAcceleration);
        Time mDecelerationTime = mMaxVelocity.divide(mMaxStopAcceleration);

        //0.5 * a * t^2
        mAccelerationDistance = new Distance(0.5 * mMaxAcceleration.getValue(METER_PER_SECOND2) * Math.pow(mAccelerationTime.getValue(SECONDS), 2), METER);
        Distance mDecelerationDistance = new Distance(0.5 * -mMaxStopAcceleration.getValue(METER_PER_SECOND2) * Math.pow(mDecelerationTime.getValue(SECONDS), 2), METER);

        Time mZeroAccelerationTime = new Time(mTotalDistance.minus(mAccelerationDistance).minus(mDecelerationDistance).getValue(METER) / mMaxVelocity.getValue(METER_PER_SECOND), SECONDS);

        if(mZeroAccelerationTime.getValue(SECONDS) < 0) {
            mAccelerationDistance = new Distance((-mMaxStopAcceleration.getValue(METER_PER_SECOND2) * totalDistance.getValue(METER)) / (mMaxAcceleration.getValue(METER_PER_SECOND2) + -mMaxStopAcceleration.getValue(METER_PER_SECOND2)), METER);
            mDecelerationDistance = totalDistance.minus(mAccelerationDistance);
        }

        mMaxAchievableVelocity = new Velocity(Math.sqrt(2 * mMaxAcceleration.getValue(METER_PER_SECOND2) * mAccelerationDistance.getValue(METER)), METER_PER_SECOND);
        mStartDecelerationDistance = totalDistance.minus(mDecelerationDistance);
    }

    public Velocity targetVelocity(Distance distance) {
        if(distance.getValue(METER) < mAccelerationDistance.getValue(METER)) {
            //v^2 = 2a(x)
            return new Velocity(Math.sqrt(2 * mMaxAcceleration.getValue(METER_PER_SECOND2) * distance.getValue(METER)), METER_PER_SECOND);
        } else if (distance.getValue(METER) > mStartDecelerationDistance.getValue(METER)) {
            return new Velocity(Math.sqrt(Math.pow(mMaxAchievableVelocity.getValue(METER_PER_SECOND), 2) + (2 * mMaxStopAcceleration.getValue(METER_PER_SECOND2) * (distance.getValue(METER) - mStartDecelerationDistance.getValue(METER)))), METER_PER_SECOND);
        } else {
            return mMaxAchievableVelocity;
        }
    }
}
