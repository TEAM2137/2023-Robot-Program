package frc.robot.library.hardware;

import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class FalconSimulation {

    public static final double StallCurrent = 257;
    public static final double FreeCurrent = 1.50;
    public static final double FreeSpeed = 6380;
    public static final double StallTorque = 4.69;

    public static double getTorqueCurveSlopeFromVoltage(double voltage) {
        return -4.31E-6 * Math.pow(voltage, 2) + 1.06E-4 * voltage - 1.38E-3;
    }

    public static double getTorqueCurveInterceptFromVoltage(double voltage) {
        return 0.0156 * Math.pow(voltage, 2) + 0.0193 * voltage + 2.19;
    }

    public static double getTorque(double rpm, double voltage) {
        return getTorqueCurveSlopeFromVoltage(voltage) * rpm + getTorqueCurveInterceptFromVoltage(voltage);
    }

    public static double getForce(double rpm, double voltage, double r) {
        return getTorque(rpm, voltage) / r;
    }

    public static Acceleration getAcceleration(double rpm, double voltage, double r, double mass) {
        return new Acceleration(getForce(rpm, voltage, r) / mass, METER_PER_SECOND2);
    }


    private final double mCurrentRPM = 0;
    private final boolean mDirectionForward = true;
    private boolean mMotorEnabled = true;
    private final boolean mStopping = false;
    private boolean mSlip = false;

    //Current Variables
    private double mVoltage = 0;
    private double mVelocity = 0;
    private double mDistance = 0;
    private final double mRestingVoltage = 0;

    private long mLastRunTime = 0;

    //Const Values
    private final double mWheelRadius = 0;
    private final double mMaxCurrent = 0;
    private final double mMotorRatio = 0;
    private final double mRobotMass = 60;
    private final double mMaxVelocityChange = 0;
    private final double mResistance = 0;
    private final double mCoefficientOfStaticFriction = 0;
    private final double mCoefficientOfDynamicFriction = 0;
    private final String mStopMethod = "Brake";

    public FalconSimulation() {

        mLastRunTime = System.currentTimeMillis();
    }

    public void run() {
        double filtering = 0.6;
        double eff = 0.8;

        double mu_s = mCoefficientOfStaticFriction * (mRobotMass / 100);
        double mu_k = mCoefficientOfDynamicFriction * (mRobotMass / 100);

        double km = StallTorque / (StallCurrent - FreeCurrent);
        double ke = 12 / FreeSpeed;
        double R = 12 / (StallCurrent - FreeCurrent);
        double Tslip_s = 9.8 * mRobotMass * mu_s * mWheelRadius;
        double Tslip_k = 9.8 * mRobotMass * mu_k * mWheelRadius;
        double Tloss = StallTorque * (1 - eff);
        double vmax = FreeSpeed / mMotorRatio * mWheelRadius;

        double dt = System.currentTimeMillis() - mLastRunTime;

        // current calculation
        double I = mMotorEnabled ? (mVoltage - mVelocity * mMotorRatio / mWheelRadius * ke) / R : 0;
        I = Math.min(Math.abs(I), mMaxCurrent) * Math.signum(I);

        // torque calculation
        double Tmotor = (I - FreeCurrent) * km;
        double T = Tmotor * mMotorRatio * eff - Tloss * (mVelocity / vmax) - (mStopping ? Tloss / eff : 0);

        // slip detection
        if (mSlip && Math.abs(T) < Tslip_k) mSlip = false;
        else if (!mSlip && Math.abs(T) > Tslip_s) mSlip = true;
        if (mSlip) {
            T = Tslip_k * Math.signum(T);
            Tmotor = T / (mMotorRatio * eff);
            I = (Math.abs(Tmotor / km) + FreeCurrent) * Math.signum(Tmotor);
        }

        // acceleration calculation
        double F = T / mWheelRadius;
        double a = F / mRobotMass;
        mVelocity += a * dt;
        mDistance += mVelocity * dt + 0.5 * a * Math.pow(dt, 2); //v0 + 1/2at^2

        // console.log(t, stop, x, v, a, V, connected, slip, T, Tmotor, I);

        if (mStopMethod != "Reverse" && mStopping) I = 0;

        double Vnew = 0;

        if (!mStopping) {
            Vnew = mRestingVoltage - mResistance * I;
        } else {
            switch (mStopMethod) {
                case "Coast":
                    Vnew = 0;
                    mMotorEnabled = false;
                    break;
                case "Brake":
                    Vnew = 0;
                    break;
                case "Reverse":
                    mVoltage = -(mRestingVoltage - mResistance * I);
                    break;
            }
        }
        mVoltage = (Math.abs(Vnew - mVoltage) > mMaxVelocityChange ? mVoltage + mMaxVelocityChange * Math.signum(Vnew - mVoltage) : Vnew * filtering + mVoltage * (1 - filtering));

        mLastRunTime = System.currentTimeMillis();
    }

    public double getVelocity() {
        return mVelocity;
    }
}
