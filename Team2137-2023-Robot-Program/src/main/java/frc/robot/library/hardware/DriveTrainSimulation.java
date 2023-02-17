package frc.robot.library.hardware;

import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;

public class DriveTrainSimulation {

    enum MotorCharacteristics {
        FALCON (257, 1.50, 6380 * (Math.PI / 30.0), 4.69),
        NEO (105, 1.8, 5676 * (Math.PI / 30.0), 2.6),
        NEO550 (100, 1.4, 11000 * (Math.PI / 30.0), 0.97),
        PRO775 (134, 0.7, 18730 * (Math.PI / 30.0), 0.71),
        CIM (131, 2.7, 5330 * (Math.PI / 30.0), 2.41),
        MINI_CIM (89, 3, 5843 * (Math.PI / 30.0), 1.41),
        BAG (53, 1.8, 13180 * (Math.PI / 30.0), 0.43),
        WINDOW (16.25, 1.5, 80 * (Math.PI / 30.0), 9.24),
        NEVEREST (9.8, 0.4, 5480 * (Math.PI / 30.0), 0.173);

        final double stallCurrent;
        final double freeCurrent;
        final double freeSpeed;
        final double stallTorque;

        MotorCharacteristics(double _StallCurrent, double _FreeCurrent, double _FreeSpeed, double _StallTorque) {
            stallCurrent = _StallCurrent;
            freeCurrent = _FreeCurrent;
            freeSpeed = _FreeSpeed;
            stallTorque = _StallTorque;
        }

        public double getStallCurrent(int motorCount) {
            return stallCurrent * motorCount;
        }

        public double getFreeCurrent(int motorCount) {
            return freeCurrent * motorCount;
        }

        public double getFreeSpeed() {
            return freeSpeed;
        }

        public double getStallTorque(int motorCount) {
            return stallTorque * motorCount;
        }
    }

    public enum StoppingMethods {
        COAST,
        BRAKE,
        REVERSE
    }

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

    public final double loopTime = 10.0 / 1_000.0;

    private boolean mMotorEnabled = true;
    public boolean mStopping = false;
    private boolean mSlip = false;

    //Current Variables
    public double mVoltage = 12.5;
    public double mVelocity = 0;
    public double mDistance = 0;
    private final double mRestingVoltage = 12.5;

    public double mLastRunTime = 0;

    //Const Values
    private double mWheelRadius = 101.6 / 2.0 / 1_000;
    private double mMaxCurrent = Double.POSITIVE_INFINITY;
    private double mMotorRatio = 6.75;
    private double mRobotMass = 60;
    private double mMaxVoltageChange = 1_200 * loopTime;
    private double mResistance = 25.0 / 1_000;
    private double mCoefficientOfStaticFriction = 1.1;
    private double mCoefficientOfDynamicFriction = 1.0;
    public StoppingMethods mStopMethod = StoppingMethods.BRAKE;

    private MotorCharacteristics motorCharacteristics = MotorCharacteristics.FALCON;
    private final int motorCount = 4;

    double filtering = 0.6;
    double eff = 0.8;

    double mu_s = mCoefficientOfStaticFriction * (mRobotMass / 100);
    double mu_k = mCoefficientOfDynamicFriction * (mRobotMass / 100);

    double km = motorCharacteristics.getStallTorque(motorCount) / (motorCharacteristics.getStallCurrent(motorCount) - motorCharacteristics.getFreeCurrent(motorCount));
    double ke = 12 / motorCharacteristics.getFreeSpeed();
    double R = 12 / (motorCharacteristics.getStallCurrent(motorCount) - motorCharacteristics.getFreeCurrent(motorCount));
    double Tslip_s = 9.8 * mRobotMass * mu_s * mWheelRadius;
    double Tslip_k = 9.8 * mRobotMass * mu_k * mWheelRadius;
    double Tloss = motorCharacteristics.getStallTorque(motorCount) * (1 - eff);
    double vmax = motorCharacteristics.getFreeSpeed() / mMotorRatio * mWheelRadius;

    public DriveTrainSimulation(Distance wheelDiameter, double motorRatio, double robotMass, double coeStaticFriction, double coeDynamicFriction, MotorCharacteristics characteristics) {
        mWheelRadius = wheelDiameter.getValue(Distance.DistanceUnits.METER) / 2.0;
        mMotorRatio = motorRatio;
        mRobotMass = robotMass;
        mCoefficientOfStaticFriction = coeStaticFriction;
        mCoefficientOfDynamicFriction = coeDynamicFriction;
        motorCharacteristics = characteristics;
    }

    public DriveTrainSimulation() {

    }

    public void run() {
        // current calculation
        double I = mMotorEnabled ? (mVoltage - mVelocity * mMotorRatio / mWheelRadius * ke) / R : 0;
        I = Math.min(Math.abs(I), mMaxCurrent) * Math.signum(I);

        // torque calculation
        double Tmotor = (I - motorCharacteristics.getFreeCurrent(motorCount)) * km;
        double T = Tmotor * mMotorRatio * eff - Tloss * (mVelocity / vmax) - (mStopping ? Tloss / eff : 0);

        // slip detection
        if (mSlip && Math.abs(T) < Tslip_k) mSlip = false;
        else if (!mSlip && Math.abs(T) > Tslip_s) mSlip = true;
        if (mSlip) {
            T = Tslip_k * Math.signum(T);
            Tmotor = T / (mMotorRatio * eff);
            I = (Math.abs(Tmotor / km) + motorCharacteristics.getFreeCurrent(motorCount)) * Math.signum(Tmotor);
        }

        // acceleration calculation
        double F = T / mWheelRadius;
        double a = F / mRobotMass;
        mVelocity += a * loopTime;
        mDistance += mVelocity * loopTime + 0.5 * a * Math.pow(loopTime, 2); //v0 + 1/2at^2

        if (mStopMethod != StoppingMethods.REVERSE && mStopping) I = 0;

        double Vnew = 0;

        if (!mStopping) {
            Vnew = mRestingVoltage - mResistance * I;
        } else {
            switch (mStopMethod) {
                case COAST:
                    Vnew = 0;
                    mMotorEnabled = false;
                    break;
                case BRAKE:
                    Vnew = 0;
                    break;
                case REVERSE:
                    mVoltage = -(mRestingVoltage - mResistance * I);
                    break;
            }
        }
        mVoltage = (Math.abs(Vnew - mVoltage) > mMaxVoltageChange ? mVoltage + mMaxVoltageChange * Math.signum(Vnew - mVoltage) : Vnew * filtering + mVoltage * (1 - filtering));

        mLastRunTime += loopTime;
    }

    public double getStopDistance(double currentVelocity, double currentVoltage, StoppingMethods stoppingMethod) {
        resetSimulation();

        mVelocity = currentVelocity;
        mVoltage = currentVoltage;
        mStopMethod = stoppingMethod;
        mStopping = true;

        while(mVelocity > 0.05) {
            run();
        }

        return mDistance;
    }

    public void resetSimulation() {
        mDistance = 0;
        mVelocity = 0;
        mVoltage = mRestingVoltage;
        mStopping = false;
    }

}
