package frc.robot.library.hardware;

import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;

public class FalconCharacteristics {

    public static final double StallCurrent = 257;
    public static final double FreeCurrent = 1.50;
    public static final double FreeSpeed = 6380;
    public static final double StallTorque = 4.69;

    public static double getTorqueCurveSlopeFromVoltage(double voltage) { return -4.31E-6 * Math.pow(voltage, 2) + 1.06E-4 * voltage - 1.38E-3; }
    public static double getTorqueCurveInterceptFromVoltage(double voltage) { return 0.0156 * Math.pow(voltage, 2) + 0.0193 * voltage + 2.19; }

    public static double getTorque(double rpm, double voltage) {
        return getTorqueCurveSlopeFromVoltage(voltage) * rpm + getTorqueCurveInterceptFromVoltage(voltage);
    }

    public static double getForce(double rpm, double voltage, double r) {
        return getTorque(rpm, voltage) * r;
    }

    public static Acceleration getAcceleration(double rpm, double voltage, double r, double mass) {
        return new Acceleration(getForce(rpm, voltage, r) / mass, METER_PER_SECOND2);
    }
}
