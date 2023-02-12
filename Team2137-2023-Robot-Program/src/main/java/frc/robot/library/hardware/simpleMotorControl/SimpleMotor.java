package frc.robot.library.hardware.simpleMotorControl;

import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Distance;

public interface SimpleMotor {

    static SimpleMotor createMotor(Motor motor) {
        switch(motor.getMotorType()) {
            case NEO:
                return new SimpleNeo(motor);
            case FALCON:
                return new SimpleTalonFX(motor);
            default:
                return null;
        }
    }

    void set(double val);
    void setPosition(Distance distance);

    int getCountPerRevolution();

    Distance getDistancePerRevolution();
    void setDistancePerRevolution(Distance distance);

    void follow(SimpleMotor other);

    Motor getMotorObj();

    void setIntegratedSensorPosition(double val);
}
