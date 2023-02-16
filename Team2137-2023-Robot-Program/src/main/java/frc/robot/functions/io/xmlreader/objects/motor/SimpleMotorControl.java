package frc.robot.functions.io.xmlreader.objects.motor;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

public interface SimpleMotorControl extends SimpleMotorData {

    void set(double val);
    void setPosition(Distance distance);
    void setPosition(Angle angle);

    void setVelocity(Velocity velocity);

    int getCountPerRevolution();

    Distance getDistancePerRevolution();
    void setDistancePerRevolution(Distance distance);

    void follow(SimpleMotorControl other);

    void setIntegratedSensorPosition(double val);

    int getID();

    void setNeutralMode(CANSparkMax.IdleMode mode);
}
