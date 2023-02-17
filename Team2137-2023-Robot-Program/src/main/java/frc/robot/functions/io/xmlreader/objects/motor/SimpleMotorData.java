package frc.robot.functions.io.xmlreader.objects.motor;

import frc.robot.functions.io.xmlreader.data.PID;

public interface SimpleMotorData {

    double getGearRatio();
    void setGearRatio(double ratio);

    Motor.MotorTypes getType();
    void setMotorType(Motor.MotorTypes _type);

    boolean inverted();
    void setInverted(boolean value);

    int getCurrentLimit();
    void setCurrentLimit(int limit);

    PID getPID(int slotID);
    void setPID(int soltID, PID pid);

    PID getPID();
    void setPID(PID pid);

    double getRampRate();
    void setRampRate(double _rampRate);

    int getID();
    void setID(int _id);
}
