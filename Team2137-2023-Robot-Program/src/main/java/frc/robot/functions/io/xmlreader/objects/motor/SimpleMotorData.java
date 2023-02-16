package frc.robot.functions.io.xmlreader.objects.motor;

import frc.robot.functions.io.xmlreader.data.PID;

public interface SimpleMotorData {

    public double getGearRatio();
    public void setGearRatio(double ratio);

    public Motor.MotorTypes getType();
    public void setMotorType(Motor.MotorTypes _type);

    public boolean inverted();
    public void setInverted(boolean value);

    public int getCurrentLimit();
    public void setCurrentLimit(int limit);

    public PID getPID(int slotID);
    public void setPID(int soltID, PID pid);

    public PID getPID();
    public void setPID(PID pid);

    public double getRampRate();
    public void setRampRate(double _rampRate);

    public int getID();
    public void setID(int _id);
}
