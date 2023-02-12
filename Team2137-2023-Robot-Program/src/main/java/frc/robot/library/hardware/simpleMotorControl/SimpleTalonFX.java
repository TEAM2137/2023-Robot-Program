package frc.robot.library.hardware.simpleMotorControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.units.TranslationalUnits.Distance;

public class SimpleTalonFX implements SimpleMotor {

    private Motor motorObj;
    private TalonFX talonFX;

    private Distance distancePerRevolution;

    private static final int CountsPerRevolution = 2048;

    public SimpleTalonFX(Motor motor) {
        talonFX = new TalonFX(motor.getID());

        init();
        motorObj.setOnImplementCallback(this::init);
    }

    private void init() {
        talonFX.configFactoryDefault();
        talonFX.setInverted(motorObj.inverted());
        talonFX.configClosedloopRamp(motorObj.getRampRate());

        PID pid = motorObj.getPID();
        talonFX.config_kP(0, pid.getP());
        talonFX.config_kI(0, pid.getI());
        talonFX.config_kD(0, pid.getD());
        talonFX.config_kF(0, pid.getFF());
    }

    @Override
    public void set(double val) {
        talonFX.set(ControlMode.PercentOutput, val);
    }

    @Override
    public void setPosition(Distance distance) {
        talonFX.set(TalonFXControlMode.Position, (distance.getValue(Distance.DistanceUnits.INCH) / distancePerRevolution.getValue(Distance.DistanceUnits.INCH)) * getCountPerRevolution());
    }

    @Override
    public int getCountPerRevolution() {
        return CountsPerRevolution;
    }

    @Override
    public Distance getDistancePerRevolution() {
        return distancePerRevolution;
    }

    @Override
    public void setDistancePerRevolution(Distance distance) {
        distancePerRevolution = distance;

    }

    @Override
    public Motor getMotorObj() {
        return motorObj;
    }

    @Override
    public void setIntegratedSensorPosition(double val) {
        talonFX.setSelectedSensorPosition(val);
    }

    @Override
    public void follow(SimpleMotor other) {
        if(other instanceof SimpleTalonFX) {
            talonFX.follow(((SimpleTalonFX) other).talonFX);
        }
    }
}
