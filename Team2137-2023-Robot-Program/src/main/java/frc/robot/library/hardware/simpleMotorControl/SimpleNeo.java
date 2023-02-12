package frc.robot.library.hardware.simpleMotorControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.units.TranslationalUnits.Distance;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class SimpleNeo implements SimpleMotor {
    private Motor motorObj;
    private CANSparkMax spark;
    private RelativeEncoder relativeEncoder;
    private SparkMaxPIDController pidController;

    private Distance distancePerRevolution;

    private static final int CountsPerRevolution = 2048;

    public SimpleNeo(Motor motor) {
        spark = new CANSparkMax(motor.getID(), motor.getMotorType().getREVType());

        init();
        motorObj.setOnImplementCallback(this::init);
    }

    private void init() {
        spark.restoreFactoryDefaults();
        spark.setInverted(motorObj.inverted());
        spark.setSmartCurrentLimit(motorObj.getCurrentLimit());
        spark.setClosedLoopRampRate(motorObj.getRampRate());

        relativeEncoder = spark.getEncoder();

        pidController = spark.getPIDController();

        PID pid = motorObj.getPID();
        pidController.setP(pid.getP());
        pidController.setI(pid.getI());
        pidController.setD(pid.getD());
        pidController.setFF(pid.getFF());
    }

    @Override
    public void set(double val) {
        spark.set(val);
    }

    @Override
    public void setPosition(Distance distance) {
        pidController.setReference((distance.getValue(INCH) / distancePerRevolution.getValue(INCH)) * getCountPerRevolution(), CANSparkMax.ControlType.kPosition);
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
        relativeEncoder.setPosition(val);
    }

    @Override
    public void follow(SimpleMotor other) {
        if(other instanceof SimpleNeo) {
            spark.follow(((SimpleNeo) other).spark);
        } else if (other instanceof SimpleTalonFX){
            spark.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, other.getMotorObj().getID());
        }
    }
}
