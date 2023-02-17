//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package frc.robot.library.hardware.swerve.module;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.library.hardware.DriveTrainSimulation;
import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Number;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.library.Constants;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;

import static frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.*;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.CTRE_VELOCITY;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

public class SwerveFALCONDriveModule extends EntityGroup implements SwerveModule {

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;

    private final TalonFX mDriveMotor;
    private final TalonFX mTurnMotor;
    private final CANCoder mTurnCANEncoder;

    private SimpleMotorFeedforward mDriveMotorObjFeedForward;
    private Rotation2d goalModuleAngle = Rotation2d.fromDegrees(0);

    private Distance dblDriveWheelRotationPerFoot;//Rotations per Unit
    private final Distance dblWheelDiameter;
    private final Number dblRobotMass;
    private final Number dblScaleSpeedOptimization;

    private Velocity mDriveVelocityGoal = new Velocity(0, FEET_PER_SECOND);
    private Distance mDriveDistanceGoal = new Distance(0, FOOT);
    private double mDriveRawGoal = 0;
    private final Motor mDriveMotorObj;
    private final Motor mTurnMotorObj;
    private final Encoder mTurnCANEncoderObj;
    private final FileLogger logger;
    private Constants.DriveControlType mDriveControlType = Constants.DriveControlType.VELOCITY;
    private SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveFALCONDriveModule(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        //region Logging Setup
        logger = fileLogger;
        logger.setTag("SwerveFALCONDriveModule");
        //endregion

        this.mDriveMotorObj = (Motor) getEntity("Drive Motor");
        this.mDriveMotor = new TalonFX(mDriveMotorObj.getID());

        this.mTurnMotorObj = (Motor) getEntity("Turn Motor");
        this.mTurnMotor = new TalonFX(mTurnMotorObj.getID());

        this.mTurnCANEncoderObj = (Encoder) getEntity("Turn Encoder");
        this.mTurnCANEncoder = new CANCoder(mTurnCANEncoderObj.getID());

        Number dia = (Number) Robot.settingsEntityGroup.getEntity("DriveTrain-WheelDiameter");
        dblWheelDiameter = new Distance(dia.getValue(), INCH);
        dblRobotMass = new Number(60);
        dblScaleSpeedOptimization = (Number) Robot.settingsEntityGroup.getEntity("DriveTrain-ScaleSpeedOptimization");

        initialize();
        this.setOnImplementCallback(this::initialize);

        //this.mDriveMotorObjFeedForward = mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
    }

    private void initialize() {
        logger.writeEvent(4, FileLogger.EventType.Debug, "Initializing Falcon Drive Motor...");
        this.mDriveMotor.configFactoryDefault();
        this.mDriveMotor.setInverted(mDriveMotorObj.inverted());
        this.mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, mDriveMotorObj.getCurrentLimit(), mDriveMotorObj.getCurrentLimit(), 1));
        this.mDriveMotor.setNeutralMode(NeutralMode.Brake);
        this.mDriveMotor.configOpenloopRamp(mDriveMotorObj.getRampRate());
        this.mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        logger.writeEvent(4, FileLogger.EventType.Debug, "Initializing Falcon Turn Motor...");
        this.mTurnMotor.configFactoryDefault();
        this.mTurnMotor.setInverted(mTurnMotorObj.inverted());
        this.mTurnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, mDriveMotorObj.getCurrentLimit(), mDriveMotorObj.getCurrentLimit(), 1));
        this.mTurnMotor.setNeutralMode(NeutralMode.Brake);
        this.mTurnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        //this.mTurnMotor.configSelectedFeedbackCoefficient(1);
        this.mTurnMotor.setSelectedSensorPosition(0);

        PID tmpPID = mTurnMotorObj.getPID(0);
        logger.writeEvent(4, FileLogger.EventType.Debug, "Configuring Turn Motor PID values...");
        this.mTurnMotor.config_kP(0, tmpPID.getP());
        this.mTurnMotor.config_kI(0, tmpPID.getI());
        this.mTurnMotor.config_kD(0, tmpPID.getD());
        //tmpPID.addToLogger(logger, getSwerveModuleLocation().toString() + " Turn Motor");

        logger.writeEvent(4, FileLogger.EventType.Debug, "Initializing Turning CAN Encoder...");
        this.mTurnCANEncoder.configFactoryDefault();
        this.mTurnCANEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        this.mTurnCANEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.mTurnCANEncoder.configMagnetOffset(mTurnCANEncoderObj.getOffset());

        logger.writeEvent(4, FileLogger.EventType.Debug, "Getting Setting Variables from file...");
        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());
        dblDriveWheelRotationPerFoot = new Distance((Math.PI * dblWheelDiameter.getValue(FOOT)) / this.mDriveMotorObj.getGearRatio(), FOOT); //Rotations per Foot (Moves PI/3 feet every rotation of wheel then divde by gear ratio
    }

    @Override
    public void periodic() {
        NetworkTableInstance table = NetworkTableInstance.getDefault();

        table.getEntry(getEntityPath() + "Speed").setDouble(getRawDrivePower());
        table.getEntry(getEntityPath() + "AngleTarget").setDouble(goalModuleAngle.getDegrees());
        table.getEntry(getEntityPath() + "Angle").setDouble(getModuleAngle().getDegrees());
        table.getEntry(getEntityPath() + "RawAngleCounts").setDouble(this.mTurnMotor.getSelectedSensorPosition());

        if(dblScaleSpeedOptimization != null && dblScaleSpeedOptimization.getValue() > 0) {
            Rotation2d deltaAngle = goalModuleAngle.minus(getModuleAngle());
            setRawDriveSpeed(mDriveRawGoal * Math.abs(Math.pow(Math.cos(deltaAngle.getRadians()), (int) dblScaleSpeedOptimization.getValue())));
        }
    }

    /**
     * getModuleAngle() takes the value from the CANCoder
     *
     * @return Wheel angle on module
     */
    @Override
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(mTurnCANEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getModuleGoalAngle() {
        return goalModuleAngle;
    }

    /**
     * Assigns a new module goal angle to the PID
     *
     * @param angle - new angle goal for the wheel
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        goalModuleAngle = angle;
        Rotation2d current = getModuleAngle();
//        Rotation2d changeInAngle = goalModuleAngle.minus(optimizeSwerveModuleAngle(goalModuleAngle, current));
        Rotation2d changeInAngle = goalModuleAngle.minus(current);

        //(Amount of revolution) * (Motor Rev Per Wheel Rev) * (Count Per Motor Rev)
        double changeInCount = (changeInAngle.getRadians() / (Math.PI * 2)) * mTurnMotorObj.getGearRatio() * 2048;

        this.mTurnMotor.set(ControlMode.Position, this.mTurnMotor.getSelectedSensorPosition() + changeInCount);
    }


    /**
     * Sets the raw speed -1~1 to motor and disables Velocity and Distance
     *
     * @param speed - raw speed value to set
     */
    @Override
    public void setRawDriveSpeed(double speed) {
        mDriveRawGoal = speed;

        if (this.mDriveControlType != Constants.DriveControlType.RAW)
            configDrivetrainControlType(Constants.DriveControlType.RAW);

        mDriveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * Set the goal velocity value to the PID Controller
     *
     * @param speed - Speed to set to the mDriveMotorObj train
     */
    @Override
    public void setVelocityDriveSpeed(Velocity speed) {
        this.mDriveVelocityGoal = speed;

        if (this.mDriveControlType != Constants.DriveControlType.VELOCITY) {
            configDrivetrainControlType(Constants.DriveControlType.VELOCITY);
        }

        this.mDriveMotor.set(TalonFXControlMode.Velocity, speed.getCTREVelocityUnit(dblDriveWheelRotationPerFoot).getValue(CTRE_VELOCITY));//,
//                DemandType.ArbitraryFeedForward, mDriveMotorObjFeedForward.calculate(speed.getValue(Distance.DistanceUnits.METER, Time2d.TimeUnits.SECONDS))); //In Ticks per 100ms and Meter per second
    }

    @Override
    public double getRawDrivePower() {
        return mDriveMotor.getMotorOutputPercent();
    }

    @Override
    public double getCurrentDriveRPM() {
        return (mDriveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048) / 60.0;
    }

    @Override
    public SwerveModuleState getSwerveModuleAccelerationState(double voltage) {
        Acceleration accel = DriveTrainSimulation.getAcceleration(getCurrentDriveRPM(), voltage, dblWheelDiameter.getValue(METER) / 2, dblRobotMass.getValue() / 4.0);

        return new SwerveModuleState(accel, new AngularAcceleration(0, RADIAN_PER_SECOND2), mSwerveDrivePosition);
    }

    /**
     * RemTurnMotorObjs the instantaneous velocity of the wheel using integrated motor mTurnCANEncoder.
     *
     * @return - a Velocity is returned
     */
    @Override
    public Velocity getDriveVelocity() {
        return new Velocity((mDriveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048) * dblDriveWheelRotationPerFoot.getValue(FOOT), FEET_PER_SECOND);
    }

    /**
     * RemTurnMotorObjs the current velocity goal of the mDriveMotorObj train
     *
     * @return current velocity goal
     */
    @Override
    public Velocity getDriveVelocityGoal() {
        return this.mDriveVelocityGoal;
    }

    /**
     * RemTurnMotorObj the current distance goal of the mDriveMotorObj train
     *
     * @return current distance goal
     */
    @Override
    public Distance getDriveDistanceTarget() {
        return this.mDriveDistanceGoal;
    }

    /**
     * RemTurnMotorObjs the current wheel position on the robot.
     *
     * @return Current mDriveMotorObj wheel mTurnCANEncoder position
     */
    @Override
    public Distance getCurrentDrivePosition() {
        return new Distance((this.mTurnCANEncoder.getPosition() / 2048) * dblDriveWheelRotationPerFoot.getValue(FOOT), INCH);
    }

    /**
     * Sets mDriveMotorObj train distance goal (Only mDriveMotorObj wheel)
     *
     * @param distance2d - Distance goal for robot mDriveMotorObj wheel
     */
    @Override
    public void setDriveDistanceTarget(Distance distance2d) {
        this.mDriveDistanceGoal = distance2d;

        if (this.mDriveControlType != Constants.DriveControlType.DISTANCE) {
            configDrivetrainControlType(Constants.DriveControlType.DISTANCE);
        }

        mDriveMotor.set(ControlMode.Position, distance2d.getValue(FOOT) / dblDriveWheelRotationPerFoot.getValue(FOOT) * 2048);
    }

    /**
     * Dictates weather the mDriveMotorObj motor is to power stop
     *
     * @param brake - True if braking is enabled
     */
    @Override
    public void setDriveCoastMode(boolean brake) {
        mDriveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * Usage is ONLY for time saving pre enable (Note change is automatic when set velocity or distance is called)
     *
     * @param control - Desired control type to be used
     */
    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        switch (control) {
            case VELOCITY:
//                PID tmpPIDVelocity = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID);
//                this.mDriveMotor.config_kP(0, tmpPIDVelocity.getP());
//                this.mDriveMotor.config_kI(0, tmpPIDVelocity.getI());
//                this.mDriveMotor.config_kD(0, tmpPIDVelocity.getD());
                //TODO fix
//                this.mDriveMotorObjFeedForward = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
                mDriveControlType = Constants.DriveControlType.VELOCITY;
                break;
            case DISTANCE:
                PID tmpPIDDistance = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID);
                this.mDriveMotor.config_kP(0, tmpPIDDistance.getP());
                this.mDriveMotor.config_kI(0, tmpPIDDistance.getI());
                this.mDriveMotor.config_kD(0, tmpPIDDistance.getD());
                //TODO fix
                this.mDriveMotorObjFeedForward = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).getWPIFeedForwardController();
                mDriveControlType = Constants.DriveControlType.DISTANCE;
                break;
            case RAW:
                mDriveControlType = Constants.DriveControlType.RAW;
                break;
        }
    }

    @Override
    public Constants.DriveControlType getDriveControlType() {
        return mDriveControlType;
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), getModuleAngle(), mSwerveDrivePosition);
//        switch (mDriveControlType) {
//            case DISTANCE:
//                return new SwerveModuleState(getCurrentDrivePosition(), getModuleAngle(), mSwerveDrivePosition);
//            case VELOCITY:
//                return new SwerveModuleState(getDriveVelocity(), getModuleAngle(), mSwerveDrivePosition);
//            default:
//                return new SwerveModuleState(mDriveMotor.getMotorOutputPercent(), getModuleAngle(), mSwerveDrivePosition);
//        }
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }

    @Override
    public double getDriveMotorVoltage() {
        return mDriveMotor.getMotorOutputVoltage();
    }

}
