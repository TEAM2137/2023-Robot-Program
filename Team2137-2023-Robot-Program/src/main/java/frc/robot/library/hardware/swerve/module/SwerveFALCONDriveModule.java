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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.*;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import frc.robot.library.units.Time2d;
import frc.robot.functions.io.xmlreader.data.Number;
import org.w3c.dom.Element;

public class SwerveFALCONDriveModule extends EntityGroup implements SwerveModule {

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;

    private final TalonFX mDriveMotor;
    private final TalonFX mTurnMotor;
    private final CANCoder mTurnCANEncoder;

    private SimpleMotorFeedforward mDriveMotorObjFeedForward;
    private Rotation2d goalModuleAngle;

    private Distance2d dblDriveWheelRotationPerFoot;//Rotations per Unit
    private Number dblWheelDiameter;

    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private final Motor mDriveMotorObj;
    private final Motor mTurnMotorObj;
    private final Encoder mTurnCANEncoderObj;
    private final FileLogger logger;
    private Constants.DriveControlType mDriveControlType = Constants.DriveControlType.UNDEFINED;
    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveFALCONDriveModule(Element element, int depth, boolean printprocess, FileLogger fileLogger) {
        super(element, depth, printprocess, fileLogger);

        //region Drive Motor Setup
        this.mDriveMotorObj = (Motor) getEntity("Drive Motor");

        this.mDriveMotor = new TalonFX(mDriveMotorObj.getID());
        this.mDriveMotor.configFactoryDefault();
        this.mDriveMotor.setInverted(mDriveMotorObj.inverted());
        this.mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, mDriveMotorObj.getCurrentLimit(), mDriveMotorObj.getCurrentLimit(), 1));
        this.mDriveMotor.setNeutralMode(NeutralMode.Brake);
        this.mDriveMotor.configOpenloopRamp(mDriveMotorObj.getRampRate());
        this.mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        //endregion


        //region Turn Motor Setup
        this.mTurnMotorObj = (Motor) getEntity("Turn Motor");

        this.mTurnMotor = new TalonFX(mTurnMotorObj.getID());
        this.mTurnMotor.configFactoryDefault();
        this.mTurnMotor.setInverted(mTurnMotorObj.inverted());
        this.mTurnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, mDriveMotorObj.getCurrentLimit(), mDriveMotorObj.getCurrentLimit(), 1));
        this.mTurnMotor.setNeutralMode(NeutralMode.Brake);
        this.mTurnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        PID tmpPID = mTurnMotorObj.getPID(0);
        this.mTurnMotor.config_kP(0, tmpPID.getP());
        this.mTurnMotor.config_kI(0, tmpPID.getI());
        this.mTurnMotor.config_kD(0, tmpPID.getD());
        //endregion


        //region Turn CAN Encoder Setup
        mTurnCANEncoderObj = (Encoder) getEntity("Turn Encoder");

        this.mTurnCANEncoder = new CANCoder(mTurnCANEncoderObj.getID());
        this.mTurnCANEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.mTurnCANEncoder.configMagnetOffset(mTurnCANEncoderObj.getOffset());
        //endregion


        //region Local Variable Declaration
        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());
        dblWheelDiameter = (Number) this.getEntity("DriveTrain-WheelDiameter");
        dblDriveWheelRotationPerFoot = Distance2d.fromFeet((Math.PI * (dblWheelDiameter.getValue() / 12.0)) / this.mDriveMotorObj.getGearRatio()); //Rotations per Foot (Moves PI/3 feet every rotation of wheel then divde by gear ratio
        //endregion


        //region Logging Setup
        logger = fileLogger;

        initializeSwerveDebug(logger, getCurrentNetworkInstance());

        logger.addScheduledDataLogger(8, "ModuleAngleIntegratedSensorPosition", getCurrentNetworkInstance(), (log, entry) -> {
            double currentIntegratedSensorPosition = this.mTurnMotor.getSelectedSensorPosition();

            log.writeEvent(8, getName() + "ModuleAngleIntegratedSensorPosition", String.valueOf(currentIntegratedSensorPosition));
            entry.setDouble(currentIntegratedSensorPosition);
        });
        //endregion

        //this.mDriveMotorObjFeedForward = mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
    }

    @Override
    public boolean onDestroy() throws Exception {
        this.mDriveMotor.DestroyObject();
        this.mTurnMotor.DestroyObject();
        this.mTurnCANEncoder.DestroyObject();

        return true;
    }

    @Override
    public void periodic() {
        logger.publishAllScheduled();
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

    /**
     * Assigns a new module goal angle to the PID
     *
     * @param angle - new angle goal for the wheel
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        goalModuleAngle = angle;
        Rotation2d changeInAngle = goalModuleAngle.minus(getModuleAngle());

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
        if (this.mDriveControlType != Constants.DriveControlType.RAW)
            this.mDriveControlType = Constants.DriveControlType.RAW;
        mDriveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * Set the goal velocity value to the PID Controller
     *
     * @param speed - Speed to set to the mDriveMotorObj train
     */
    @Override
    public void setVelocityDriveSpeed(Speed2d speed) {
        this.mDriveVelocityGoal = speed;

        if (this.mDriveControlType != Constants.DriveControlType.VELOCITY) {
            configDrivetrainControlType(Constants.DriveControlType.VELOCITY);
        }

        this.mDriveMotor.set(TalonFXControlMode.Velocity, speed.getCTREVelocityUnit(dblDriveWheelRotationPerFoot));//,
//                DemandType.ArbitraryFeedForward, mDriveMotorObjFeedForward.calculate(speed.getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS))); //In Ticks per 100ms and Meter per second
    }

    @Override
    public double getRawDrivePower() {
        return mDriveMotor.getMotorOutputPercent();
    }

    /**
     * RemTurnMotorObjs the instantaneous velocity of the wheel using integrated motor mTurnCANEncoder.
     *
     * @return - a Speed2d is returned
     */
    @Override
    public Speed2d getDriveVelocity() {
        return new Speed2d(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, (mDriveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048) * dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET));
    }

    /**
     * RemTurnMotorObjs the current velocity goal of the mDriveMotorObj train
     *
     * @return current velocity goal
     */
    @Override
    public Speed2d getDriveVelocityGoal() {
        return this.mDriveVelocityGoal;
    }

    /**
     * RemTurnMotorObj the current distance goal of the mDriveMotorObj train
     *
     * @return current distance goal
     */
    @Override
    public Distance2d getDriveDistanceTarget() {
        return this.mDriveDistanceGoal;
    }

    /**
     * RemTurnMotorObjs the current wheel position on the robot.
     *
     * @return Current mDriveMotorObj wheel mTurnCANEncoder position
     */
    @Override
    public Distance2d getCurrentDrivePosition() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, (this.mTurnCANEncoder.getPosition() / 2048) * dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET));
    }

    /**
     * Sets mDriveMotorObj train distance goal (Only mDriveMotorObj wheel)
     *
     * @param distance2d - Distance goal for robot mDriveMotorObj wheel
     */
    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        this.mDriveDistanceGoal = distance2d;

        if (this.mDriveControlType != Constants.DriveControlType.DISTANCE) {
            configDrivetrainControlType(Constants.DriveControlType.DISTANCE);
        }

        mDriveMotor.set(ControlMode.Position, distance2d.getValue(Distance2d.DistanceUnits.FEET) /  dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET) * 2048);
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
                PID tmpPIDVelocity = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID);
                this.mDriveMotor.config_kP(0, tmpPIDVelocity.getP());
                this.mDriveMotor.config_kI(0, tmpPIDVelocity.getI());
                this.mDriveMotor.config_kD(0, tmpPIDVelocity.getD());
                //TODO fix
                this.mDriveMotorObjFeedForward = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
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
        switch (mDriveControlType) {
            case DISTANCE:
                return new SwerveModuleState(getCurrentDrivePosition(), getModuleAngle(), mSwerveDrivePosition);
            case VELOCITY:
                return new SwerveModuleState(getDriveVelocity(), getModuleAngle(), mSwerveDrivePosition);
            default:
                return new SwerveModuleState(mDriveMotor.getMotorOutputPercent(), getModuleAngle(), mSwerveDrivePosition);
        }
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }

}
