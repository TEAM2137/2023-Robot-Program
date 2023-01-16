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
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANCoder encoder;

    private SimpleMotorFeedforward driveFeedForward;
    private Rotation2d goalModuleAngle;

    private Distance2d dblDriveWheelRotationPerFoot;//Rotations per Unit
    private Number dblWheelDiameter;

    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private final Motor mDriveMotorObj;
    private final Motor mTurnMotorObj;
    private FileLogger logger;
    private Constants.DriveControlType mDriveControlType = Constants.DriveControlType.UNDEFINED;
    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveFALCONDriveModule(Element element, int depth, boolean printprocess, FileLogger fileLogger) {
        super(element, depth, printprocess, fileLogger);

        Motor drive = (Motor) getEntity("Drive Motor");
        Motor turn = (Motor) getEntity("Turn Motor");
        Encoder encoder = (Encoder) getEntity("Turn Encoder");

        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());

        this.mDriveMotorObj = drive;

        this.driveMotor = new TalonFX(drive.getID());
        this.driveMotor.configFactoryDefault();
        this.driveMotor.setInverted(drive.inverted());
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, drive.getCurrentLimit(), drive.getCurrentLimit(), 1));
        this.driveMotor.setNeutralMode(NeutralMode.Brake);
        this.driveMotor.configOpenloopRamp(drive.getRampRate());
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        dblWheelDiameter = (Number) this.getEntity("DriveTrain-WheelDiameter");
        dblDriveWheelRotationPerFoot = Distance2d.fromFeet((Math.PI * (dblWheelDiameter.getValue() / 12.0)) / this.mDriveMotorObj.getGearRatio()); //Rotations per Foot (Moves PI/3 feet every rotation of wheel then divde by gear ratio

        this.mTurnMotorObj = turn;

        // Turning motor setup
        this.turningMotor = new TalonFX(turn.getID());
        this.turningMotor.configFactoryDefault();
        this.turningMotor.setInverted(turn.inverted());
        this.turningMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, drive.getCurrentLimit(), drive.getCurrentLimit(), 1));
        this.turningMotor.setNeutralMode(NeutralMode.Brake);
        this.turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

        PID tmpPID = turn.getPID(0);
        this.turningMotor.config_kP(0, tmpPID.getP());
        this.turningMotor.config_kI(0, tmpPID.getI());
        this.turningMotor.config_kD(0, tmpPID.getD());

        // Encoder setup
        this.encoder = new CANCoder(encoder.getID());
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.encoder.configMagnetOffset(encoder.getOffset());

        logger = fileLogger;

        initializeSwerveDebug(logger, getCurrentNetworkInstance());

        logger.addScheduledDataLogger(8, "ModuleAngleIntegratedSensorPosition", getCurrentNetworkInstance(), (log, entry) -> {
            double currentIntegratedSensorPosition = this.turningMotor.getSelectedSensorPosition();

            log.writeEvent(8, getName() + "ModuleAngleIntegratedSensorPosition", String.valueOf(currentIntegratedSensorPosition));
            entry.setDouble(currentIntegratedSensorPosition);
        });

        //this.driveFeedForward = drive.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
    }

    @Override
    public boolean onDestroy() throws Exception {
        this.driveMotor.DestroyObject();
        this.turningMotor.DestroyObject();
        this.encoder.DestroyObject();

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
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
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

        this.turningMotor.set(ControlMode.Position, this.turningMotor.getSelectedSensorPosition() + changeInCount);
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
        driveMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    /**
     * Set the goal velocity value to the PID Controller
     *
     * @param speed - Speed to set to the drive train
     */
    @Override
    public void setVelocityDriveSpeed(Speed2d speed) {
        this.mDriveVelocityGoal = speed;

        if (this.mDriveControlType != Constants.DriveControlType.VELOCITY) {
            configDrivetrainControlType(Constants.DriveControlType.VELOCITY);
        }

        this.driveMotor.set(TalonFXControlMode.Velocity, speed.getCTREVelocityUnit(dblDriveWheelRotationPerFoot));//,
//                DemandType.ArbitraryFeedForward, driveFeedForward.calculate(speed.getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS))); //In Ticks per 100ms and Meter per second
    }

    @Override
    public double getRawDrivePower() {
        return driveMotor.getMotorOutputPercent();
    }

    /**
     * Returns the instantaneous velocity of the wheel using integrated motor encoder.
     *
     * @return - a Speed2d is returned
     */
    @Override
    public Speed2d getDriveVelocity() {
        return new Speed2d(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS, (driveMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048) * dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET));
    }

    /**
     * Returns the current velocity goal of the drive train
     *
     * @return current velocity goal
     */
    @Override
    public Speed2d getDriveVelocityGoal() {
        return this.mDriveVelocityGoal;
    }

    /**
     * Return the current distance goal of the drive train
     *
     * @return current distance goal
     */
    @Override
    public Distance2d getDriveDistanceTarget() {
        return this.mDriveDistanceGoal;
    }

    /**
     * Returns the current wheel position on the robot.
     *
     * @return Current drive wheel encoder position
     */
    @Override
    public Distance2d getCurrentDrivePosition() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, (this.encoder.getPosition() / 2048) * dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET));
    }

    /**
     * Sets drive train distance goal (Only drive wheel)
     *
     * @param distance2d - Distance goal for robot drive wheel
     */
    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        this.mDriveDistanceGoal = distance2d;

        if (this.mDriveControlType != Constants.DriveControlType.DISTANCE) {
            configDrivetrainControlType(Constants.DriveControlType.DISTANCE);
        }

        driveMotor.set(ControlMode.Position, distance2d.getValue(Distance2d.DistanceUnits.FEET) /  dblDriveWheelRotationPerFoot.getValue(Distance2d.DistanceUnits.FEET) * 2048);
    }

    /**
     * Dictates weather the drive motor is to power stop
     *
     * @param brake - True if braking is enabled
     */
    @Override
    public void setDriveCoastMode(boolean brake) {
        driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
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
                this.driveMotor.config_kP(0, tmpPIDVelocity.getP());
                this.driveMotor.config_kI(0, tmpPIDVelocity.getI());
                this.driveMotor.config_kD(0, tmpPIDVelocity.getD());
                //TODO fix
                this.driveFeedForward = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID).getWPIFeedForwardController();
                mDriveControlType = Constants.DriveControlType.VELOCITY;
                break;
            case DISTANCE:
                PID tmpPIDDistance = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID);
                this.driveMotor.config_kP(0, tmpPIDDistance.getP());
                this.driveMotor.config_kI(0, tmpPIDDistance.getI());
                this.driveMotor.config_kD(0, tmpPIDDistance.getD());
                //TODO fix
                this.driveFeedForward = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID).getWPIFeedForwardController();
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
                return new SwerveModuleState(driveMotor.getMotorOutputPercent(), getModuleAngle(), mSwerveDrivePosition);
        }
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }

}
