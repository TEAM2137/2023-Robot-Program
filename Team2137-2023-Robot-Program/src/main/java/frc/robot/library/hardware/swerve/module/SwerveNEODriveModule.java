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

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.*;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.Constants.DriveControlType;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import frc.robot.library.units.Time2d;
import org.w3c.dom.Element;

//@SuppressWarnings("All")
public class SwerveNEODriveModule extends EntityGroup implements SwerveModule {

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;
    public CANSparkMax mDriveMotor;
    public CANSparkMax mTurnMotor;
    private RelativeEncoder mTurnMotorEncoder;
    private RelativeEncoder mDriveMotorEncoder;
    private CANCoder mTurnEncoder;
    private SparkMaxPIDController mDrivePIDController;
    private PIDController mTurnPIDController;
    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private Rotation2d turningSetPoint;
    private Motor mDriveMotorObj;
    private DriveControlType mDriveControlType = DriveControlType.RAW;
    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveNEODriveModule(Element element, int depth, boolean printprocess, FileLogger fileLogger) {
        super(element, depth, printprocess, fileLogger);

        Motor drive = (Motor) getEntity("Drive Motor");
        Motor turn = (Motor) getEntity("True Motor");
        Encoder encoder = (Encoder) getEntity("Turn Encoder");

        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());

        this.mDriveMotorObj = drive;

        this.mDriveMotor = new CANSparkMax(drive.getID(), drive.getMotorType().getREVType());
        this.mDriveMotor.restoreFactoryDefaults();
        this.mDriveMotor.setInverted(drive.inverted());
        this.mDriveMotor.setSmartCurrentLimit(drive.getCurrentLimit());
        this.mDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.mDriveMotorEncoder = this.mDriveMotor.getEncoder();

        //Inches per Rotation.
        //For MK3 Swerve (3.1415 * 4")/8.16
        this.mDriveMotorEncoder.setPositionConversionFactor(drive.getGearRatio());

        this.mDrivePIDController = this.mDriveMotor.getPIDController();

        this.mDrivePIDController.setSmartMotionAllowedClosedLoopError(1, 0);
        this.mDrivePIDController.setSmartMotionMaxVelocity(2000, 0);
        this.mDrivePIDController.setSmartMotionMaxAccel(1500, 0);

        this.mTurnMotor = new CANSparkMax(turn.getID(), turn.getMotorType().getREVType());
        this.mTurnMotor.restoreFactoryDefaults();
        this.mTurnMotor.setInverted(turn.inverted());
        this.mTurnMotor.setSmartCurrentLimit(turn.getCurrentLimit());
        this.mTurnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.mTurnEncoder = new CANCoder(encoder.getID());
        this.mTurnMotorEncoder = this.mTurnMotor.getEncoder();

        this.mTurnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.mTurnEncoder.configMagnetOffset(encoder.getOffset());

        this.mTurnPIDController = new PIDController(
                turn.getPID().getP(),
                turn.getPID().getI(),
                turn.getPID().getD());

        this.mTurnPIDController.enableContinuousInput(-180, 180); // allows module to wrap properly

//        this.mTurnMotorEncoder.setPosition(this.mTurnEncoder.getAbsolutePosition() / 360);
//        this.mTurnMotorEncoder.setPositionConversionFactor((1 / turn.getGearRatio()) / 360);

//        this.mTurnPIDController.setSmartMotionAllowedClosedLoopError(1, 0);
//        this.mTurnPIDController.setSmartMotionMaxVelocity(2000, 0);
//        this.mTurnPIDController.setSmartMotionMaxAccel(1500, 0);

    }

    @Override
    public boolean onDestroy() throws Exception {
        this.mDriveMotor.close();
        this.mTurnMotor.close();
        this.mTurnEncoder.DestroyObject();

        return true;
    }

    /**
     * Periodic for NEO Type Module does not need anything in the periodic
     */
    @Override
    public void periodic() {
        this.mTurnMotor.set(this.mTurnPIDController.calculate(getModuleAngle().getDegrees()));
    }

    /**
     * Returns the current wheel angle on the swereve module using the motor encoder.
     * @return - current wheel angle
     */
    @Override
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(this.mTurnEncoder.getAbsolutePosition());
    }

    /**
     * Sets the target wheel angle to feed the pid controller
     * @param angle - desired wheel angle (Degrees 0-360)
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        turningSetPoint = angle;
        this.mTurnPIDController.setSetpoint(turningSetPoint.getDegrees());
    }

    /**
     * Sets the raw wheel speed -1~1
     * @param speed - desired raw power
     */
    @Override
    public void setRawDriveSpeed(double speed) {
        this.mDriveMotor.set(speed);
    }

    //Velocity getters and setters

    @Override
    public void setVelocityDriveSpeed(Speed2d speed) {
        mDriveVelocityGoal = speed;

        if (mDriveControlType != DriveControlType.VELOCITY) {
            configDrivetrainControlType(DriveControlType.VELOCITY);
        }

        this.mDrivePIDController.setReference(mDriveVelocityGoal.getValue(Distance2d.DistanceUnits.METER, Time2d.TimeUnits.SECONDS), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public double getRawDrivePower() {
        return mDriveMotor.get();
    }

    @Override
    public Speed2d getDriveVelocity() {
        return new Speed2d(Distance2d.DistanceUnits.INCH, Time2d.TimeUnits.MINUTES, this.mDriveMotorEncoder.getVelocity());
    }

    @Override
    public Speed2d getDriveVelocityGoal() {
        return mDriveVelocityGoal;
    }

    //Distance getters and setters

    /**
     * Sets the distance target of the drive wheel and automatically sets the PID values
     * @param distance2d - Target distance for the wheel.
     */
    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        mDriveDistanceGoal = distance2d;

        if (mDriveControlType != DriveControlType.DISTANCE) {
            configDrivetrainControlType(DriveControlType.DISTANCE);
        }

        this.mDriveMotorEncoder.setPosition(0);
        this.mDrivePIDController.setReference(mDriveDistanceGoal.getValue(Distance2d.DistanceUnits.INCH), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public Distance2d getCurrentDrivePosition() {
        return Distance2d.fromUnit(Distance2d.DistanceUnits.INCH, this.mDriveMotorEncoder.getPosition());
    }

    @Override
    public Distance2d getDriveDistanceTarget() {
        return mDriveDistanceGoal;
    }

    @Override
    public void setDriveCoastMode(boolean brake) {
        this.mDriveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    /**
     * Usage is ONLY for time saving pre enable (Note change is automatic when set velocity or distance is called)
     *
     * @param control
     */
    @Override
    public void configDrivetrainControlType(DriveControlType control) {
        PID pid;
        switch (control) {
            case VELOCITY:
                pid = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID);
                mDriveControlType = DriveControlType.VELOCITY;
                break;
            case DISTANCE:
                pid = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID);
                mDriveControlType = DriveControlType.DISTANCE;
                break;
            default:
                pid = new PID(0.0, 0.0, 0.0, "NULL PID");
                mDriveControlType = DriveControlType.RAW;
                break;
        }

        this.mDrivePIDController.setP(pid.getP());
        this.mDrivePIDController.setI(pid.getI());
        this.mDrivePIDController.setD(pid.getD());
    }

    public SwerveModuleState getSwerveModuleState() {
        switch (mDriveControlType) {
            case DISTANCE:
                return new SwerveModuleState(getCurrentDrivePosition(), getModuleAngle(), mSwerveDrivePosition);
            case VELOCITY:
                return new SwerveModuleState(getDriveVelocity(), getModuleAngle(), mSwerveDrivePosition);
            default:
                return new SwerveModuleState(mDriveMotor.get(), getModuleAngle(), mSwerveDrivePosition);
        }
    }
}
