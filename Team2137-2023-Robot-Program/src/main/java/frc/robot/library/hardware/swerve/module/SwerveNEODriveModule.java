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
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.*;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.motor.NeoMotor;
import frc.robot.library.Constants.DriveControlType;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

//@SuppressWarnings("All")
public class SwerveNEODriveModule extends EntityGroup implements SwerveModule {

    private final NeoMotor mDriveMotor;
    private final NeoMotor mTurnMotor;

    private static final int intDriveVelocityPIDSlotID = 0;
    private static final int intDriveDistancePIDSlotID = 1;
    private final CANCoder mTurnEncoder;
    private Velocity mDriveVelocityGoal = new Velocity(0, FEET_PER_SECOND);
    private Distance mDriveDistanceGoal = new Distance(0, FOOT);
    private Rotation2d turningSetPoint;
    private DriveControlType mDriveControlType = DriveControlType.RAW;
    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveNEODriveModule(Element element, EntityGroup parent) {
        super(element, parent, true);

        mDriveMotor = (NeoMotor) getEntity("Drive Motor");
        mTurnMotor = (NeoMotor) getEntity("True Motor");
        Encoder encoder = (Encoder) getEntity("Turn Encoder");

        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());

        this.mTurnEncoder = new CANCoder(encoder.getID());
//        this.mTurnMotorEncoder = this.mTurnMotor.getEncoder();

        this.mTurnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); // -180 to 180
        this.mTurnEncoder.configMagnetOffset(encoder.getOffset());
    }

    /**
     * Periodic for NEO Type Module does not need anything in the periodic
     */
    @Override
    public void periodic() {
    }

    /**
     * Returns the current wheel angle on the swereve module using the motor encoder.
     * @return - current wheel angle
     */
    @Override
    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(this.mTurnEncoder.getAbsolutePosition());
    }

    @Override
    public Rotation2d getModuleGoalAngle() {
        return turningSetPoint;
    }

    /**
     * Sets the target wheel angle to feed the pid controller
     * @param angle - desired wheel angle (Degrees 0-360)
     */
    @Override
    public void setModuleAngle(Rotation2d angle) {
        turningSetPoint = angle;
        mTurnMotor.setPosition(new Angle(angle.getDegrees(), DEGREE));
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
    public void setVelocityDriveSpeed(Velocity speed) {
        mDriveVelocityGoal = speed;

        if (mDriveControlType != DriveControlType.VELOCITY) {
            configDrivetrainControlType(DriveControlType.VELOCITY);
        }

        mDriveMotor.setVelocity(speed);
//        this.mDrivePIDController.setReference(mDriveVelocityGoal.getValue(METER_PER_SECOND), CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public double getRawDrivePower() {
        return mDriveMotor.get();
    }

    @Override
    public double getCurrentDriveRPM() {
        return 0;
    }

    @Override
    public Velocity getDriveVelocity() {
//        return new Velocity(this.mDriveMotorEncoder.getVelocity(), );
        return new Velocity(0, FEET_PER_SECOND);
    }

    @Override
    public Velocity getDriveVelocityGoal() {
        return mDriveVelocityGoal;
    }

    //Distance getters and setters

    /**
     * Sets the distance target of the drive wheel and automatically sets the PID values
     * @param distance2d - Target distance for the wheel.
     */
    @Override
    public void setDriveDistanceTarget(Distance distance2d) {
        mDriveDistanceGoal = distance2d;

        if (mDriveControlType != DriveControlType.DISTANCE) {
            configDrivetrainControlType(DriveControlType.DISTANCE);
        }

        this.mDriveMotor.setIntegratedSensorPosition(0);
//        this.mDrivePIDController.setReference(mDriveDistanceGoal.getValue(FOOT), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public Distance getCurrentDrivePosition() {
//        return new Distance(this.mDriveMotor.getPosition(), INCH);
        return new Distance(0, INCH);
    }

    @Override
    public Distance getDriveDistanceTarget() {
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
//                pid = this.mDriveMotorObj.getPID(intDriveVelocityPIDSlotID);
                mDriveControlType = DriveControlType.VELOCITY;
                break;
            case DISTANCE:
//                pid = this.mDriveMotorObj.getPID(intDriveDistancePIDSlotID);
                mDriveControlType = DriveControlType.DISTANCE;
                break;
            default:
                pid = new PID(0.0, 0.0, 0.0, "NULL PID");
                mDriveControlType = DriveControlType.RAW;
                break;
        }

//        this.mDrivePIDController.setP(pid.getP());
//        this.mDrivePIDController.setI(pid.getI());
//        this.mDrivePIDController.setD(pid.getD());
    }

    @Override
    public DriveControlType getDriveControlType() {
        return mDriveControlType;
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

    @Override
    public SwerveModuleState getSwerveModuleAccelerationState(double voltage) {
        return null;
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }

    @Override
    public double getDriveMotorVoltage() {
        return 0;
    }
}
