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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.library.Constants;
import frc.robot.library.units.TranslationalUnits.Velocity;
import org.w3c.dom.Element;

import static frc.robot.library.Constants.DriveControlType.*;
import static frc.robot.library.units.AngleUnits.AngularAcceleration.AngularAccelerationUnits.RADIAN_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.*;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

public class SwerveSimulationDriveModule extends EntityGroup implements SwerveModule {

    public Motor mDriveMotor;
    public Motor mTurnMotor;
    private Encoder mTurnEncoder;
    private PIDController mDrivePIDController;
    private PIDController mTurnPIDController;

    private Velocity mDriveVelocityGoal = new Velocity(0, FEET_PER_SECOND);
    private Distance mDriveDistanceGoal = new Distance(0, FOOT);
    private double mDriveRawGoal = 0;
    private Rotation2d turningSetPoint = Rotation2d.fromDegrees(0);

    private Acceleration mDriveAccelerationCurrent = new Acceleration(0, METER_PER_SECOND2);
    private Velocity mDriveVelocityCurrent = new Velocity(0, FEET_PER_SECOND);
    private Distance mDriveDistanceCurrent = new Distance(0, FOOT);
    private final double mDriveRawPercent = 0;
    private double mCurrentDriveRPM = 0;
    private Rotation2d turningCurrent = Rotation2d.fromDegrees(0);
    private final Distance dblWheelDiameter = new Distance(4, INCH);

    private Constants.DriveControlType mDriveControlType = RAW;
    //private SwerveModuleState.SwerveModulePositions swerveModulePosition;

    private final FileLogger logger;

    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveSimulationDriveModule(Element element, EntityGroup parent) {
        super(element, parent, false);

        logger = getLogger();
        logger.writeEvent(0, FileLogger.EventType.Error, "Simulated SwerveModuleCreated " + this.getName());

        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());

//        dblWheelDiameter = (Number) XMLSettingReader.settingsEntityGroup.getEntity("DriveTrain-WheelDiameter");
//        logger.writeEvent(0, FileLogger.EventType.Debug, "WheelDiameter: " + dblWheelDiameter.getValue());

        configDrivetrainControlType(RAW);

        lastLoopTime = System.currentTimeMillis();
    }

    private long lastLoopTime;

    @Override
    public void periodic() {
//        if(Math.abs(mDriveRawGoal) > 0.1 && mCurrentDriveRPM < 6500.0) {
//            mDriveAccelerationCurrent = DriveTrainSimulation.getAcceleration(Math.abs(mCurrentDriveRPM), Math.abs(getDriveMotorVoltage()), 0.0508 / 2, 60.0/4).times(Math.signum(mDriveRawGoal));
//        } else if(Math.abs(mCurrentDriveRPM) > 10){
//            mDriveAccelerationCurrent = new Acceleration( 3 * -Math.signum(mDriveRawGoal), METER_PER_SECOND2);
//        } else {
//            mDriveAccelerationCurrent = new Acceleration(0, METER_PER_SECOND2);
//        }
//
//        Velocity deltaV = mDriveAccelerationCurrent.times(new Time(System.currentTimeMillis() - lastLoopTime, MILLISECONDS)).times(8.75);

//        double deltaRPM = (deltaV.getValue(METER_PER_SECOND) / (Math.PI * 2 * 0.0508)) * 60;
//        mCurrentDriveRPM += deltaRPM;

        lastLoopTime = System.currentTimeMillis();

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        table.getEntry(getEntityPath() + "Speed").setDouble(mCurrentDriveRPM * (Math.PI * 2 * 0.0508));
        table.getEntry(getEntityPath() + "Angle").setDouble(getModuleAngle().getDegrees());
    }

    @Override
    public void setModuleAngle(Rotation2d angle) {
        turningSetPoint = angle;
        turningCurrent = angle;
    }

    @Override
    public Rotation2d getModuleGoalAngle() {
        return turningSetPoint;
    }

    @Override
    public Rotation2d getModuleAngle() {
        return turningSetPoint;
        //return turningCurrent;
    }

    @Override
    public void setRawDriveSpeed(double speed) {
        mDriveRawGoal = speed;
    }

    @Override
    public double getRawDrivePower() {
        return mDriveRawGoal;
    }

    @Override
    public double getCurrentDriveRPM() {
        return mCurrentDriveRPM;
    }

    @Override
    public void setVelocityDriveSpeed(Velocity speed) {
        mDriveVelocityGoal = speed;
        mDriveRawGoal = speed.getValue(FEET_PER_SECOND) / 16.5;
        mDriveVelocityCurrent = speed;
    }

    @Override
    public Velocity getDriveVelocity() {
        return mDriveVelocityCurrent;
    }

    @Override
    public Velocity getDriveVelocityGoal() {
        return mDriveVelocityGoal;
    }

    @Override
    public void setDriveDistanceTarget(Distance distance2d) {
        mDriveDistanceGoal = distance2d;
    }

    @Override
    public Distance getDriveDistanceTarget() {
        return mDriveDistanceGoal;
    }

    @Override
    public Distance getCurrentDrivePosition() {
        return mDriveDistanceCurrent;
    }

    @Override
    public void setDriveCoastMode(boolean brake) {

    }

    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        mDriveControlType = control;
    }

    @Override
    public Constants.DriveControlType getDriveControlType() {
        return mDriveControlType;
    }

    @Override
    public void setSwerveModuleState(SwerveModuleState states) {
        setModuleAngle(states.getRotation2d());
        switch(states.getControlType()) {
            case VELOCITY:
                setVelocityDriveSpeed(states.getSpeed2d());
                break;
            case DISTANCE:
                setDriveDistanceTarget(states.getDistance2d());
                break;
            case RAW:
                setRawDriveSpeed(states.getRawPowerValue());
                break;
        }
    }

    public SwerveModuleState getSwerveModuleState() {
//        switch(mDriveControlType) {
//            case DISTANCE:
//                return new SwerveModuleState(mDriveDistanceCurrent, turningCurrent, mSwerveDrivePosition);
//            case VELOCITY:
                return new SwerveModuleState(mDriveVelocityCurrent, turningCurrent, mSwerveDrivePosition);
//            default:
//                return new SwerveModuleState(mDriveRawPercent, turningCurrent, mSwerveDrivePosition);
//        }
    }

    @Override
    public SwerveModuleState getSwerveModuleAccelerationState(double voltage) {
        return new SwerveModuleState(mDriveAccelerationCurrent, new AngularAcceleration(0, RADIAN_PER_SECOND2), mSwerveDrivePosition);
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }

    @Override
    public double getDriveMotorVoltage() {
        return mDriveRawGoal * 12;
    }
}
