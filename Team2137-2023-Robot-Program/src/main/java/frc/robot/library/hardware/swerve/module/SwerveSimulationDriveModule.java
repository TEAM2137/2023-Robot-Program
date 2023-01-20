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

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.data.Number;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.Constants;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import org.w3c.dom.Element;

import java.util.ArrayList;

public class SwerveSimulationDriveModule extends EntityGroup implements SwerveModule {

    public Motor mDriveMotor;
    public Motor mTurnMotor;
    private Encoder mTurnEncoder;
    private PIDController mDrivePIDController;
    private PIDController mTurnPIDController;

    private Speed2d mDriveVelocityGoal = new Speed2d(0);
    private Distance2d mDriveDistanceGoal = Distance2d.fromFeet(0);
    private double mDriveRawGoal = 0;
    private Rotation2d turningSetPoint = Rotation2d.fromDegrees(0);

    private final Speed2d mDriveVelocityCurrent = new Speed2d(0);
    private final Distance2d mDriveDistanceCurrent = Distance2d.fromFeet(0);
    private final double mDriveRawPercent = 0;
    private final Rotation2d turningCurrent = Rotation2d.fromDegrees(0);
    private final Number dblWheelDiameter;

    private Constants.DriveControlType mDriveControlType = Constants.DriveControlType.RAW;
    //private SwerveModuleState.SwerveModulePositions swerveModulePosition;

    private final FileLogger logger;

    private final SwerveModuleState.SwerveModulePositions mSwerveDrivePosition;

    public SwerveSimulationDriveModule(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        fileLogger.writeEvent(0, FileLogger.EventType.Error, "Simulated SwerveModuleCreated " + this.getName());

        mSwerveDrivePosition = SwerveModuleState.SwerveModulePositions.getPositionFromString(this.getName());

        logger = fileLogger;

        dblWheelDiameter = (Number) XMLSettingReader.settingsEntityGroup.getEntity("DriveTrain-WheelDiameter");
        logger.writeEvent(0, FileLogger.EventType.Debug, "WheelDiameter: " + dblWheelDiameter.getValue());

        configDrivetrainControlType(Constants.DriveControlType.RAW);
    }

    @Override
    public void periodic() {
        NetworkTableInstance table = NetworkTableInstance.getDefault();
        table.getEntry(getEntityPath() + "Speed").setDouble(getRawDrivePower());
        table.getEntry(getEntityPath() + "Angle").setDouble(getRawDrivePower());

//        if(lastRecordTime + periodBetweenRecords > System.currentTimeMillis()) {
//            getSwerveModuleState().writeToFileLoggerReplayFormat(logger);

//            lastRecordTime = System.currentTimeMillis();
            //System.out.println(state.toString());
//        }
    }

    @Override
    public void setModuleAngle(Rotation2d angle) {
        turningSetPoint = angle;
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
    public void setVelocityDriveSpeed(Speed2d speed) {
        mDriveVelocityGoal = speed;
    }

    @Override
    public Speed2d getDriveVelocity() {
        return mDriveVelocityCurrent;
    }

    @Override
    public Speed2d getDriveVelocityGoal() {
        return mDriveVelocityGoal;
    }

    @Override
    public void setDriveDistanceTarget(Distance2d distance2d) {
        mDriveDistanceGoal = distance2d;
    }

    @Override
    public Distance2d getDriveDistanceTarget() {
        return mDriveDistanceGoal;
    }

    @Override
    public Distance2d getCurrentDrivePosition() {
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

    public SwerveModuleState getSwerveModuleState() {
        switch(mDriveControlType) {
            case DISTANCE:
                return new SwerveModuleState(mDriveDistanceCurrent, turningCurrent, mSwerveDrivePosition);
            case VELOCITY:
                return new SwerveModuleState(mDriveVelocityCurrent, turningCurrent, mSwerveDrivePosition);
            default:
                return new SwerveModuleState(mDriveRawPercent, turningCurrent, mSwerveDrivePosition);
        }
    }

    @Override
    public SwerveModuleState.SwerveModulePositions getSwerveModuleLocation() {
        return mSwerveDrivePosition;
    }
}
