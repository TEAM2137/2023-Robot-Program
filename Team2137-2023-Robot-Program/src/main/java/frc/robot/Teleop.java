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

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.library.*;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.Gamepad;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;

public class Teleop implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 0;

    private EntityGroup mRobotSubsystem;
    private XMLSettingReader mSettingReader;
    private SwerveDrivetrain mDrivetrain;

    private Runnable mCurrentDrivetrainPeriodRunnable;

    private final Gamepad mDriverController = new Gamepad(0);
    private final Gamepad mOperatorController = new Gamepad(1);

    @Override
    public void init(XMLSettingReader xmlSettingReader, XMLStepReader xmlStepReader, FileLogger fileLogger) {
        this.logger = fileLogger;

        this.mSettingReader = xmlSettingReader;
        this.mRobotSubsystem = this.mSettingReader.getRobot();

        switch(mRobotSubsystem.getEntityGroupByType("DriveTrain").getName()) {
            case "Swerve Falcon":
            case "Swerve NEO":
                logger.writeEvent(0, mRobotSubsystem.getEntityGroupByType("DriveTrain").getName());
                mCurrentDrivetrainPeriodRunnable = this::SwerveDrivetrainPeriodic;
                this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
                break;
        }

        mDrivetrain.leftFrontModule.setModuleAngle(Rotation2d.fromDegrees(90));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Front Module Actual Angle", mDrivetrain.leftFrontModule.getModuleAngle().getDegrees());
//        mCurrentDrivetrainPeriodRunnable.run();
    }

    @Override
    public void end() {
        logger.writeEvent(0, FileLogger.EventType.Status, "TELEOP Ending");
    }

    private void SwerveDrivetrainPeriodic() {
        logger.setTag("SwerveDrivetrainPeriodic()");
        double xMag = mDriverController.getX(Gamepad.Hand.kLeft, 0.1);
        double yMag = mDriverController.getY(Gamepad.Hand.kLeft, 0.1);
        double rMag = mDriverController.getX(Gamepad.Hand.kRight, 0.1); //TODO must fix TrackWidth
        SwerveModuleState[] state = ((SwerveDrivetrain) mDrivetrain).calculateSwerveMotorSpeeds(xMag, yMag, rMag, 1, 1, Constants.DriveControlType.RAW);

        SmartDashboard.putNumber(logger.getTag() + "-RightBackPower", state[0].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-RightBackAngle", state[0].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-LeftBackPower", state[1].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-LeftBackAngle", state[1].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-RightFrontPower", state[2].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-RightFrontAngle", state[2].getRotation2d().getDegrees());
        SmartDashboard.putNumber(logger.getTag() + "-LeftFrontPower", state[3].getRawPowerValue());
        SmartDashboard.putNumber(logger.getTag() + "-LeftFrontAngle", state[3].getRotation2d().getDegrees());

        ((SwerveDrivetrain) mDrivetrain).setSwerveModuleStates(state);
    }
}
