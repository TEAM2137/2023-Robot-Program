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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.library.*;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;

public class Teleop implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 8;

    private EntityGroup mRobotSubsystem;
    private XMLSettingReader mSettingReader;
    private SwerveDrivetrain mDrivetrain;

    private Runnable mCurrentDrivetrainPeriodRunnable;

    private XboxController mDriverController;
    //private final Gamepad mOperatorController = new Gamepad(1);

    @Override
    public void init(XMLSettingReader xmlSettingReader, XMLStepReader xmlStepReader, FileLogger fileLogger) {
        this.logger = fileLogger;

        mDriverController = new XboxController(0);

        logger.writeEvent(0, "Controller Connect: " + mDriverController.isConnected());

        this.mSettingReader = xmlSettingReader;
        this.mRobotSubsystem = this.mSettingReader.getRobot();

        switch(mRobotSubsystem.getEntityGroupByType("DriveTrain").getName()) {
            case "Swerve Falcon":
            case "Swerve NEO":
            case "Swerve Simulation":
                logger.writeEvent(0, mRobotSubsystem.getEntityGroupByType("DriveTrain").getName());
                mCurrentDrivetrainPeriodRunnable = this::SwerveDrivetrainPeriodic;
                this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
                break;
        }
    }

    @Override
    public void periodic() {
        mCurrentDrivetrainPeriodRunnable.run();
    }

    @Override
    public void end() {
        logger.writeEvent(0, FileLogger.EventType.Status, "TELEOP Ending");
    }

    private void SwerveDrivetrainPeriodic() {
        logger.setTag("SwerveDrivetrainPeriodic()");
        double xMag = Constants.deadband(mDriverController.getLeftX(), 0.05); //TODO fix deadbad to be triangulated not single axis
        double yMag = -Constants.deadband(mDriverController.getLeftY(), 0.05);
        double rMag = Constants.deadband(mDriverController.getRightX(), 0.05); //TODO must fix TrackWidth
        SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeeds(xMag, yMag, rMag, 1, 1, Constants.DriveControlType.RAW);

        for(SwerveModuleState state : states) {
            state.writeToFileLoggerReplayFormat(logger);
        }

        mDrivetrain.setSwerveModuleStates(states);
    }
}
