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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;
import frc.robot.library.hardware.FusedTrackingAlgorithm;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.SwerveKinematics;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.*;
import frc.robot.library.units.Number;

import static frc.robot.library.units.Units.Unit.FEET_PER_SECOND;
import static frc.robot.library.units.Units.Unit.INCH;

public class Teleop implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 8;

    private EntityGroup mRobotSubsystem;
    private XMLSettingReader mSettingReader;
    private SwerveDrivetrain mDrivetrain;
    private FusedTrackingAlgorithm mTrackingAlgorithm;
    private SwerveKinematics<Number> mKinematic;

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
        mTrackingAlgorithm = new FusedTrackingAlgorithm(60, (a) -> {});

        switch(mRobotSubsystem.getEntityGroupByType("DriveTrain").getName()) {
            case "Swerve Falcon":
            case "Swerve NEO":
            case "Swerve Simulation":
                logger.writeEvent(0, mRobotSubsystem.getEntityGroupByType("DriveTrain").getName());
                mCurrentDrivetrainPeriodRunnable = this::SwerveDrivetrainPeriodic;
                this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
                mKinematic = new SwerveKinematics<>(new Distance(1, INCH), new Distance(1, INCH));
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
        //Pair<Double, Double> xy = Constants.joyStickRadialDeadband(mDriverController.getLeftX(), -mDriverController.getLeftY(), 0.08);
        Pair<Double, Double> xy = Pair.of(mDriverController.getLeftX(), -mDriverController.getLeftY());

        //double rMag = Math.sin(Constants.deadband(mDriverController.getRightX(), 0.08)); //TODO must fix TrackWidth
        double rMag = 0;

        SwerveModuleState[] states = mKinematic.getSwerveModuleState(xy.getFirst(), xy.getSecond(), rMag);
        for(SwerveModuleState state : states) {
            System.out.println(state.toString());
        }
        //SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(xy.getFirst(), xy.getSecond(), rMag, 1, 1, Constants.DriveControlType.RAW);

        mDrivetrain.setSwerveModuleStates(states);
    }
}
