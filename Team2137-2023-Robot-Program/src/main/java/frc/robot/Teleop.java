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
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;
import frc.robot.library.hardware.FusedTrackingAlgorithm;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.SwerveKinematics;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Number;
import frc.robot.library.units.TranslationalUnits.Velocity;

import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

public class Teleop implements OpMode {

    private FileLogger logger;
    private final int mintDebug = 8;

    private EntityGroup mRobotSubsystem;
    private XMLSettingReader mSettingReader;
    private SwerveDrivetrain mDrivetrain;
    private FusedTrackingAlgorithm mTrackingAlgorithm;
    private SwerveKinematics<Number> mKinematic;

    private Runnable mCurrentDrivetrainPeriodRunnable;

    @Override
    public void init(XMLSettingReader xmlSettingReader, XMLStepReader xmlStepReader, FileLogger fileLogger) {
        this.logger = fileLogger;

        logger.writeEvent(0, "Controller Connect: " + Robot.primaryController.isConnected());

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
                this.mDrivetrain.configDrivetrainControlType(Constants.DriveControlType.VELOCITY);
                mKinematic = new SwerveKinematics<>(new Distance(1, INCH), new Distance(1, INCH));
                break;
        }
    }

    @Override
    public void periodic() {
//        mCurrentDrivetrainPeriodRunnable.run();

//        logger.writeEvent(0, "Current Action Step Count: " + Robot.currentActiveSteps.size());

        for(int i = 0; i < Robot.currentActiveSteps.size(); i++) {
            Step tmpStep = Robot.currentActiveSteps.get(i);
//            logger.writeEvent(0, "Running Command With Name: " + tmpStep.getCommand());

            if (tmpStep.getStepState() == Constants.StepState.STATE_FINISH) {
                this.logger.writeEvent(3, FileLogger.EventType.Debug, tmpStep.getCommand() + " finished, now removing from operational stack");
                Robot.currentActiveSteps.remove(tmpStep);
            } else {
                if(Robot.subSystemCommandList.containsKey(tmpStep.getCommand())) {
                    Robot.subSystemCommandList.get(tmpStep.getCommand()).accept(tmpStep);
                }
            }
        }
    }

    @Override
    public void end() {
        logger.writeEvent(0, FileLogger.EventType.Status, "TELEOP Ending");
    }

    private void SwerveDrivetrainPeriodic() {
        logger.setTag("SwerveDrivetrainPeriodic()");
        Pair<Double, Double> xy = Constants.joyStickSlopedDeadband(Robot.primaryController.getLeftX(), -Robot.primaryController.getLeftY(), 0.08);
        //double rMag = Constants.deadband(mDriverController.getRightX(), 0.08) * 35; //TODO must fix TrackWidth
        double rMag = Constants.deadband(Robot.primaryController.getRightX(), 0.08); //TODO must fix TrackWidth

        Velocity x = new Velocity(xy.getFirst() * 16.5, FEET_PER_SECOND);
        Velocity y = new Velocity(xy.getSecond() * 16.5, FEET_PER_SECOND);
        AngularVelocity r = new AngularVelocity(rMag * 10.0, RADIAN_PER_SECOND);

        SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(x, y, r);
//        SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(xy.getFirst(), xy.getSecond(), rMag * 2, 1, 1, Constants.DriveControlType.RAW);

        mDrivetrain.setSwerveModuleStates(states);
    }
}
