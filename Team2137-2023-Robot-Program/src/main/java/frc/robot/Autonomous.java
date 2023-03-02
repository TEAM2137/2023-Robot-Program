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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.FileLogger.EventType;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.splines.QuinticSpline;
import frc.robot.functions.splines.TrapezoidalVelocity;
import frc.robot.functions.splines.VelocityGenerator;
import frc.robot.library.*;
import frc.robot.library.Constants.StepState;
import frc.robot.library.PurePursuit.PurePursuitGenerator;
import frc.robot.library.hardware.deadReckoning.DeadWheelActiveTracking;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Time;
import frc.robot.library.units.TranslationalUnits.TranslationUnit;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.UnitContainers.Vector2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.RADIAN;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.RADIAN_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.FEET_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.METER_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.METER_PER_SECOND;


@SuppressWarnings(value="FieldCanBeLocal")
public class Autonomous implements OpMode {

    //Filelogger values
    private FileLogger logger;

    //XML Setting reader File
    private XMLSettingReader mSettingReader;
    private XMLStepReader mStepReader;

    private EntityGroup mRobotSubsystem;
    private SwerveDrivetrain mDrivetrain;

    private List<Step> mCurrentDriveSteps;

//    private boolean boolAcceleratePeriodic = true;
//    private ScheduledThreadPoolExecutor mPeriodicAccelerator;

    private StepState mDriveStepState = StepState.STATE_NOT_STARTED;

    //region Main Autonomous Opmode Functions
    @Override
    public void init(XMLSettingReader xmlSettingReader, FileLogger fileLogger) {
        //Create Filelogger object and provide the current OPMODE state and debug
        this.logger = fileLogger;
        this.mSettingReader = xmlSettingReader;
        this.mCurrentDriveSteps = new ArrayList<>();

        this.mRobotSubsystem = mSettingReader.getRobot();

        fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
        this.mStepReader = new XMLStepReader(Constants.StandardFileAndDirectoryLocations.GenericStepList.getFileLocation(isSimulation()), fileLogger);
        fileLogger.writeEvent(0, "Step Count: " + this.mStepReader.getSteps().size());

        this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
        this.mDrivetrain.resetOdometry();

        this.mDriveStepState = StepState.STATE_NOT_STARTED;

        mCurrentDriveSteps.addAll(Robot.currentActiveAutonomousSteps);
        this.mDriveStepState = StepState.STATE_INIT;

//        if(boolAcceleratePeriodic) {
//            mPeriodicAccelerator = new ScheduledThreadPoolExecutor(1);
//            mPeriodicAccelerator.scheduleAtFixedRate(this::periodicImpl, 0, 50, TimeUnit.MILLISECONDS);
//        }
    }

    @Override
    public void periodic() {
        if (mCurrentDriveSteps.size() == 0 && Robot.currentActiveAutonomousSteps.size() == 0) {
            //Restock the Drive Steps
            mCurrentDriveSteps.addAll(mStepReader.prePullSplineSteps());
            if(mCurrentDriveSteps.size() > 0) mDriveStepState = StepState.STATE_INIT;

            //Restock the Action Steps
            Robot.currentActiveAutonomousSteps.addAll(mStepReader.pullNextSteps());
        }

        swerveDrivePurePursuit();
        //logger.writeEvent(0, "Current Action Step Count: " + Robot.currentActiveSteps.size());

        for(int i = 0; i < Robot.currentActiveAutonomousSteps.size(); i++) {
            Step tmpStep = Robot.currentActiveAutonomousSteps.get(i);
            //logger.writeEvent(0, "Running Command With Name: " + tmpStep.getCommand());

            if (tmpStep.getStepState() == StepState.STATE_FINISH) {
                this.logger.writeEvent(3, EventType.Debug, tmpStep.getCommand() + " finished, now removing from operational stack");
                Robot.currentActiveAutonomousSteps.remove(tmpStep);
            } else {
                runCommand(tmpStep.getCommand(), tmpStep);
            }
        }

        for(Step step : Robot.persistenceAutonomousSteps) {
            runCommand(step.getCommand(), step);
        }
    }

    @Override
    public void end() {
        logger.writeEvent(0, EventType.Status, "Autonomous Ending");
        logger.close();
        Robot.currentActiveAutonomousSteps.clear();
    }
    //endregion

    //region Autonomous Running Util Functions
    /**
     * Util Functions
     */
    public void runCommand(String commandName, Step step) {

        if(Robot.subSystemCommandList.containsKey(commandName)) {
            Robot.subSystemCommandList.get(commandName).accept(step);
        }

        switch (commandName) {
            case "TestPrint":
                print(step);
                break;
            case "SetSwerveVelocity":
                setSwerveDrivetrainVelocity(step);
                break;
            case "Drive":
//                drive(step);
                break;
        }
    }
    //endregion

    //region Test Functional Commands
    private static int mCommandPrintTestCounter = 0;

    public void print(Step step) {
        switch(step.getStepState()) {
            case STATE_INIT:
                logger.writeEvent(0, "Test Print Function Entered STATE_INIT");

                step.changeStepState(StepState.STATE_RUNNING);
                break;

            case STATE_RUNNING:
                logger.writeEvent(0, "Running " + mCommandPrintTestCounter + " times");
                mCommandPrintTestCounter++;

                if(mCommandPrintTestCounter >= step.getParm(1)) {
                    step.changeStepState(StepState.STATE_FINISH);
                }
                break;

            case STATE_FINISH:
                logger.writeEvent(0, "Finished Test Print Function");
                break;
        }
    }
    //endregion

    private List<PoseWithCurvature> mDrivePoseWithCurvatureList;
    private PurePursuitGenerator mDrivePurePursuitGenerator;
    private VelocityGenerator velocityGenerator;
    private Distance mPurePursuitLookaheadDistance;
    private Distance mDriveMarginOfError = new Distance(1, INCH);

    //region Generic Drive Command
    public void swerveDrivePurePursuit() {
        switch(mDriveStepState) {
            case STATE_INIT:
                List<Pose2d> poseList = new ArrayList<>();

                logger.writeEvent(0, "Starting drive steps: " + mCurrentDriveSteps.size());
                for(Step driveStep : mCurrentDriveSteps) {
                    poseList.add(new Pose2d(new Translation2d(driveStep.getXDistance(), driveStep.getYDistance()), Rotation2d.fromDegrees(driveStep.getParm(1))));
                }

                QuinticSpline spline = new QuinticSpline(poseList, 0.6);
                mDrivePoseWithCurvatureList = spline.getSplinePoints();

                velocityGenerator = new VelocityGenerator(mDrivePoseWithCurvatureList, new Velocity(16, FEET_PER_SECOND), new Acceleration(8, FEET_PER_SECOND2), 1);
                List<Velocity> velocities = velocityGenerator.getSpeeds();

                //mPurePursuitLookaheadDistance = new Distance(((Number) Robot.settingsEntityGroup.getEntity("PurePursuitLookahead")).getValue(), FOOT);
                mPurePursuitLookaheadDistance = new Distance(1, FOOT);
                mDrivePurePursuitGenerator = new PurePursuitGenerator(mPurePursuitLookaheadDistance, mDrivePoseWithCurvatureList, velocities);

                DriverStation.silenceJoystickConnectionWarning(true);

                mDriveStepState = StepState.STATE_RUNNING;
                break;
            case STATE_RUNNING:
                SwerveDrivetrain drivetrain = mDrivetrain;
                frc.robot.library.units.UnitContainers.Pose2d<Distance> currentPosition = drivetrain.getCurrentOdometryPosition();

                Map.Entry<Pose2d, Velocity> results = mDrivePurePursuitGenerator.calculateGoalPose(new Translation2d(currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT)));

                double xVel = (results.getKey().getX() - currentPosition.getX().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);
                double yVel = (results.getKey().getY() - currentPosition.getY().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);

                double max = Math.max(xVel, yVel);
                xVel /= max;
                yVel /= max;

                xVel *= results.getValue().getValue(FEET_PER_SECOND);
                yVel *= results.getValue().getValue(FEET_PER_SECOND);

                SwerveModuleState[] states = drivetrain.swerveKinematics.getSwerveModuleState(new Velocity(xVel, FEET_PER_SECOND), new Velocity(yVel, FEET_PER_SECOND), new AngularVelocity(0, RADIAN_PER_SECOND));

//                System.out.printf("(%.4f, %.4f),", currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT));

                Translation2d lastPose = mDrivePoseWithCurvatureList.get(mDrivePoseWithCurvatureList.size() - 1).poseMeters.getTranslation();

                if(lastPose.getDistance(new Translation2d(currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT))) < mDriveMarginOfError.getValue(FOOT)) {
                    mDriveStepState = StepState.STATE_FINISH;
                    for (Step step : mCurrentDriveSteps) {
                        if(step.getCommand().equalsIgnoreCase("Drive"))
                            step.changeStepState(StepState.STATE_FINISH);
                    }
                    drivetrain.setSpeed(0.0);
                }

                drivetrain.setSwerveModuleStates(states);
                break;
            case STATE_FINISH:

                break;
        }
    }
    //endregion

    //region Swerve Autonomous Commands
    /**
     * Command that sets the Swerve Drivetrain X,Y,R velocities
     *
     * Step layout:
     *  -XDistance is X Velocity
     *  -YDistance is Y Velocity
     *  -Parm 0 is the R Velocity
     *
     *  Command Mode:
     *   -
     *
     * @param step Provided Step
     */
    public void setSwerveDrivetrainVelocity(Step step) {
        logger.setTag("setSwerveDrivetrainVelocity()");
        switch (step.getStepState()) {
            case STATE_INIT:
                SwerveDrivetrain swerve = mDrivetrain;

                Velocity xVelocity = new Velocity(step.getXDistance(), FEET_PER_SECOND);
                Velocity yVelocity = new Velocity(step.getYDistance(), FEET_PER_SECOND);
                Velocity rotationVelocity = new Velocity(step.getParm(0, 0.0), FEET_PER_SECOND);

                SmartDashboard.putNumber(logger.getTag() + "-XVelocity", xVelocity.getValue(FEET_PER_SECOND));
                SmartDashboard.putNumber(logger.getTag() + "-YVelocity", yVelocity.getValue(FEET_PER_SECOND));
                SmartDashboard.putNumber(logger.getTag() + "-RVelocity", rotationVelocity.getValue(FEET_PER_SECOND));

                //TODO fix the axel track
                swerve.setSwerveModuleStates(swerve.calculateSwerveMotorSpeeds(xVelocity.getValue(FEET_PER_SECOND), yVelocity.getValue(FEET_PER_SECOND), rotationVelocity.getValue(FEET_PER_SECOND), 1, 1, Constants.DriveControlType.VELOCITY));

                step.changeStepState(StepState.STATE_RUNNING);
                step.StartTimer();

                this.logger.writeEvent(5, EventType.Status, "Init Finished for set Swerve Drivetrain Velocity {" + xVelocity + ", " + yVelocity + ", " + rotationVelocity + "}");
                break;

            case STATE_RUNNING:

                if(step.getParm(2, 0d) == 1) {

                }

                if(step.getParm(1, 0d) != 0 && step.hasTimeElapsed(new Time(step.getParm(1, 0d), SECONDS))) {
                    mDrivetrain.setSpeed(0);
                    step.changeStepState(StepState.STATE_FINISH);
                    this.logger.writeEvent(5, EventType.Status, "Finished Timed Set Swerve Drive Train Velocity");
                }
                break;

            case STATE_FINISH:
                break;
        }
    }
    //endregion
}
