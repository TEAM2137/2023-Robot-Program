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

import edu.wpi.first.hal.REVPHJNI;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.FileLogger.EventType;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.splines.QuinticSpline;
import frc.robot.functions.splines.VelocityGenerator;
import frc.robot.library.Constants;
import frc.robot.library.Constants.StepState;
import frc.robot.library.OpMode;
import frc.robot.library.PurePursuit.PurePursuitGenerator;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Time;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.Point2d;
import frc.robot.library.units.UnitContainers.Pose2d;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.FEET_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;


@SuppressWarnings(value="FieldCanBeLocal")
public class Autonomous implements OpMode {

    //Filelogger values
    private FileLogger logger;

    //XML Setting reader File
    private XMLSettingReader mSettingReader;
    private XMLStepReader mStepReader;

    private EntityGroup mRobotSubsystem;
    private SwerveDrivetrain mDrivetrain;

    private int currentSplineIndex = 0;
    private List<List<PoseWithCurvature>> mPregeneratedSplines = new ArrayList<>();
    private List<List<Velocity>> mPregeneratedVelocities = new ArrayList<>();

    private SendableChooser<String> xmlStepChooserForDrew = new SendableChooser<>();

    public Autonomous() {
        try {
            File directory = new File(Constants.StandardFileAndDirectoryLocations.GenericStepList.getMainDirectory(isSimulation()));
            for (File options : directory.listFiles()) {
                if (options.getName().equalsIgnoreCase("Steps.xml")) {
                    xmlStepChooserForDrew.setDefaultOption(options.getName(), options.getAbsolutePath());
                } else if (!options.getName().equalsIgnoreCase("Settings.xml") && options.isFile()) {
                    xmlStepChooserForDrew.addOption(options.getName(), options.getAbsolutePath());
                }
            }

            SmartDashboard.putData("AutonomousChooser", xmlStepChooserForDrew);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    //region Main Autonomous Opmode Functions
    @Override
    public void init(XMLSettingReader xmlSettingReader, FileLogger fileLogger) {
        //Create Filelogger object and provide the current OPMODE state and debug
        this.logger = fileLogger;
        this.mSettingReader = xmlSettingReader;

        this.mRobotSubsystem = mSettingReader.getRobot();

        fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
//        this.mStepReader = new XMLStepReader(Constants.StandardFileAndDirectoryLocations.GenericStepList.getFileLocation(isSimulation()), fileLogger);
        this.mStepReader = new XMLStepReader(xmlStepChooserForDrew.getSelected(), fileLogger);
        fileLogger.writeEvent(0, "Step Count: " + this.mStepReader.getSteps().size());

        this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
        this.mDrivetrain.resetOdometry();

//        fileLogger.writeEvent(0, EventType.Status, "Starting Spline Pregeneration...");
//        mCurrentSplineIndex = -1;
//        preGenerateSplines();
//        fileLogger.writeEvent(0, EventType.Status, "Finished Spline Pregeneration...");

        DriverStation.silenceJoystickConnectionWarning(true);

        mPregeneratedSplines = new ArrayList<>();
        mPregeneratedVelocities = new ArrayList<>();

        pregeneratedSplines();
    }

    private int mLogCount = 0;

    public void pregeneratedSplines() {
        boolean generatedLast = true;
        List<PoseWithCurvature> catchList = new ArrayList<>();
        int currentCatchListSplineCount = 0;
        int currentSplineIdx = 0;

        while(mStepReader.hasSteps()) {
            Step step = mStepReader.pullNextStep();

            if(XMLStepReader.isDriveStep(step.getCommand()) && !mStepReader.isFirstDriveStep()) {
                List<edu.wpi.first.math.geometry.Pose2d> poses = new ArrayList<>();
                Pose2d<Distance> currentPosition = mDrivetrain.getCurrentOdometryPosition();

                if (mStepReader.hasPastDriveStep()) {
                    Step lds = mStepReader.getLastDriveStep();

                    if (mStepReader.lastStepsContainsDrive()) { //&& !mStepReader.isLastStep()) {
                        System.out.printf("Start Point %s (%.4f, %.4f) R%.4f\n", lds.getCommand(), lds.getXDistance(), lds.getYDistance(), lds.getParm(1));
                        poses.add(new edu.wpi.first.math.geometry.Pose2d(lds.getXDistance(), lds.getYDistance(), Rotation2d.fromDegrees(lds.getParm(1))));
                    } else {
                        double dx = currentPosition.getX().getValue(FOOT) - lds.getXDistance();
                        double dy = currentPosition.getY().getValue(FOOT) - lds.getYDistance();
                        System.out.printf("Start Point %s (%.4f, %.4f) R%.4f\n", lds.getCommand(), lds.getXDistance(), lds.getYDistance(), Math.toDegrees(Math.atan2(dy, dx)));
                        poses.add(new edu.wpi.first.math.geometry.Pose2d(lds.getXDistance(), lds.getYDistance(), Rotation2d.fromRadians(Math.atan2(dy, dx))));
                    }
                } else {
//                    System.out.printf("Start Point CURRENT (%.4f, %.4f) R%.4f", currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT), Rotation2d.fromDegrees(0));
                    poses.add(new edu.wpi.first.math.geometry.Pose2d(currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT), Rotation2d.fromDegrees(0))); //currentPosition.getTheta().getValue(DEGREE)
//                    poses.add(new edu.wpi.first.math.geometry.Pose2d(0, 0, Rotation2d.fromDegrees(currentPosition.getTheta().getValue(DEGREE))));
                }

                System.out.printf("End Point %s (%.4f, %.4f) R%.4f\n", step.getCommand(), step.getXDistance(), step.getYDistance(), step.getParm(1));
                poses.add(new edu.wpi.first.math.geometry.Pose2d(step.getXDistance(), step.getYDistance(), Rotation2d.fromDegrees(step.getParm(1))));

                QuinticSpline spline = new QuinticSpline(poses, 0.6);

                List<PoseWithCurvature> tmp = spline.getSplinePoints();
                System.out.printf("Generated %d points for spline\n", tmp.size());
                mPregeneratedSplines.add(tmp);
                catchList.addAll(tmp);
                currentCatchListSplineCount++;
                System.out.printf("Catch List Size Grew To %d\n", catchList.size());
            }

            if((!XMLStepReader.isDriveStep(step.getCommand()) || mStepReader.isLastDriveStep()) && !step.isParallel()) {
                if(generatedLast || catchList.size() == 0)
                    continue;

                System.out.println("Started Velocity Generation on Batch");

                VelocityGenerator velocityGenerator = new VelocityGenerator(catchList, new Velocity(16, FEET_PER_SECOND), new Acceleration(8, FEET_PER_SECOND2), 1);
                List<Velocity> velocities = velocityGenerator.getSpeeds();

                System.out.printf("Generate Velocity Count: %d\n", velocities.size());

                List<Velocity> tmpList = new ArrayList<>();
                for(int i = 0; i < velocities.size(); i++) {
                    tmpList.add(velocities.get(i));

//                    if(i == mPregeneratedSplines.get(Math.min(currentSplineIdx, mPregeneratedSplines.size() - 1)).size() - 1) {
                    if(tmpList.size() == mPregeneratedSplines.get(currentSplineIdx).size()) {
                        currentSplineIdx++;
                        mPregeneratedVelocities.add(tmpList);
                        System.out.printf("Spline finished at %d velocity, moving to next\n", tmpList.size());
                        tmpList = new ArrayList<>();
                    }
                }

                if(tmpList.size() > 0) {
                    mPregeneratedVelocities.add(tmpList);
                    System.out.printf("Final spline finished at %d velocity, moving to next\n", tmpList.size());
                }

                catchList = new ArrayList<>();
                currentCatchListSplineCount = 0;
            } else {
                generatedLast = false;
            }
        }

        mStepReader.resetCounter();

        for (int i = 0; i < mPregeneratedSplines.size(); i++) {
            logSplineToFile(Constants.StandardFileAndDirectoryLocations.GenericFileLoggerDir.getFileLocation(true) + "Spline" + mLogCount + ".txt",
                    mPregeneratedSplines.get(i),
                    mPregeneratedVelocities.get(Math.min(i, mPregeneratedVelocities.size() - 1)));
            mLogCount++;
        }
    }

    public void logSplineToFile(String pathAndName, List<PoseWithCurvature> poses, List<Velocity> velocities) {
        try {
            PrintWriter writer = new PrintWriter(pathAndName);

            for (PoseWithCurvature pose : poses) {
                writer.printf("(%.4f, %.4f)\n", pose.poseMeters.getX(), pose.poseMeters.getY());
            }

            for (int i = 0; i < velocities.size(); i++) {
                writer.printf("(%.4f, %.4f)\n", poses.get(Math.min(i, poses.size() - 1)).poseMeters.getX(), velocities.get(i).getValue(FEET_PER_SECOND));
            }

            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        if (Robot.currentActiveAutonomousSteps.size() == 0) {
            List<Step> nextSteps = mStepReader.pullNextSteps();

            for(Step step : nextSteps)
                step.changeStepState(StepState.STATE_INIT);

            //Restock the Action Steps
            Robot.currentActiveAutonomousSteps.addAll(nextSteps);
        }

//        System.out.println("Count:" + Robot.currentActiveAutonomousSteps.size());
        for(int i = 0; i < Robot.currentActiveAutonomousSteps.size(); i++) {
            Step tmpStep = Robot.currentActiveAutonomousSteps.get(i);
//            System.out.println("Running: " + tmpStep.getCommand());

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
            case "Drive":
            case "drive":
                swerveDrivePurePursuit(step);
                break;
            case "SetSwerveVelocity":
                setSwerveDrivetrainVelocity(step);
                break;
        }
    }
    //endregion

    private PurePursuitGenerator mDrivePurePursuitGenerator;
    private PIDController mDriveThetaController;
    private Distance mPurePursuitLookaheadDistance;
    private Distance mDriveMarginOfError = new Distance(1, INCH);

    //region Generic Drive Command
    public void swerveDrivePurePursuit(Step step) {
        switch(step.getStepState()) {
            case STATE_INIT:
                if(currentSplineIndex >= mPregeneratedSplines.size()) {
                    step.changeStepState(StepState.STATE_FINISH);
                    return;
                }

                mPurePursuitLookaheadDistance = new Distance(1, FOOT);
                mDrivePurePursuitGenerator = new PurePursuitGenerator(mPurePursuitLookaheadDistance,
                        mPregeneratedSplines.get(currentSplineIndex),
                        mPregeneratedVelocities.get(currentSplineIndex));

                if(step.hasValue("parm3") && step.hasValue("parm2") && step.getParm(3) == 1 && mDrivetrain.getThetaPIDController() != null) {
                    PID values = mDrivetrain.getThetaPIDController();
                    mDriveThetaController = new PIDController(values.getP(), values.getI(), values.getD());

                    mDriveThetaController.setSetpoint(step.getParm(2));
                } else {
                    mDriveThetaController = null;
                }

                step.changeStepState(StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                frc.robot.library.units.UnitContainers.Pose2d<Distance> currentPositionRun = mDrivetrain.getCurrentOdometryPosition();
//                currentPositionRun = new Pose2d<>(currentPositionRun.getY(), currentPositionRun.getX().times(-1), currentPositionRun.getTheta());
//                currentPositionRun = new Pose2d<>(currentPositionRun.getY(), currentPositionRun.getX(), currentPositionRun.getTheta());
//                System.out.printf("Current Run Position (%.4f, %.4f)\n", currentPositionRun.getX().getValue(FOOT), currentPositionRun.getY().getValue(FOOT));

                Map.Entry<edu.wpi.first.math.geometry.Pose2d, Velocity> results = mDrivePurePursuitGenerator.calculateGoalPose(new Translation2d(currentPositionRun.getX().getValue(FOOT), currentPositionRun.getY().getValue(FOOT)));
                SmartDashboard.putNumber("GoalX", results.getKey().getX());
                SmartDashboard.putNumber("GoalY", results.getKey().getY());

                double xVel = (results.getKey().getX() - currentPositionRun.getX().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);
                double yVel = (results.getKey().getY() - currentPositionRun.getY().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);

                double max = Math.max(Math.abs(xVel), Math.abs(yVel));

                Translation2d lastPose = mPregeneratedSplines.get(currentSplineIndex).get(mPregeneratedSplines.get(currentSplineIndex).size() - 1).poseMeters.getTranslation();

//                double currentDistance = lastPose.getDistance(new Translation2d(currentPositionRun.getX().getValue(FOOT), currentPositionRun.getY().getValue(FOOT)));

                xVel /= max;
                yVel /= max;

                xVel *= results.getValue().getValue(FEET_PER_SECOND);
                yVel *= results.getValue().getValue(FEET_PER_SECOND);
//                xVel *= 8 * (currentDistance / StartDistance);
//                yVel *= 8 * (currentDistance / StartDistance);

                SmartDashboard.putNumber("GoalVelX", xVel);
                SmartDashboard.putNumber("GoalVelY", yVel);

                if (!isSimulation()) {
                    double angleGoal;

                    if(mDriveThetaController != null) {
                        angleGoal = mDriveThetaController.calculate(mDrivetrain.getAngle().getValue(DEGREE));
                    } else {
                        angleGoal = 0;
                    }

                    SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(new Velocity(xVel, FEET_PER_SECOND), new Velocity(yVel, FEET_PER_SECOND), new AngularVelocity(angleGoal, DEGREE_PER_SECOND));
                    mDrivetrain.setSwerveModuleStates(states);
                } else {
                    mDrivetrain.swerveKinematics.reset(new Point2d<Distance>(
                            new Distance(xVel * 0.01 + currentPositionRun.getX().getValue(FOOT), FOOT),
                            new Distance(yVel * 0.01 + currentPositionRun.getY().getValue(FOOT), FOOT)));
                }

                if (lastPose.getDistance(new Translation2d(currentPositionRun.getX().getValue(FOOT), currentPositionRun.getY().getValue(FOOT))) < mDriveMarginOfError.getValue(FOOT)) {
                    step.changeStepState(StepState.STATE_FINISH);

                    currentSplineIndex++;

                    System.out.println("Finished Curve");
                    DriverStation.reportError("Finished Curve", false);
                    mDrivetrain.setSpeed(0.0);
                }
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
