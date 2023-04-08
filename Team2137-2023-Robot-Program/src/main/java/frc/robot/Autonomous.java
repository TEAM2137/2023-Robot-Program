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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.library.hardware.endeffector.EndEffector;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.Time;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.Point2d;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitContainers.Vector2d;
import frc.robot.vision.objects.ObjectTracker;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;

import static edu.wpi.first.wpilibj.RobotBase.getRuntimeType;
import static edu.wpi.first.wpilibj.RobotBase.isSimulation;
import static frc.robot.Robot.mGameElementTracker;
import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.Time.TimeUnits.MILLISECONDS;
import static frc.robot.library.units.Time.TimeUnits.SECONDS;
import static frc.robot.library.units.TranslationalUnits.Acceleration.AccelerationUnits.FEET_PER_SECOND2;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

@SuppressWarnings(value = "FieldCanBeLocal")
public class Autonomous implements OpMode {

    //Filelogger values
    private FileLogger logger;

    //XML Setting reader File
    private XMLSettingReader mSettingReader;
    private XMLStepReader mStepReader;

    private EntityGroup mRobotSubsystem;
    private SwerveDrivetrain mDrivetrain;
    private EndEffector mClaw;

    private int currentSplineIndex = 0;
    private List<List<PoseWithCurvature>> mPregeneratedSplines = new ArrayList<>();
    private List<List<Velocity>> mPregeneratedVelocities = new ArrayList<>();

    private SendableChooser<String> xmlStepChooserForDrew = new SendableChooser<>();
    private HashMap<String, String> xmlStepChooserValues = new HashMap<String, String>();
    private String mCurrentPreloadedXMLFilePath;

    public Autonomous() {
        try {
            File directory = new File(Constants.StandardFileAndDirectoryLocations.GenericStepList.getMainDirectory(isSimulation()));
            for (File options : directory.listFiles()) {
                if (options.getName().equalsIgnoreCase("Steps.xml")) {
                    xmlStepChooserForDrew.setDefaultOption(options.getName(), options.getAbsolutePath());
                    xmlStepChooserValues.put(options.getName(), options.getAbsolutePath());
                } else if (!options.getName().equalsIgnoreCase("Settings.xml") && options.isFile()) {
                    xmlStepChooserForDrew.addOption(options.getName(), options.getAbsolutePath());
                    xmlStepChooserValues.put(options.getName(), options.getAbsolutePath());
                }
            }

            SmartDashboard.putData("AutonomousChooser", xmlStepChooserForDrew);
        } catch (Exception e) {
            e.printStackTrace();
        }

        mCurrentPreloadedXMLFilePath = xmlStepChooserForDrew.getSelected();
        Robot.fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
        this.mStepReader = new XMLStepReader(xmlStepChooserForDrew.getSelected(), Robot.fileLogger);
        Robot.fileLogger.writeEvent(0, "Step Count: " + this.mStepReader.getSteps().size());

        pregeneratedSplines();

        //Set up on change
        NetworkTable chooserTable = NetworkTableInstance.getDefault().getTable("/SmartDashboard/AutonomousChooser");
        chooserTable.addListener("selected", EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (table, key, event) -> { //was selected
            if(!event.valueData.value.getString().equalsIgnoreCase(mCurrentPreloadedXMLFilePath)) {
                mCurrentPreloadedXMLFilePath = xmlStepChooserValues.getOrDefault(event.valueData.value.getString(), "Steps.xml");
                DriverStation.reportError("Selected " + mCurrentPreloadedXMLFilePath, false);
                Robot.fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
                this.mStepReader = new XMLStepReader(mCurrentPreloadedXMLFilePath, Robot.fileLogger);
                Robot.fileLogger.writeEvent(0, "Step Count: " + this.mStepReader.getSteps().size());

                DriverStation.reportWarning("WAIT Updating Spline Pregenerated Values", false);
                pregeneratedSplines();
                DriverStation.reportWarning("FINISHED Updating Spline Pregenerated Values", false);
            }
        });

        Robot.subSystemCommandList.put("SetDrivetrainAngle", this::setDrivetrainAngle);
        Robot.subSystemCommandList.put("Seek2023Element", this::SeekGameElement2023);
    }

    //region Main Autonomous Opmode Functions
    @Override
    public void init(XMLSettingReader xmlSettingReader, FileLogger fileLogger) {
        //Create Filelogger object and provide the current OPMODE state and debug
        this.logger = fileLogger;
        this.mSettingReader = xmlSettingReader;

        this.mRobotSubsystem = mSettingReader.getRobot();

        this.mDrivetrain = (SwerveDrivetrain) mRobotSubsystem.getEntityGroupByType("DriveTrain");
        this.mDrivetrain.resetOdometry();

        this.mClaw = (EndEffector) mRobotSubsystem.getEntityGroupByType("Claw");

//        mCurrentPreloadedXMLFilePath = xmlStepChooserForDrew.getSelected();
//        Robot.fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
//        this.mStepReader = new XMLStepReader(xmlStepChooserForDrew.getSelected(), Robot.fileLogger);
//        Robot.fileLogger.writeEvent(0, "Step Count: " + this.mStepReader.getSteps().size());

//        pregeneratedSplines();

//        fileLogger.writeEvent(0, EventType.Status, "Starting Spline Pregeneration...");
        //currentSplineIndex = 0;
//        preGenerateSplines();
//        fileLogger.writeEvent(0, EventType.Status, "Finished Spline Pregeneration...");

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private int mLogCount = 0;

    public void pregeneratedSplines() {
        boolean generatedLast = true;
        List<PoseWithCurvature> catchList = new ArrayList<>();
        int currentSplineIdxL = 0;

        currentSplineIndex = 0;

        mPregeneratedSplines = new ArrayList<>();
        mPregeneratedVelocities = new ArrayList<>();

        mStepReader.resetCounter();

        while (mStepReader.hasSteps()) {
            Step step = mStepReader.pullNextStep();

            if(mStepReader.isFirstDriveStep()) {
                System.out.println("First Drive Step " + step.getCommand());
            }

            if (XMLStepReader.isDriveStep(step.getCommand()) && !mStepReader.isFirstDriveStep()) {
                List<edu.wpi.first.math.geometry.Pose2d> poses = new ArrayList<>();

//                Pose2d<Distance, Angle> currentPosition = new Pose2d<>(new Distance(0, FOOT), new Distance(0, FOOT), new Angle(0, DEGREE));
//                if(mDrivetrain != null)
//                    currentPosition = mDrivetrain.getCurrentOdometryPosition();

//                if (mStepReader.hasPastDriveStep()) {
                    Step lds = mStepReader.getLastDriveStep();

                    if (mStepReader.lastStepsContainsDrive()) { //&& !mStepReader.isLastStep()) {
                        System.out.printf("Start Point %s (%.4f, %.4f) R%.4f\n", lds.getCommand(), lds.getXDistance(), lds.getYDistance(), lds.getParm(1));
                        poses.add(new edu.wpi.first.math.geometry.Pose2d(lds.getXDistance(), lds.getYDistance(), Rotation2d.fromDegrees(lds.getParm(1))));
                    } else {
                        double dx = step.getXDistance() - lds.getXDistance();
                        double dy = step.getYDistance() - lds.getYDistance();
                        System.out.printf("Start Point %s (%.4f, %.4f) R%.4f\n", lds.getCommand(), lds.getXDistance(), lds.getYDistance(), Math.toDegrees(Math.atan2(dy, dx)));
                        poses.add(new edu.wpi.first.math.geometry.Pose2d(lds.getXDistance(), lds.getYDistance(), Rotation2d.fromRadians(Math.atan2(dy, dx))));
                    }
//                } else {
//                    System.out.printf("Start Point CURRENT (%.4f, %.4f) R%.4f", currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT), Rotation2d.fromDegrees(0));
//                    poses.add(new edu.wpi.first.math.geometry.Pose2d(currentPosition.getX().getValue(FOOT), currentPosition.getY().getValue(FOOT), Rotation2d.fromDegrees(0))); //currentPosition.getTheta().getValue(DEGREE)
//                    poses.add(new edu.wpi.first.math.geometry.Pose2d(0, 0, Rotation2d.fromDegrees(currentPosition.getTheta().getValue(DEGREE))));
//                }

                System.out.printf("End Point %s (%.4f, %.4f) R%.4f\n", step.getCommand(), step.getXDistance(), step.getYDistance(), step.getParm(1));
                poses.add(new edu.wpi.first.math.geometry.Pose2d(step.getXDistance(), step.getYDistance(), Rotation2d.fromDegrees(step.getParm(1))));

                QuinticSpline spline = new QuinticSpline(poses, 0.6);

                List<PoseWithCurvature> tmp = spline.getSplinePoints();
                System.out.printf("Generated %d points for spline\n", tmp.size());
                mPregeneratedSplines.add(tmp);
                catchList.addAll(tmp);
                System.out.printf("Catch List Size Grew To %d\n", catchList.size());
            }

            if ((!XMLStepReader.isDriveStep(step.getCommand()) || mStepReader.isLastDriveStep()) && !step.isParallel()) {
                if (generatedLast || catchList.size() == 0)
                    continue;

                System.out.println("Started Velocity Generation on Batch");

                VelocityGenerator velocityGenerator = new VelocityGenerator(catchList, new Velocity(16, FEET_PER_SECOND), new Acceleration(8, FEET_PER_SECOND2), 1);
                List<Velocity> velocities = velocityGenerator.getSpeeds();

                System.out.printf("Generate Velocity Count: %d\n", velocities.size());

                List<Velocity> tmpList = new ArrayList<>();
                for (int i = 0; i < velocities.size(); i++) {
                    tmpList.add(velocities.get(i));

//                    if(i == mPregeneratedSplines.get(Math.min(currentSplineIdx, mPregeneratedSplines.size() - 1)).size() - 1) {
                    if (tmpList.size() == mPregeneratedSplines.get(currentSplineIdxL).size()) {
                        currentSplineIdxL++;
                        mPregeneratedVelocities.add(tmpList);
                        System.out.printf("Spline finished at %d velocity, moving to next\n", tmpList.size());
                        tmpList = new ArrayList<>();
                    }
                }

                if (tmpList.size() > 0) {
                    mPregeneratedVelocities.add(tmpList);
                    System.out.printf("Final spline finished at %d velocity, moving to next\n", tmpList.size());
                }

                catchList = new ArrayList<>();
            } else {
                generatedLast = false;
            }
        }

        mStepReader.resetCounter();

//        for (int i = 0; i < mPregeneratedSplines.size(); i++) {
//            logSplineToFile(Constants.StandardFileAndDirectoryLocations.GenericFileLoggerDir.getFileLocation(true) + "Spline" + mLogCount + ".txt",
//                    mPregeneratedSplines.get(i),
//                    mPregeneratedVelocities.get(Math.min(i, mPregeneratedVelocities.size() - 1)));
//            mLogCount++;
//        }
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

            for (Step step : nextSteps)
                step.changeStepState(StepState.STATE_INIT);

            //Restock the Action Steps
            Robot.currentActiveAutonomousSteps.addAll(nextSteps);
        }

//        System.out.println("Count:" + Robot.currentActiveAutonomousSteps.size());
        for (int i = 0; i < Robot.currentActiveAutonomousSteps.size(); i++) {
            if(i == 0) {
                SmartDashboard.putString("CurrentRunningStep", Robot.currentActiveAutonomousSteps.get(i).getCommand());
            }

            Step tmpStep = Robot.currentActiveAutonomousSteps.get(i);
//            System.out.println("Running: " + tmpStep.getCommand());

            if (tmpStep.getStepState() == StepState.STATE_FINISH) {
                this.logger.writeEvent(3, EventType.Debug, tmpStep.getCommand() + " finished, now removing from operational stack");
                Robot.currentActiveAutonomousSteps.remove(tmpStep);
            } else {
                runCommand(tmpStep.getCommand(), tmpStep);
            }
        }

        for (Step step : Robot.persistenceAutonomousSteps) {
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

        if (Robot.subSystemCommandList.containsKey(commandName)) {
            Robot.subSystemCommandList.get(commandName).accept(step);
        }

        switch (commandName) {
            case "Drive":
            case "drive":
                swerveDrivePurePursuit(step);
                break;
        }
    }
    //endregion

    public void setDrivetrainAngle(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                PID values = mDrivetrain.getThetaPIDController();
                mDriveThetaController = new PIDController(values.getP(), values.getI(), values.getD());

                mDriveThetaController.setTolerance(1);
                mDriveThetaController.enableContinuousInput(-180, 180);
                mDriveThetaController.setSetpoint(step.getParm(1));

                DriverStation.reportError("Set drivetrain angle target to: " + step.getParm(1), false);
                step.changeStepState(StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                double angleGoal = -mDriveThetaController.calculate(mDrivetrain.getAngle().getValue(DEGREE));
                SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(new Velocity(0, FEET_PER_SECOND), new Velocity(0, FEET_PER_SECOND), new AngularVelocity(angleGoal, DEGREE_PER_SECOND), new Velocity(16, FEET_PER_SECOND));
                mDrivetrain.setSwerveModuleStates(states);

                if (mDriveThetaController.atSetpoint())
                    step.changeStepState(StepState.STATE_FINISH);

                break;
        }
    }

    private PurePursuitGenerator mDrivePurePursuitGenerator;
    private PIDController mDriveThetaController;
    private Distance mPurePursuitLookaheadDistance;
    private Distance mDriveMarginOfError = new Distance(1, INCH);

    //region Generic Drive Command
    public void swerveDrivePurePursuit(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                if (currentSplineIndex >= mPregeneratedSplines.size()) {
                    step.changeStepState(StepState.STATE_FINISH);
                    return;
                }

                mPurePursuitLookaheadDistance = new Distance(2, FOOT);
                mDrivePurePursuitGenerator = new PurePursuitGenerator(mPurePursuitLookaheadDistance,
                        mPregeneratedSplines.get(currentSplineIndex),
                        mPregeneratedVelocities.get(currentSplineIndex));

                PID values = mDrivetrain.getThetaPIDController();
                mDriveThetaController = new PIDController(values.getP(), values.getI(), values.getD());

                mDriveThetaController.setTolerance(1);
                mDriveThetaController.enableContinuousInput(-180, 180);
                mDriveThetaController.setSetpoint(step.getParm(2));

                DriverStation.reportError("Starting the Drive seqence for spline " + currentSplineIndex, false);

                step.StartTimer();
                step.changeStepState(StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                frc.robot.library.units.UnitContainers.Pose2d<Distance, Angle> currentPositionRun = mDrivetrain.getCurrentOdometryPosition();

                PurePursuitGenerator.PurePursuitOutput results = mDrivePurePursuitGenerator.calculateGoalPose(new Translation2d(currentPositionRun.getX().getValue(FOOT), currentPositionRun.getY().getValue(FOOT)));
                SmartDashboard.putNumber("GoalX", results.pose2d.getX());
                SmartDashboard.putNumber("GoalY", results.pose2d.getY());

                double xVel = (results.pose2d.getX() - currentPositionRun.getX().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);
                double yVel = (results.pose2d.getY() - currentPositionRun.getY().getValue(FOOT)); //* results.getValue().getValue(FEET_PER_SECOND);

                double max = Math.max(Math.abs(xVel), Math.abs(yVel));

                Translation2d lastPose = mPregeneratedSplines.get(currentSplineIndex).get(mPregeneratedSplines.get(currentSplineIndex).size() - 1).poseMeters.getTranslation();
                SmartDashboard.putNumber("LastPoseX", lastPose.getX());
                SmartDashboard.putNumber("LastPoseY", lastPose.getY());

                xVel /= max;
                yVel /= max;

                xVel *= results.velocity.getValue(FEET_PER_SECOND);
                yVel *= results.velocity.getValue(FEET_PER_SECOND);

                SmartDashboard.putNumber("GoalVelX", xVel);
                SmartDashboard.putNumber("GoalVelY", yVel);

                double angleGoal = -mDriveThetaController.calculate(mDrivetrain.getAngle().getValue(DEGREE));
                SmartDashboard.putNumber("ThetaController", angleGoal);

                SwerveModuleState[] states = mDrivetrain.calculateSwerveMotorSpeedsFieldCentric(new Velocity(xVel, FEET_PER_SECOND), new Velocity(yVel, FEET_PER_SECOND), new AngularVelocity(angleGoal, DEGREE_PER_SECOND));
                mDrivetrain.setSwerveModuleStates(states);

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

    private PIDController mSeekThetaController;

    public void SeekGameElement2023(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                PID values = mDrivetrain.getSeekGameElementPIDValues();
                mSeekThetaController = new PIDController(values.getP(), values.getI(), values.getD());

                mSeekThetaController.setSetpoint(0);

                DriverStation.reportWarning("Starting Seek...", false);

                step.changeStepState(StepState.STATE_RUNNING);
                step.StartTimer();
                break;
            case STATE_RUNNING:
                double rotationPower = mSeekThetaController.calculate(-mGameElementTracker.getClosestConeXPercent());

                SmartDashboard.putNumber("SeekElementThetaControllerOutput", rotationPower);

                SwerveModuleState[] swerveModuleStates = mDrivetrain.calculateSwerveMotorSpeeds(new Number(0), new Number(-step.getSpeed()), new Number(rotationPower));
//                SwerveModuleState[] swerveModuleStates = mDrivetrain.calculateSwerveMotorSpeeds(new Number(0), new Number(-step.getSpeed()), new Number(0));
                mDrivetrain.setSwerveModuleStates(swerveModuleStates);

                if(step.getParm(1) != null && step.getParm(1) != 0 && step.hasTimeElapsed(new Time(step.getParm(1), SECONDS))) {
                    step.changeStepState(StepState.STATE_FINISH);
                    mDrivetrain.setSpeed(0);
                }

                if(mClaw.isJawClosed() || mGameElementTracker.getConeCount() <= 0) {
                    step.changeStepState(StepState.STATE_FINISH);
                    mDrivetrain.setSpeed(0);

                    DriverStation.reportWarning("Element Lose or grabed", false);
                    //mClaw.setEffectorState(true);
                }
                break;
        }
    }
    //endregion

//    public void SeekPositionOverDistance(Step step) {
//        switch (step.getStepState()) {
//            case STATE_INIT:
//
//                step.changeStepState(StepState.STATE_RUNNING);
//                step.StartTimer();
//                break;
//
//            case STATE_RUNNING:
//                Pose2d<Distance, Angle> current = mDrivetrain.getCurrentOdometryPosition();
//                Point2d<Distance> desired = new Point2d<Distance>(new Distance(step.getXDistance(), FOOT), new Distance(step.getYDistance(), FOOT));
//
//                Angle angle = Math.atan2();
//        }
//
//    }
}
