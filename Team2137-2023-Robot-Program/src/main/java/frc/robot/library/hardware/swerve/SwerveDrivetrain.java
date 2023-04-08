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

package frc.robot.library.hardware.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.InstantCommand;
import frc.robot.functions.io.xmlreader.data.mappings.PersistentCommand;
import frc.robot.functions.io.xmlreader.objects.Gyro;
import frc.robot.functions.io.xmlreader.objects.motor.Motor;
import frc.robot.library.Constants;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.FusedTrackingAlgorithm;
import frc.robot.library.hardware.swerve.module.SwerveModule;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngleUnit;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Unit;
import frc.robot.library.units.UnitContainers.CartesianValue;
import frc.robot.library.units.UnitContainers.Point2d;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitEnum;
import frc.robot.library.units.UnitUtil;
import frc.robot.vision.limelight.AprilTags;
import frc.robot.vision.limelight.ReflectiveTape;
import org.ejml.simple.SimpleMatrix;
import org.w3c.dom.Element;

import java.util.ArrayList;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;


public class SwerveDrivetrain extends EntityGroup implements DriveTrain {

    public SwerveModule leftFrontModule;
    public SwerveModule leftBackModule;
    public SwerveModule rightFrontModule;
    public SwerveModule rightBackModule;

    private Distance robotLength;
    private Distance robotWidth;
    private Velocity maxDrivetrainVelocity;

    private double mInitialPitch = 0;

    private PID mThetaPIDValues;

    private PIDController mSeekGameElementPIDController;
    private PID mSeekGameElementPIDValues;
    private PIDController mThetaController;

    public SwerveKinematics swerveKinematics;

    public Motor.MotorTypes driveTrainType;

    private final FileLogger logger;

    private final Pigeon2 pigeonIMU;
    private Gyro gyroObj;

    private FusedTrackingAlgorithm fusedTrackingAlgorithm;

    public SwerveDrivetrain(Element element, EntityGroup parent) {
        super(element, parent, true);

        logger = getLogger();
        logger.setTag("SwerveDriveTrainConstructor");

        robotLength = (Distance) Robot.settingsEntityGroup.getEntity("RobotLength");
        if (robotLength == null) {
            logger.writeEvent(1, FileLogger.EventType.Error, "RobotLength Element is not found! Using default 28\"");
            robotLength = new Distance(28, INCH);
        }

        robotWidth = (Distance) Robot.settingsEntityGroup.getEntity("RobotWidth");
        if (robotWidth == null) {
            logger.writeEvent(1, FileLogger.EventType.Error, "RobotWidth Element is not found! Using default 28\"");
            robotWidth = new Distance(28, INCH);
        }

        PID autoLevel = (PID) getEntity("AutoLevel");
        if (autoLevel != null) {
            DriverStation.reportWarning("Built Auto Level PID", false);
            mAutoLevelPIDController = new PIDController(autoLevel.getP(), autoLevel.getI(), autoLevel.getD());
        }

        PID thetaLevel = (PID) getEntity("ThetaController");
        if(thetaLevel != null) {
            mThetaPIDValues = thetaLevel;
        }

        mSeekGameElementPIDValues = (PID) getEntity("SeekThetaController");
        if(mSeekGameElementPIDValues != null) {
            mSeekGameElementPIDController = new PIDController(mSeekGameElementPIDValues.getP(), mSeekGameElementPIDValues.getI(), mSeekGameElementPIDValues.getD());
        }

        Number velocityVal = (Number) getEntity("MaxVelocity");
        if(velocityVal != null) {
            maxDrivetrainVelocity = new Velocity(velocityVal.getValue(), FEET_PER_SECOND);
        } else {
            maxDrivetrainVelocity = new Velocity(Double.POSITIVE_INFINITY, FEET_PER_SECOND);
        }

        logger.writeEvent(6, FileLogger.EventType.Debug, "Creating Swerve Kinematics class...");
        swerveKinematics = new SwerveKinematics(robotWidth, robotLength, 1.0);

        logger.writeEvent(6, FileLogger.EventType.Debug, "Finding swerve modules from EntityGroup");
        try {
            leftFrontModule = (SwerveModule) getChildEntityGroup("LeftFront");
            leftBackModule = (SwerveModule) getChildEntityGroup("LeftBack");
            rightFrontModule = (SwerveModule) getChildEntityGroup("RightFront");
            rightBackModule = (SwerveModule) getChildEntityGroup("RightBack");
        } catch (Exception e) {
            logger.writeEvent(0, FileLogger.EventType.Error, "Failed to find or convert to Swerve Modules!!");
            logger.writeLine(e.getMessage());
        }

        logger.writeEvent(6, FileLogger.EventType.Debug, "Creating Pigeon Gyro...");
        gyroObj = (Gyro) getEntity("Pigeon");
        pigeonIMU = new Pigeon2(gyroObj.getID());
        pigeonIMU.configFactoryDefault();
        pigeonIMU.configMountPose(gyroObj.getOffset(), 0, 0);
        mInitialPitch = pigeonIMU.getRoll();

        this.addSubsystemCommand("rawDrive", this::rawDrive);
        this.addSubsystemCommand("SetDrivetrainVelocity", this::setDrivetrainVelocity);
        this.addSubsystemCommand("SetPosition", this::setPosition);
        this.addSubsystemCommand("GyroReset", this::gyroReset);
        this.addSubsystemCommand("AutoLevel", this::autoLevel);
        this.addSubsystemCommand("DriveOnCharge", this::DriveOnChargeStep);
        this.addSubsystemCommand("xLock", this::xLock);

        Robot.threadPoolExecutor.scheduleAtFixedRate(this::updateKinematics, 0, 50, TimeUnit.MILLISECONDS);

        AprilTags.init();
//        double[] test = new double[] {};

//        ArrayList<Callable<Point2d<Distance>>> callables = new ArrayList<>();
//        ArrayList<>
//        callables.add(swerveKinematics::getCurrentRobotPosition);
//        callables.add(() -> new Point2d<Distance>(new Distance(AprilTags.getX(), INCH), new Distance(AprilTags.getY(), INCH)));
//
//        fusedTrackingAlgorithm = new FusedTrackingAlgorithm(swerveKinematics::getCurrentRobotPosition, () -> {
//            return new Point2d<Distance>(new Distance(AprilTags.getX(), INCH), new Distance(AprilTags.getY(), INCH));
//        }, 50);
    }

    /**
     * Private function to pass to a thread executor to update the odometry. Should have a run period equal to that of
     * the CAN packet period for the encoder.
     */
    private void updateKinematics() {
        SwerveModuleState lfVelState = leftFrontModule.getSwerveModuleState();
        SwerveModuleState lbVelState = leftBackModule.getSwerveModuleState();
        SwerveModuleState rfVelState = rightFrontModule.getSwerveModuleState();
        SwerveModuleState rbVelState = rightBackModule.getSwerveModuleState();

        Point2d<Distance> dist = swerveKinematics.updateSwerveKinematics(new SwerveModuleState[]{lfVelState, lbVelState, rfVelState, rbVelState}, getAngle());

        SmartDashboard.putNumber("Dist X", dist.getX().getValue(FOOT));
        SmartDashboard.putNumber("Dist Y", dist.getY().getValue(FOOT));
        SmartDashboard.putNumber("Theta", getAngle().getValue(DEGREE));
    }

    /**
     * Instant Command that resets the gyro to the zero value for the driver's usage.
     * @param step - Step to run the reset on.
     */
    @InstantCommand
    public void gyroReset(Step step) {
        if (step.getStepState() == Constants.StepState.STATE_INIT) {
//            pigeonIMU.configMountPoseYaw(gyroObj.getOffset());

            pigeonIMU.setYaw(0);

            logger.writeEvent(0, FileLogger.EventType.Status, "DriverGryoReset Complete");

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    /**
     * Instant Command that sets the position of the drive train usually at the beginning of the match.
     * @param step - Step to run inorder to set position
     */
    @InstantCommand
    public void setPosition(Step step) {
        if (step.getStepState() == Constants.StepState.STATE_INIT) {
            swerveKinematics.reset(new Point2d<Distance>(new Distance(step.getXDistance(), FOOT), new Distance(step.getYDistance(), FOOT)));

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    public PIDController mAutoLevelPIDController;
    public Timer mAutoLevelSustainedTargetTimer;
    public boolean mAutoLevelTimerStarted;

    /**
     * Auto Level command for the 2023 FRC game runs when robot is ON the charge station and uses a PID loop to level
     * the robot
     * @param step - Step for the command to run
     */
    @InstantCommand
    public void autoLevel(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                mAutoLevelPIDController.reset();
                mAutoLevelPIDController.setSetpoint(0);

                mAutoLevelSustainedTargetTimer = new Timer();
                mAutoLevelSustainedTargetTimer.stop();

                mAutoLevelTimerStarted = false;
                break;
            case STATE_RUNNING:
                Angle angle = new Angle(-(pigeonIMU.getRoll() - mInitialPitch), DEGREE);
                double output = mAutoLevelPIDController.calculate(-angle.getValue(DEGREE));

                SwerveModuleState[] swerveModuleState = new SwerveModuleState[]{
                        new SwerveModuleState(new Number(output), Rotation2d.fromDegrees(0), SwerveModuleState.SwerveModulePositions.LEFT_FRONT),
                        new SwerveModuleState(new Number(output), Rotation2d.fromDegrees(0), SwerveModuleState.SwerveModulePositions.LEFT_BACK),
                        new SwerveModuleState(new Number(output), Rotation2d.fromDegrees(0), SwerveModuleState.SwerveModulePositions.RIGHT_FRONT),
                        new SwerveModuleState(new Number(output), Rotation2d.fromDegrees(0), SwerveModuleState.SwerveModulePositions.RIGHT_BACK)};

                SmartDashboard.putNumber("AutoLevelOutputEffort", output);

                setSwerveModuleStates(swerveModuleState);

                if(Math.abs(angle.getValue(DEGREE)) < 2) {
                    if (!mAutoLevelTimerStarted) {
                        mAutoLevelSustainedTargetTimer.reset();
//                        mAutoLevelSustainedTargetTimer.start();
                        mAutoLevelTimerStarted = true;
                    } else if (mAutoLevelSustainedTargetTimer.hasElapsed(3)) {
                        mAutoLevelSustainedTargetTimer.stop();
                        step.changeStepState(Constants.StepState.STATE_FINISH);

                        xLock();

                        leftFrontModule.setRawDriveSpeed(0.0);
                        leftBackModule.setRawDriveSpeed(0.0);
                        rightFrontModule.setRawDriveSpeed(0.0);
                        rightBackModule.setRawDriveSpeed(0.0);
                    }
                } else if (mAutoLevelTimerStarted) {
                    mAutoLevelSustainedTargetTimer.stop();
                    mAutoLevelTimerStarted = false;
                }
                break;
        }
    }

    private boolean mDriveOnChargePeakFlag = false;

    /**
     * Instant Command - Used for the FRC 2023 season in conjunction with the AutoLevel command. This command moves the
     * robot until it is on the charge station.
     * @param step - Step to run this command with.
     *             Speed  = Speed that the robot initially uses to get on charge station
     *             Parm 1 = Slower Speed once robot hits peak angle
     */
    @InstantCommand
    public void DriveOnChargeStep(Step step) {
        switch(step.getStepState()) {
            case STATE_INIT:

                SwerveModuleState[] moduleStates = calculateSwerveMotorSpeedsFieldCentric(new Number(-step.getSpeed()), new Number(0), new AngularVelocity(0, DEGREE_PER_SECOND), new Number(1));
                this.setSwerveModuleStates(moduleStates);

                step.changeStepState(Constants.StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                Angle angle = new Angle(-(pigeonIMU.getRoll() - mInitialPitch), DEGREE);

                if(angle.getValue(DEGREE) < -14.0 && !mDriveOnChargePeakFlag) { //&& mAutoLevelAtTarget) {
                    mDriveOnChargePeakFlag = true;

                    if(step.hasValue("parm1") && step.getParm(1) != 0) {
                        SwerveModuleState[] slowDownState = calculateSwerveMotorSpeedsFieldCentric(new Number(-step.getParm(1)), new Number(0), new AngularVelocity(0, DEGREE_PER_SECOND), new Number(1));
                        this.setSwerveModuleStates(slowDownState);
                    }
                }

                if(mDriveOnChargePeakFlag && angle.getValue(DEGREE) > -8) {
                    this.setSpeed(0);
                    xLock();

                    step.changeStepState(Constants.StepState.STATE_FINISH);
                }

                break;
        }
    }

    private int mDriveOverChargeStationStepState = 0;
    private Angle mDriveOverChargeStationLastAngle; //High peak, Low peak, High peak, Low peak
    
    @InstantCommand
    public void DriveOverChargeStep(Step step) {
        switch(step.getStepState()) {
            case STATE_INIT:

                SwerveModuleState[] moduleStates = calculateSwerveMotorSpeedsFieldCentric(new Number(-step.getSpeed()), new Number(0), new AngularVelocity(0, DEGREE_PER_SECOND), new Number(1));
                this.setSwerveModuleStates(moduleStates);

                mDriveOverChargeStationStepState = 0;
                mDriveOverChargeStationLastAngle = new Angle(-(pigeonIMU.getRoll() - mInitialPitch), DEGREE);

                step.changeStepState(Constants.StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                Angle currentAngle = new Angle(-(pigeonIMU.getRoll() - mInitialPitch), DEGREE);

                if(currentAngle.getValue(DEGREE) < -14.0 && mDriveOverChargeStationStepState == 0) {
                    mDriveOverChargeStationStepState++;
                }

                if(currentAngle.getValue(DEGREE) > 4.0 && mDriveOverChargeStationLastAngle.getValue(DEGREE) < -4.0 && mDriveOverChargeStationStepState == 1) {
                    mDriveOverChargeStationStepState++;
                }

                if(currentAngle.getValue(DEGREE) < 14.0 && mDriveOverChargeStationStepState == 2) {
                    mDriveOverChargeStationStepState++;
                }

                if(Math.abs(currentAngle.getValue(DEGREE)) < 2.0 && mDriveOverChargeStationStepState == 3) {
                    this.setSpeed(0);

                    step.changeStepState(Constants.StepState.STATE_FINISH);
                }

                mDriveOverChargeStationLastAngle = currentAngle;
                break;
        }
    }

    @InstantCommand
    private void setDrivetrainVelocity(Step step) {
        switch (step.getStepState()) {
            case STATE_INIT:
                SwerveModuleState[] states = calculateSwerveMotorSpeedsFieldCentric(new Velocity(step.getXDistance(), FEET_PER_SECOND), new Velocity(step.getYDistance(), FEET_PER_SECOND), new AngularVelocity(step.getParm(1), DEGREE_PER_SECOND));
                setSwerveModuleStates(states);

                step.StartTimer();
                step.changeStepState(Constants.StepState.STATE_RUNNING);
                break;
            case STATE_RUNNING:
                SwerveModuleState[] states2 = calculateSwerveMotorSpeedsFieldCentric(new Velocity(step.getXDistance(), FEET_PER_SECOND), new Velocity(step.getYDistance(), FEET_PER_SECOND), new AngularVelocity(step.getParm(1), DEGREE_PER_SECOND));
                setSwerveModuleStates(states2);

                if(step.hasTimeoutElapsed()) {
                    step.changeStepState(Constants.StepState.STATE_FINISH);
                    setSpeed(0);
                }
                break;
        }
    }

    public void xLock() {
        leftFrontModule.setModuleAngle(Rotation2d.fromDegrees(45));
        leftBackModule.setModuleAngle(Rotation2d.fromDegrees(-45));
        rightFrontModule.setModuleAngle(Rotation2d.fromDegrees(-45));
        rightBackModule.setModuleAngle(Rotation2d.fromDegrees(45));
    }

    @InstantCommand
    public void xLock(Step step) {
        if(step.getStepState() == Constants.StepState.STATE_INIT) {
            xLock();

            step.changeStepState(Constants.StepState.STATE_FINISH);
        }
    }

    @PersistentCommand
    public void rawDrive(Step step) {
        if (step.getStepState() == Constants.StepState.STATE_INIT) {
            double scaler = step.getParm(2, 1.0);

            Pair<Double, Double> xy = Pair.of(step.getXDistance(), -step.getYDistance());
            xy = Constants.joyStickSlopedDeadband(xy.getFirst(), xy.getSecond(), true, 0.1);
            xy = Pair.of(xy.getFirst() * scaler, xy.getSecond() * scaler);

            double rMag = Math.pow(Math.sin(Constants.deadband(step.getParm(1), 0.08) * (Math.PI / 2)), 5);
            rMag *= scaler;

            if(xy.getFirst() == 0 && xy.getSecond() == 0 && rMag == 0) {
                setSpeed(0);
            } else {
                SwerveModuleState[] states = calculateSwerveMotorSpeedsFieldCentric(new Number(xy.getFirst()), new Number(xy.getSecond()), new Number(rMag), new Number(1));
                setSwerveModuleStates(states);
            }
        }
    }

    @PersistentCommand
    public void AutoScoreAlignment(Step step) {
        if (step.getStepState() == Constants.StepState.STATE_INIT) {
            if(step.getParm(1) > 0.5) {
                step.changeStepState(Constants.StepState.STATE_RUNNING);

                mSeekGameElementPIDController.setSetpoint(0);
            }
        }

        if(step.getStepState() == Constants.StepState.STATE_RUNNING) {

            //Check if driver is still holding the command
            if(step.getParm(1) <= 0.5) {
                //Change step state to INIT (Canceling Current Step)
                step.changeStepState(Constants.StepState.STATE_INIT);

                setSpeed(0);
                return;
            }

            Angle robotAngle = getAngle();

            double omegaVelocity = -mSeekGameElementPIDController.calculate(robotAngle.getValue(DEGREE));;
            double xVelocity;
            double YVelocity;

            if(ReflectiveTape.hasTarget()) {
//                omegaVelocity =
            }

        }
    }

    public void resetOdometry() {
        swerveKinematics.resetSwerveKinematics();
    }

    public PID getThetaPIDController() {
        return mThetaPIDValues;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roll Drivetrain", -(pigeonIMU.getRoll() - mInitialPitch));
    }

    @Override
    public Pose2d<Distance, Angle> getCurrentOdometryPosition() {
        Point2d<Distance> pos = swerveKinematics.getCurrentRobotPosition();
//        fusedTrackingAlgorithm.

        return new Pose2d<>(pos.getX(), pos.getY(), getAngle());
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(Unit<?, ? extends UnitEnum> xMag, Unit<?, ? extends UnitEnum> yMag, AngleUnit<?, ? extends UnitEnum> rMag, Unit<?, ? extends UnitEnum> maximum) {
        SimpleMatrix fieldCentric = Constants.convertFrame(getAngle(), Constants.createFrameMatrix(xMag.getValueInPrimaryUnit(), yMag.getValueInPrimaryUnit(), rMag.getValueInPrimaryUnit()));

        Unit<?, ? extends UnitEnum> x = UnitUtil.create(fieldCentric.get(0, 0), xMag.getPrimaryUnit());
        Unit<?, ? extends UnitEnum> y = UnitUtil.create(fieldCentric.get(1, 0), yMag.getPrimaryUnit());
        Unit<?, ? extends UnitEnum> w = UnitUtil.create(fieldCentric.get(2, 0), rMag.getPrimaryUnit());

        return swerveKinematics.getSwerveModuleState(x, y, w, maximum);
    }

    public SwerveModuleState[] calculateSwerveMotorSpeeds(Unit<?, ? extends UnitEnum> xMag, Unit<?, ? extends UnitEnum> yMag, AngleUnit<?, ? extends UnitEnum> rMag, Unit<?, ? extends UnitEnum> maximum) {
        return swerveKinematics.getSwerveModuleState(xMag, yMag, rMag, maximum);
    }

    public SwerveModuleState[] calculateSwerveMotorSpeedsFieldCentric(Unit<?, ? extends UnitEnum> xMag, Unit<?, ? extends UnitEnum> yMag, AngleUnit<?, ? extends UnitEnum> rMag) {
        if(xMag instanceof Velocity)
            return calculateSwerveMotorSpeedsFieldCentric(xMag, yMag, rMag, maxDrivetrainVelocity);
        if(xMag instanceof Number)
            return calculateSwerveMotorSpeedsFieldCentric(xMag, yMag, rMag, new Number(1));

        return calculateSwerveMotorSpeedsFieldCentric(xMag, yMag, rMag, new Number(Double.POSITIVE_INFINITY));
    }

    public SwerveModuleState[] calculateSwerveMotorSpeeds(Unit<?, ? extends UnitEnum> xMag, Unit<?, ? extends UnitEnum> yMag, AngleUnit<?, ? extends UnitEnum> rMag) {
        if(xMag instanceof Velocity)
            return calculateSwerveMotorSpeeds(xMag, yMag, rMag, maxDrivetrainVelocity);
        if(xMag instanceof Number)
            return calculateSwerveMotorSpeeds(xMag, yMag, rMag, new Number(1));

        return calculateSwerveMotorSpeeds(xMag, yMag, rMag, new Number(Double.POSITIVE_INFINITY));
    }

    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates) {
        for (SwerveModuleState state : swerveModuleStates) {
            switch (state.getPosition()) {
                case LEFT_FRONT:
                    this.leftFrontModule.setSwerveModuleState(state);
                    break;
                case LEFT_BACK:
                    this.leftBackModule.setSwerveModuleState(state);
                    break;
                case RIGHT_FRONT:
                    this.rightFrontModule.setSwerveModuleState(state);
                    break;
                case RIGHT_BACK:
                    this.rightBackModule.setSwerveModuleState(state);
                    break;
            }
        }
    }

    @Override
    public void setLeftSpeed(double speed) {
        this.leftFrontModule.setRawDriveSpeed(speed);
        this.leftBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public void setRightSpeed(double speed) {
        this.rightFrontModule.setRawDriveSpeed(speed);
        this.rightBackModule.setRawDriveSpeed(speed);
    }

    @Override
    public Angle getAngle() {
        double raw = pigeonIMU.getYaw() % 360;
        return new Angle(raw, DEGREE);
    }

    @Override
    public CartesianValue<Angle> getCartesianAngles() {
        return new CartesianValue<>(new Angle(pigeonIMU.getRoll() % 360, DEGREE), new Angle(pigeonIMU.getPitch() % 360, DEGREE), new Angle(pigeonIMU.getYaw() % 360, DEGREE));
    }

    public Velocity getMaxDrivetrainVelocity() {
        return maxDrivetrainVelocity;
    }

    @Override
    public void setAngleOffset(Rotation2d offset) {
        this.pigeonIMU.configMountPoseYaw(offset.getDegrees());
    }

    @Override
    public void configDrivetrainControlType(Constants.DriveControlType control) {
        leftFrontModule.configDrivetrainControlType(control);
        rightFrontModule.configDrivetrainControlType(control);
        leftBackModule.configDrivetrainControlType(control);
        rightBackModule.configDrivetrainControlType(control);
    }

    public Motor.MotorTypes getDrivetrainType() {
        return driveTrainType;
    }

    public PID getSeekGameElementPIDValues() {
        return mSeekGameElementPIDValues;
    }

}
