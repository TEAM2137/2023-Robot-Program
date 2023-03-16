// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.LogFactory;
import frc.robot.functions.io.xmlreader.Entity;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.library.Constants;
import frc.robot.library.Gamepad;
import frc.robot.library.OpMode;

import java.io.File;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.function.Consumer;

public class Robot extends TimedRobot {

  private static OpMode autonomousClass;
  private static OpMode teleopClass;
  private static OpMode testClass;
  private static OpMode disabledClass;

  private static Constants.RobotState lastRobotState;
  public static ArrayList<Consumer<Constants.RobotState>> robotStateSubscribers = new ArrayList<>();

  public static List<EntityGroup> subSystemCallList = new ArrayList<>();
  public static HashMap<String, Consumer<Step>> subSystemCommandList = new HashMap<>();

  public static ArrayList<Step> currentActiveTeleopSteps = new ArrayList<>();
  public static ArrayList<Step> currentActiveAutonomousSteps = new ArrayList<>();
  public static ArrayList<Step> persistenceTeleopSteps = new ArrayList<>();
  public static ArrayList<Step> persistenceAutonomousSteps = new ArrayList<>();

  public static ArrayList<Entity> allEntities = new ArrayList<>();

  public static ScheduledThreadPoolExecutor threadPoolExecutor = new ScheduledThreadPoolExecutor(2);

  public static DateTimeFormatter robotDateTimeFormatter = DateTimeFormatter.ofPattern("MMdd_HHmmss_SSS");

  public static EntityGroup robotEntityGroup;
  public static EntityGroup settingsEntityGroup;

  public static XMLSettingReader settingReader;
  public static LogFactory logFactory;
  public static FileLogger fileLogger;

  public static NetworkTable configurationNetworkTable;

  public static Gamepad primaryController = new Gamepad(0);
  public static Gamepad secondaryController = new Gamepad(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    boolean isSimulation = isSimulation();

//    System.out.println(System.getProperty("user.home"));
//    if(Constants.StandardFileAndDirectoryLocations.GenerateAllStandardDirectories(isSimulation)) { //@TODO replace with driver station value
//      System.out.println("Created new Standard Directories for FRC Robot");
//    } else {
//      System.out.println("Using existing Standard Directories");
//    }

    DriverStation.silenceJoystickConnectionWarning(true);

    logFactory = new LogFactory(10, isSimulation());

    fileLogger = logFactory.buildLogger("Main");
    fileLogger.writeEvent(0, FileLogger.EventType.Status, "Started FileLogger Continuing with code...");

    fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Settings XML File...");
    settingReader = new XMLSettingReader(Constants.StandardFileAndDirectoryLocations.GenericSettings.getFileLocation(isSimulation), logFactory, fileLogger);

    configurationNetworkTable = NetworkTableInstance.getDefault().getTable("XMLConfiguration");
    NetworkTable smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

    autonomousClass = new Autonomous();

    NetworkTableEntry implementChanges = smartDashboardTable.getEntry("ImplementChanges");
    NetworkTableEntry flushConfigurationToXML = smartDashboardTable.getEntry("FlushConfigurationToXML");
    NetworkTableEntry enterConfigurationMode = smartDashboardTable.getEntry("EnterConfigurationMode");
    enterConfigurationMode.setBoolean(false);

    smartDashboardTable.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
      switch (key) {
        case "ImplementChanges":
          if (implementChanges.getBoolean(false)) {
            settingReader.getRobot().pullFromNetworkTable();
            settingReader.getRobot().OnImplement();

            implementChanges.setBoolean(false);
          }
          break;
        case "FlushConfigurationToXML":
          if (flushConfigurationToXML.getBoolean(false)) {
            settingReader.getRobot().pullFromNetworkTable();
            settingReader.getRobot().updateElement();

            StringBuilder builder = new StringBuilder();
            settingReader.getRobot().constructTreeItemPrintout(builder, 0);
            fileLogger.writeLine(builder.toString());

            fileLogger.writeEvent(0, "Writing to XML file....");
            settingReader.write();
            flushConfigurationToXML.setBoolean(false);
          }
          break;
        case "EnterConfigurationMode":
          if (enterConfigurationMode.getBoolean(false)) {
            settingReader.getRobot().addToNetworkTable(configurationNetworkTable);
            implementChanges.setBoolean(false);
            flushConfigurationToXML.setBoolean(false);
          } else {
//            settingReader.getRobot().removeFromNetworkTable();
//            implementChanges.unpublish();
//            flushConfigurationToXML.unpublish();
          }
          break;
      }
    });

    Robot.subSystemCommandList.put("Delay", this::delay);
  }

  private Timer mDelayStepTimer;

  public void delay(Step step) {
    switch(step.getStepState()) {
      case STATE_INIT:
        mDelayStepTimer = new Timer();
        mDelayStepTimer.reset();
        mDelayStepTimer.start();

        step.changeStepState(Constants.StepState.STATE_RUNNING);
        break;
      case STATE_RUNNING:
        if(mDelayStepTimer.hasElapsed(step.getParm(1))) {
          step.changeStepState(Constants.StepState.STATE_FINISH);
          mDelayStepTimer.stop();
        }
        break;
    }
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    for (EntityGroup subSystem : subSystemCallList) {
      subSystem.periodic();
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      if(lastRobotState != null) callEndFunction();
      clearOpModes();

      callStateSubscribers(Constants.RobotState.DISABLED);

      lastRobotState = Constants.RobotState.DISABLED;

      disabledClass = new Disabled();
      disabledClass.init(settingReader, fileLogger);

  }

  @Override
  public void disabledPeriodic() {
    callPeriodicFunction();
  }

  @Override
  public void autonomousInit() {
      if(lastRobotState != null) callEndFunction();
      clearOpModes();

      callStateSubscribers(Constants.RobotState.AUTONOMOUS);
      currentActiveAutonomousSteps.clear();

      lastRobotState = Constants.RobotState.AUTONOMOUS;

      autonomousClass.init(settingReader, fileLogger);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     callPeriodicFunction();
  }

  @Override
  public void teleopInit() {
      if(lastRobotState != null) callEndFunction();
      clearOpModes();

      for(Step step : persistenceTeleopSteps) {
        step.changeStepState(Constants.StepState.STATE_INIT);
      }
      currentActiveTeleopSteps.clear();

      callStateSubscribers(Constants.RobotState.TELEOP);

      lastRobotState = Constants.RobotState.TELEOP;

      teleopClass = new Teleop();
      teleopClass.init(settingReader, fileLogger);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     callPeriodicFunction();
  }

  @Override
  public void testInit() {
      if(lastRobotState != null) callEndFunction();
      clearOpModes();

      callStateSubscribers(Constants.RobotState.TEST);

      lastRobotState = Constants.RobotState.TEST;
      // Cancels all running commands at the start of test mode.
      CommandScheduler.getInstance().cancelAll();

      testClass = new Test();
      testClass.init(settingReader, fileLogger);

  }

  private void callStateSubscribers(Constants.RobotState newState) {
    for (Consumer<Constants.RobotState> consumer : robotStateSubscribers) {
      consumer.accept(newState);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    callPeriodicFunction();
  }

  private void callEndFunction() {
    switch(lastRobotState) {
      case AUTONOMOUS:
        autonomousClass.end();
        break;
      case TELEOP:
        teleopClass.end();
        break;
      case DISABLED:
        disabledClass.end();
        break;
      case TEST:
        testClass.end();
        break;
    }
  }

  private void callPeriodicFunction() {
    switch(lastRobotState) {
      case AUTONOMOUS:
        autonomousClass.periodic();
        break;
      case TELEOP:
        teleopClass.periodic();
        break;
      case DISABLED:
        disabledClass.periodic();
        break;
      case TEST:
        testClass.periodic();
        break;
    }
  }

  private void clearOpModes() {
//    autonomousClass = null; //"Lose" all the pointers to the objects
    teleopClass = null;
    testClass = null;
    disabledClass = null;

    //subSystemCallList.clear();

    System.gc(); //Hopefully Garbage Collector takes Opmodes
  }
}
