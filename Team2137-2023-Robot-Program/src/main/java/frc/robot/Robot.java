// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.library.Constants;
import frc.robot.library.OpMode;

import java.io.Console;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;

public class Robot extends TimedRobot {

  private static OpMode autonomousClass;
  private static OpMode teleopClass;
  private static OpMode testClass;
  private static OpMode disabledClass;

  private static Constants.RobotState lastRobotState;
  public static List<EntityGroup> subSystemCallList = new ArrayList<>();

  public static XMLSettingReader settingReader;
  public static XMLStepReader stepReader;
  public static FileLogger fileLogger;

  public static NetworkTable configurationNetworkTable;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    boolean isSimulation = isSimulation();

    if(Constants.StandardFileAndDirectoryLocations.GenerateAllStandardDirectories(isSimulation)) { //@TODO replace with driver station value
      System.out.println("Created new Standard Directories for FRC Robot");
    } else {
      System.out.println("Using existing Standard Directories");
    }

    fileLogger = new FileLogger(10, Constants.RobotState.MAIN, Constants.StandardFileAndDirectoryLocations.GenericFileLoggerDir.getFileLocation(isSimulation), isSimulation);
    fileLogger.writeEvent(0, FileLogger.EventType.Status, "Started FileLogger Continuing with code...");


    fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Settings XML File...");
    settingReader = new XMLSettingReader(Constants.StandardFileAndDirectoryLocations.GenericSettings.getFileLocation(isSimulation), true, fileLogger);
    fileLogger.writeEvent(0, FileLogger.EventType.Status, "Opening Step XML File...");
    stepReader = new XMLStepReader(Constants.StandardFileAndDirectoryLocations.GenericStepList.getFileLocation(isSimulation), fileLogger);
    fileLogger.writeEvent(0, "Steps" + stepReader.getSteps().size());

    configurationNetworkTable = NetworkTableInstance.getDefault().getTable("XMLConfiguration");

    NetworkTableEntry implementChanges = configurationNetworkTable.getEntry("ImplementChanges");
    NetworkTableEntry flushConfigurationToXML = configurationNetworkTable.getEntry("FlushConfigurationToXML");
    NetworkTableEntry enterConfigurationMode = configurationNetworkTable.getEntry("EnterConfigurationMode");
    enterConfigurationMode.setBoolean(false);

    configurationNetworkTable.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
      System.out.println("updated: " + key);
      System.out.println("Name: " + enterConfigurationMode.getName());
      switch (key) {
        case "ImplementChanges":
          if (implementChanges.getBoolean(false)) {
            settingReader.getRobot().pullFromNetworkTable(configurationNetworkTable);

            settingReader.getRobot().callOnChange();
            implementChanges.setBoolean(false);
          }
          break;
        case "FlushConfigurationToXML":
          if (flushConfigurationToXML.getBoolean(false)) {
            settingReader.getRobot().pullFromNetworkTable(configurationNetworkTable);
            settingReader.getRobot().updateElement();
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
            settingReader.getRobot().removeFromNetworkTable(configurationNetworkTable);
            implementChanges.unpublish();
            flushConfigurationToXML.unpublish();
          }
          break;
      }
    });
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
      if(lastRobotState != null) callEndFunction();
      clearOpModes();

      lastRobotState = Constants.RobotState.DISABLED;

      disabledClass = new Disabled();
      disabledClass.init(settingReader, stepReader, fileLogger);

  }

  @Override
  public void disabledPeriodic() {
    callPeriodicFunction();
  }

  @Override
  public void autonomousInit() {

      callEndFunction();
      clearOpModes();

      lastRobotState = Constants.RobotState.AUTONOMOUS;

      autonomousClass = new Autonomous();
      autonomousClass.init(settingReader, stepReader, fileLogger);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     callPeriodicFunction();
  }

  @Override
  public void teleopInit() {
      callEndFunction();
      clearOpModes();

      lastRobotState = Constants.RobotState.TELEOP;

      teleopClass = new Teleop();
      teleopClass.init(settingReader, stepReader, fileLogger);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     callPeriodicFunction();
  }

  @Override
  public void testInit() {
      callEndFunction();
      clearOpModes();

      lastRobotState = Constants.RobotState.TEST;
      // Cancels all running commands at the start of test mode.
      CommandScheduler.getInstance().cancelAll();

      testClass = new Test();
      testClass.init(settingReader, stepReader, fileLogger);

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

//    for (EntityGroup subSystem : subSystemCallList) {
//      subSystem.periodic();
//    }
  }

  private void clearOpModes() {
    autonomousClass = null; //"Lose" all the pointers to the objects
    teleopClass = null;
    testClass = null;
    disabledClass = null;

    subSystemCallList.clear();

    System.gc(); //Hopefully Garbage Collector takes Opmodes
  }
}
