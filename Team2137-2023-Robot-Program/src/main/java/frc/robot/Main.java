// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.splines.TrapezoidalVelocity;
import frc.robot.library.hardware.DriveTrainSimulation;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.METER;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main
{
    private Main() {}

    public static FileLogger fileLogger;
   /**
    * Main initialization method. Do not perform any initialization here.
    * <p>
    * If you change your main Robot class (name), change the parameter type.
    */
    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
//        DriveTrainSimulation sim = new DriveTrainSimulation();
//        Distance totalDist = new Distance(10, FOOT);
//        TrapezoidalVelocity trap = new TrapezoidalVelocity(totalDist);

//        double pos = 0;
//
//        while(true) {
//            sim.run();

            //System.out.printf("(%.5f, %.9f)\n", sim.mLastRunTime, sim.mVoltage);
//            System.out.println("(" + sim.mLastRunTime + ", " + sim.mVoltage + ")");

//            System.out.printf("(%.5f, %.5f)\n", pos, trap.targetVelocity(new Distance(pos, METER)).getValue(Velocity.VelocityUnits.FEET_PER_SECOND));
//            sim.mStopping = trap.braking(sim.mDistance);

//            pos += 0.1;

//            if(sim.mLastRunTime > 5)
//            if(pos > totalDist.getValue(METER))
//                break;
//        }
    }
}
