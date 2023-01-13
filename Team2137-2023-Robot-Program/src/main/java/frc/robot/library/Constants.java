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

package frc.robot.library;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.units.Distance2d;
import org.ejml.simple.SimpleMatrix;

import java.io.File;

public final class Constants {

    //region File Locations
    public enum StandardFileAndDirectoryLocations {
        GenericSettings("Settings.xml"),
        GenericStepList("Steps.xml"),
        GenericFileLoggerDir("\\FileLogs\\"),

        //region 2022 XML File Locations
        TEAM2137RED1_2022("Team2137_2022_Red1_Steps.xml"),
        TEAM2137RED2_2022("Team2137_2022_Red2_Steps.xml"),
        TEAM2137RED3_2022("Team2137_2022_Red3_Steps.xml"),
        TEAM2137BLUE1_2022("Team2137_2022_Blue1_Steps.xml"),
        TEAM2137BLUE2_2022("Team2137_2022_Blue2_Steps.xml"),
        TEAM2137BLUE3_2022("Team2137_2022_Blue3_Steps.xml"),
        TEAM2137Test_2022("Team2137_2022_Test_Steps.xml"),
        TEAM2137Settings_2022("Team2137_2022_Settings.xml")
        //endregion
        ;

        private final String loc;

        //Uses current users home location (for windows C:/Users/Wyatt)
        public static final String xmlDesktopFileLocation = System.getProperty("user.home") + "\\FRC_ROBOT_FILES\\";
        public static final String xmlRoboRioFileLocation = System.getProperty("user.home") + "\\FRC_ROBOT_FILES\\";

        StandardFileAndDirectoryLocations(String location) {
            loc = location;
        }

        /**
         * @param simulation - Boolean whether the robot is running on desktop (if true uses {@see xmlDesktopFileLocation} else {@see xmlRoboRioFileLocation})
         * @return - Full File Location
         */
        public String getFileLocation(boolean simulation) {
            if (simulation) {
                return xmlDesktopFileLocation + loc;
            } else {
                return xmlRoboRioFileLocation + loc;
            }
        }

        /**
         * Creates the new xml standard directories {@see XMLSettingReader}
         * @param simulation - Whether the robot is on desktop or roborio
         * @return - True if directory is created and false for already existing
         */
        public static boolean GenerateBaseXMLDirectories(boolean simulation) {
            File xmlLocation = simulation ? new File(xmlDesktopFileLocation) : new File(xmlRoboRioFileLocation);

            return xmlLocation.mkdir();
        }

        /**
         * Creates the new fileLogger standard directories {@see FileLogger}
         * @param simulation - Whether the robot is on desktop or roborio
         * @return - True if directory is created and false for already existing
         */
        public static boolean GenerateBaseFileLoggerDirectories(boolean simulation) {
            File fileLogLocation = new File(GenericFileLoggerDir.getFileLocation(simulation));

            return fileLogLocation.mkdir();
        }

        /**
         * Creates all standard directories for {@see GenerateBaseXMLDirectories} & {@see GenerateBaseFileLoggerDirectories}
         * @param simulation - Whether code is running on desktop or roborio
         * @return - True if any directories were created
         */
        public static boolean GenerateAllStandardDirectories(boolean simulation) {
            return GenerateBaseXMLDirectories(simulation) | GenerateBaseFileLoggerDirectories(simulation);
        }
    }
    //endregion

    public enum RobotState {
        AUTONOMOUS("Auto"),
        DISABLED("Disa"),
        TELEOP("Tele"),
        MAIN("Main"),
        TEST("Test");

        private final String name;

        RobotState(String _name) {
            name = _name;
        }

        public String toString() {
            return name;
        }
    }

    public enum StepState {
        STATE_INIT ("STATE_INIT"),
        STATE_PAUSED ("STATE_PAUSED"),
        STATE_RUNNING ("STATE_RUNNING"),
        STATE_FINISH ("STATE_FINISHED"),
        STATE_NOT_STARTED ("STATE_NOT_STARTED");

        private String name = "";

        StepState(String name) {
            this.name = name;
        }

        public boolean isFinished() {
            return this == STATE_FINISH;
        }

        public String toString() {
            return this.name;
        }
    }

    public enum DriveControlType {
        RAW ("Raw"),
        VELOCITY ("Velocity"),
        DISTANCE ("Distance");

        private final String name;

        DriveControlType(String _name) {
            this.name = _name;
        }

        public String toString() {
            return name;
        }
    }

    //////Utility Functions//////

    /**
     * input matrix ->
     * [X1][X2]
     * [Y1][Y2]
     * [R1][R2]
     *
     * @param angleRadians - Frame angle offset
     * @param locationMatrix - Matrix with components to be transformed
     * @return - resultant matrix
     */
    public static SimpleMatrix convertFrame(double angleRadians, SimpleMatrix locationMatrix) {
        SimpleMatrix rotationalMatrix = new SimpleMatrix(
                new double[][] {
                        new double[] {Math.cos(angleRadians), Math.sin(angleRadians), 0.0},
                        new double[] {Math.sin(angleRadians), -Math.cos(angleRadians), 0.0},
                        new double[] {0.0, 0.0, 1.0}
                }
        );

        /* Resultant matrix ->
         * [Y1][Y2]
         * [X1][X2]
         * [R1][R2] (Angles)
         */
        SimpleMatrix returner = rotationalMatrix.mult(locationMatrix);

        SimpleMatrix xLine = returner.rows(1,2);
        SimpleMatrix yLine = returner.rows(0,1);
        SimpleMatrix rLine = returner.rows(2,3);

        return xLine.concatRows(yLine, rLine);
    }

    public static SimpleMatrix createFrameMatrix(double x, double y, double r) {
        return new SimpleMatrix(new double[][]{
                new double[] {x},
                new double[] {y},
                new double[] {r}
        });
    }

    public static SimpleMatrix createFrameMatrix(Distance2d x, Distance2d y, Rotation2d r) {
        return createFrameMatrix(x.getValue(Distance2d.DistanceUnits.INCH), y.getValue(Distance2d.DistanceUnits.INCH), r.getRadians());
    }

    public static double deadband(double value, double deadband) {
        if(Math.abs(value) < deadband) {
            return 0;
        } else {
            return value;
        }
    }

    //////Exception Classes//////

    /**
     * PIDSlotIDOutOfRangeException is used by Motor class when slot id provided in XML File is outside acceptable range
     * @see Motor
     */
    public static class PIDSlotIDOutOfRangeException extends Exception {
        public PIDSlotIDOutOfRangeException(String deviceName) {
            super("PID Slot ID is Out of Range in XML File (" + deviceName + ")");
        }
    }

//    /**
//     * MissingDeviceException is used by CANSubsystem class when required hardware is not contain in XML File
//     * @see frc.robot.library.CANSubsystem
//     */
//    public static class MissingDeviceException extends Exception {
//        public MissingDeviceException(CANSubsystem subsystem, String deviceName) {
//            super("Missing Device from XML File in " + subsystem.getName() + ". Please add " + deviceName + " to the XML file");
//        }
//    }
}
