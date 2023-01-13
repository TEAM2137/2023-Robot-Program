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

package frc.robot.functions.io;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.library.Constants.RobotState;
import frc.robot.library.units.Time2d;

import java.io.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;

/**
 * FileLogger class - Filelogger creates a local text file on the roborio that then can be read from a laptop
 * or from a filelogger reader application. Inorder to adjust based on the situation of the robot a debug from 1 to 10
 * is given when created so only some messages write at certain debug levels or higher. Each line is also given a time
 * stamp that allows for the user to find when the robot had a fault. One final note at debug level 10 flushing is
 * enabled at every write making the program much slower
 *
 * Created by: Wyatt Ashley 2022
 */
public class FileLogger {
    public enum EventType {
        Debug ("Debug: "),
        Error ("ERROR: "),
        Warning ("Warning: "),
        Status ("Status: ");

        String name = "";

        EventType(String string) {
            this.name = string;
        }

        public String toString() {
            return this.name;
        }
    }

    private final int debug;
    private static FileWriter writer;
    private final String logName;
    private final String logFileDirectory;
    private final int maxLogFiles = 5;

    private boolean copyToSystemStream = true;

    private final DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("MMdd_HHmmss_SSS");
    private String tagString = "";

    private static ScheduledThreadPoolExecutor flushExecutor;

    /**
     * Constructor for Filelogger, (creates file, starts flushing thread)
     * @param _debug - Debug choice
     * @param robotState - Current robot state (Auto, Tele)
     */
    public FileLogger(int _debug, RobotState robotState, String directory, boolean _copyToSystemStream){
        this.debug = _debug;
        copyToSystemStream = _copyToSystemStream;

        //startThreadedNetworkTable(100);

        this.logName = "log_" + robotState.toString() + "_" + getDateString() + ".txt";

//        this.logFileDirectory = System.getProperty("user.dir") + "\\File Logger\\";
        this.logFileDirectory = directory;
//        this.logFileDirectory = "/home/lvuser/File Logger/";

        try {
            writer = new FileWriter(this.logFileDirectory + this.logName);
            writer.write("MMdd_HHmmss_SSS-Type: Event\n");
            flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
        startThreadedFlush();
        cleanLogs();
    }

    private synchronized void startThreadedFlush() {
        writeEvent(0, EventType.Status, "Starting Threaded File Flush with 100ms periods");
        flush();
        flushExecutor = new ScheduledThreadPoolExecutor(1);
        flushExecutor.scheduleAtFixedRate(this::flush, 0, 100, TimeUnit.MILLISECONDS);
    }

    /**
     * Function that instead of flushing at every write, writes only after certain time
     * @param periodTime - Time between writes and often should be same run period as robot
     */
    private synchronized void startThreadedNetworkTable(int periodTime) {
//        flushExecutor.scheduleAtFixedRate(this::logNetworkTables, 0, periodTime, TimeUnit.MILLISECONDS);
    }

    /**
     * Flushes all the contents in the buffer into the file
     */
    public synchronized void flush() {
        try {
            writer.flush();
        } catch (Exception e) {
            System.out.println("Failed to flush FileWriter data");
            DriverStation.reportWarning("Failed to flush FileWriter data", false);
            e.printStackTrace();
        }
    }

    private String getDateString() {
        return dateTimeFormatter.format(LocalDateTime.now());
    }

    private synchronized void writingAction(int _debug, Runnable runnable) {
        if(_debug >= debug)
            runnable.run();
    }

    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     *
     * @param _debug
     * @param Title
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Title, String Message){
        writeEvent(_debug, Title + "-" + Message);
    }

    /**
     * Writes an event to a log file stored on the directory given
     * Writes only if the given debug level is above the one given in the contructor
     * The event type is used as the Title for the log
     *
     * @param _debug
     * @param event
     * @param Message
     */
    public synchronized void writeEvent(int _debug, EventType event, String Message) {
        writeEvent(_debug, event.toString() + Message);
    }

    /**
     * Writes an event to a glof file stored on the diectory given
     * Writes only if the given debug level is above the one given in the contructor
     * Only writes the message with no Title
     *
     * @param _debug
     * @param Message
     */
    public synchronized void writeEvent(int _debug, String Message) {
        if (_debug <= this.debug) {
            writeLine(Message);
        }
    }

    /**
     * Writes an event to a log file on the directory given
     * Does not start a new line in the file
     * @param Message
     */
    public synchronized void write(String Message) {

        if (!getTag().equals(""))
            rawWrite(getDateString() + "~" + getTag() + "~"  + Message);
        else
            rawWrite(getDateString() + "-" + Message);
    }

    /**
     * Used to add information to the log and is the lowest level function.
     * @param message
     */
    public synchronized void rawWrite(String message) {
        try {
            writer.write(message);
            if(copyToSystemStream)
                System.out.print(message);

        } catch (IOException e) {
            System.out.println("Error Writing To File Logger!!");
            e.printStackTrace();
        }

        if(debug >= 10)
            flush();
    }

    /**
     * Internal function that writes a single line to the file and able to handle errors thrown
     * @param toWrite - String to write
     */
    public synchronized void writeLine(String toWrite) {
        write(toWrite + "\n");
    }

    public void runnableLogAction(int minDebug, Runnable run) {
        if (minDebug >= debug)
            run.run();
    }

    /**
     * List all the files in the Log directory
     * Then if there is more than the max log amount then delete
     */
    private void cleanLogs(){
        File[] fileList = new File(logFileDirectory).listFiles();

        Arrays.sort(fileList, Comparator.comparing(File::lastModified).reversed());

        if (fileList.length > maxLogFiles) {
            writeEvent(0, EventType.Status, "Cleaning old logs...");

            for (int i = maxLogFiles; i < fileList.length; i++) {
                fileList[i].delete();
            }
        }
    }

    /**
     * Close the FileLogger writer
     */
    public synchronized void close(){
        flushExecutor.shutdown();

        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets the tag a small string that goes infront of any logs made
     * @return - String Tag
     */
    public String getTag() {
        return tagString;
    }

    /**
     * Set the tag or a small string that goes infront of any logs made
     * @param name - Name of an action or place in the code
     */
    public void setTag(String name) {
        tagString = name;
    }
}
