package frc.robot.functions.io;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.library.Constants;

import java.io.File;
import java.io.FilenameFilter;
import java.nio.file.FileAlreadyExistsException;
import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;

import static edu.wpi.first.wpilibj.RobotBase.isSimulation;

public class LogFactory {

    private HashMap<String, FileLogger> loggers = new HashMap<>();
    private static File logDirectory;
    private String pathSeparator = System.getProperty("file.separator");
    private int maxLogFiles = 10;
    private int debugLevel;
    private boolean copyToStream;

    public LogFactory(int _debugLevel, String directory, boolean _copyToStream) {
        debugLevel = _debugLevel;
        copyToStream = _copyToStream;

        String name;
        if(DriverStation.isFMSAttached()) {
            name = "Match" + DriverStation.getMatchNumber() + "Event" + DriverStation.getEventName();
        } else {
            name = Robot.robotDateTimeFormatter.format(LocalDateTime.now());
        }

        logDirectory = new File(directory + name + pathSeparator);

        if(logDirectory.mkdir()) {
            DriverStation.reportWarning("Created Log Directory Named: " + logDirectory.getName(), false);
        } else {
            DriverStation.reportError("Failed to create log directory", true);
        }

        cleanLogs();
    }

    public LogFactory(int debug, boolean copyToStream) {
        this(debug, Constants.StandardFileAndDirectoryLocations.GenericFileLoggerDir.getMainDirectory(isSimulation()), copyToStream);
    }

    public FileLogger buildLogger(String name) {
        if (loggers.containsKey(name)) {
            File[] files = logDirectory.listFiles((file, filename) -> filename.contains(name));
            int maxFileNumber = -1;
            for (File file : files) {
                String filename = file.getName();

                int fileNumber;
                if (filename.length() == name.length())
                    fileNumber = 0;
                else
                    fileNumber = Integer.parseInt(filename.substring(name.length() - 1, filename.length()));

                if (fileNumber > maxFileNumber)
                    maxFileNumber = fileNumber;
            }

            loggers.put(name, new FileLogger(debugLevel, logDirectory.getAbsolutePath() + pathSeparator + name + maxFileNumber + ".txt", copyToStream));
        } else {
            loggers.put(name, new FileLogger(debugLevel, logDirectory.getAbsolutePath() + pathSeparator + name + ".txt", copyToStream));
        }

        return loggers.get(name);
    }

    public FileLogger getLogger(String name) {
        return loggers.get(name);
    }

    private void cleanLogs(){
        File[] directories = logDirectory.getParentFile().listFiles((file, name) -> {
            if(name.matches("...._......_..."))
                System.out.println(name);
            //MMdd_HHmmss_SSS
            return file.isDirectory() && name.matches("...._......_...");
        });

        if(directories == null || directories.length == 0)
            return;

        Arrays.sort(directories, Comparator.comparing(File::lastModified).reversed());

        if (directories.length > maxLogFiles) {
//            Robot.fileLogger.writeEvent(0, FileLogger.EventType.Status, "Cleaning old logs...");

            for (int i = maxLogFiles; i < directories.length; i++) {
                deleteDirectory(directories[i]);
                directories[i].delete();
            }
        }
    }

    private static void deleteDirectory(File file)
    {
        for (File subfile : file.listFiles()) {

            if (subfile.isDirectory()) {
                deleteDirectory(subfile);
            }

            // delete files and empty subfolders
            subfile.delete();
        }
    }

    public void close(){
        loggers.forEach((a, b) -> {
            b.close();
        });
    }
}
