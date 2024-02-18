package frc.robot.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Date;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Util;

public class Logger {
    public static Logger m_Logger;

    private static final String[] m_DriveRoots = { "/media/sdb1/", "/media/sda1/", "/media/sdc1/" };

    private static final DateFormat dateFormat = new SimpleDateFormat("yy-MMM-dd-hh-mm-ss-SS-aa");
    private static final DateFormat simpleDateFormat = new SimpleDateFormat("mm:ss.SSS");

    private HashMap<String, Callable<Object>> m_ToLog = new HashMap<String, Callable<Object>>();

    private Deque<String> m_ToWrite = new ArrayDeque<String>();
    private Deque<String> m_ToWriteCSV = new ArrayDeque<String>();

    public static void log(String Name, String Value) {
        m_Logger.m_ToWrite.add(getSimpleCurrentTimeFormatted() + " -> [" + Name + "] " + Value + "\n");
    }

    public enum type {
        none, 
        minimal, 
        debug,
    }

    private Logger(type LogType) {
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        Thread main = new Thread(new LogThread());
        addClassToLog(LogType, Shooter.get(), Swerve.get(), Intake.get(), Climber.get());
        main.start();
    }

    private void addClassToLog(type logtype, Object... objectsToAdd) {
        for (Object objectToAdd : objectsToAdd) {
            for (Method method : objectToAdd.getClass().getDeclaredMethods()) {
                if (method.isAnnotationPresent(Log.class))
                    m_ToLog.put(objectToAdd.getClass().getName() + method.getName(), ()->method.invoke(objectToAdd));    
                
                if (logtype == type.debug)
                    if (method.isAnnotationPresent(Debug.class))
                        m_ToLog.put("[D] " + objectToAdd.getClass().getSimpleName() + "/" + method.getName(), ()->method.invoke(objectToAdd));    
            }
        }
    }

    public static void start(type LogType) {
        if (LogType == type.none) return;

        m_Logger = new Logger(LogType);
    }

    private class LogThread implements Runnable {

        @Override
        public void run() {
            FileWriter writer;
            FileWriter writerCSV;
            String logPath;
            try {
                logPath = getDirectory();
                File logDir = new File(logPath);
                logDir.mkdir();
                writer = new FileWriter(logPath + "/main.log");
                writerCSV = new FileWriter(logPath + "/values.csv");
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed To Write File", e);
                return;
            }

            PLog.info("Logger","Started Logging at " + logPath);
            Util.sleep(100);

            String headers = "time,";
            for (Map.Entry<String,Callable<Object>> func : m_ToLog.entrySet()) {
                headers += func.getKey() + ",";
            }
            try {
                writerCSV.write(headers + "\n");
                writerCSV.flush();
            }catch (Exception e) {
                PLog.fatalException("Logger", "Failed to write headers", e);
            }

            while (true) {

                String time = getSimpleCurrentTimeFormatted();
                try {
                    m_ToWriteCSV.add(time);
                    for (Map.Entry<String,Callable<Object>> func : m_ToLog.entrySet()) {
                        String Name = func.getKey();
                        String Key = (func.getValue().call().toString());
                        SmartDashboard.putString(Name, Key);
                        m_ToWriteCSV.add("\"" + Key + "\"");
                    }
                    while (m_ToWrite.size() > 0) {
                        writer.write(m_ToWrite.pop());
                    }          
                    while (m_ToWriteCSV.size() > 0) {
                        writerCSV.write(m_ToWriteCSV.pop() + ",");
                    }          
                    writerCSV.write("\n");
                    writerCSV.flush();
                    writer.flush();
                }
                catch (Exception e) {
                    PLog.fatalException("Logger", "Failed to log value", e);
                    break;
                }
                Util.sleep(200);
            }
            try {
                writer.close();
                writerCSV.close();
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed to close Writer", e);
            }
            PLog.info("Logger", "Closing Logger");
            m_ToWrite.clear();
            m_ToWriteCSV.clear();
        }
    }

    private static String getCurrentTimeFormatted() {
        return dateFormat.format(new Date(System.currentTimeMillis()));
    }

    private static String getSimpleCurrentTimeFormatted() {
        return simpleDateFormat.format(new Date(System.currentTimeMillis()));
    }

    private static String getBaseDirectory() { 
        if (Robot.isSimulation()) 
            return "src/main/deploy/Logs/";

        for (String potential : m_DriveRoots) { 
            if (Files.exists(Paths.get(potential))) {
                return potential;
            }
        }

        return "home/lvuser/Logs/";
    }

    private static String getDirectory() { 
        String mBaseDirectory = getBaseDirectory();
            
        File rootDirectory = new File(mBaseDirectory);
        if (!rootDirectory.isDirectory()) {
            rootDirectory.mkdir();
        }

        // count up previous logging session and number this session accordingly
        Integer maxNum = 0;
        for (final File entry : rootDirectory.listFiles()) {
            try {
                if (!entry.isDirectory()) {
                    continue;
                }
                String directory_name = entry.getName();
                int char_index = directory_name.indexOf(")");
                int num = Integer.parseInt(directory_name.substring(1, char_index));
                if (num > maxNum) {
                    maxNum = num;
                }
            } catch (Exception e) {
                // Files that are not numbers are expected and ignored
            }
        }
        maxNum++;

        char matchType;
        switch (DriverStation.getMatchType()) {
          case Practice:
            matchType = 'P';
            break;
          case Qualification:
            matchType = 'Q';
            break;
          case Elimination:
            matchType = 'E';
            break;
          default:
            matchType = '_';
            break;
        }
        if (DriverStation.isFMSAttached()) {
            return String.format("%s(%d) %s: %s %d", mBaseDirectory, maxNum, DriverStation.getEventName(), matchType, DriverStation.getMatchNumber()); 
        } else {
            return String.format("%s(%d) %s", mBaseDirectory, maxNum, getCurrentTimeFormatted()); 
        }
    }

}
