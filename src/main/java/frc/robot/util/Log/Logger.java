package frc.robot.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.Deque;
import java.util.List;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.Callable;
import java.util.regex.Matcher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.Util;

public class Logger {
    public static Logger m_Logger;
    private Thread main;

    private static final String[] m_DriveRoots = { "/media/sdb1/", "/media/sda1/", "/media/sdc1/" };

    private static final DateFormat dateFormat = new SimpleDateFormat("yy-MMM-dd-hh-mm-ss-SS-aa");
    private static final DateFormat simpleDateFormat = new SimpleDateFormat("mm:ss.SSS");

    //private HashMap<String, Callable<Object>> m_ToLog = new HashMap<String, Callable<Object>>();
    private List<Map.Entry<String, Callable<Object>>> m_ToLog = new ArrayList<Map.Entry<String, Callable<Object>>>();

    private Deque<String> m_ToWrite = new ArrayDeque<String>();
    private Deque<String> m_ToWriteCSV = new ArrayDeque<String>();

    private type m_logType = type.none;

    private String curDirectory = "";

    public static void log(String Name, String Value) {
        m_Logger.m_ToWrite.add(getSimpleCurrentTimeFormatted() + " -> [" + Name + "] " + Value + "\n");
    }

    public enum type {
        none, 
        minimal, 
        debug,
    }

    private Logger(type LogType) {
        m_logType = LogType;
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        main = new Thread(new LogThread());
    }

    private void addClassToLog(type logtype, Object... objectsToAdd) {
        for (Object objectToAdd : objectsToAdd) {
            for (Method method : objectToAdd.getClass().getDeclaredMethods()) {
                if (method.isAnnotationPresent(Log.class))
                    m_ToLog.add(Map.entry(objectToAdd.getClass().getSimpleName() + "/" + method.getName(), ()->method.invoke(objectToAdd)));    
                
                if (logtype == type.debug)
                    if (method.isAnnotationPresent(Debug.class))
                        m_ToLog.add(Map.entry(objectToAdd.getClass().getSimpleName() + "/D/" + method.getName(), ()->method.invoke(objectToAdd)));    
            }
        }
    }

    public static void registerLoggable(type logType, String name, Callable<Object> func) {
        if(m_Logger == null) {
            PLog.info("Logger", "Logger Not Started.");
            return;
        }

        if(logType == type.none) return;

        if (logType == type.debug)
            m_Logger.m_ToLog.add(Map.entry("D/" + name, func));  
        else 
            m_Logger.m_ToLog.add(Map.entry(name, func));    

        if (m_Logger.curDirectory != "")
            m_Logger.updateHeaders(m_Logger.curDirectory);
    } 

    public static Logger init(type LogType, Object... objects) {
        if (m_Logger != null) {
            PLog.info("Logger", "Don't Reinitialize The Logger");
            return m_Logger;
        }

        m_Logger = new Logger(LogType);
        m_Logger.addClassToLog(m_Logger.m_logType, objects);
        return m_Logger;
    }

    public void start() {
        if (main.isAlive()) {
            PLog.info("Logger", "Don't Start The Logger Twice");
            return;
        }
        main.start();
    }

    private class LogThread implements Runnable {

        @Override
        public void run() {
            FileWriter writer;
            FileWriter writerCSV;
            String headers = "time,";
            try {
                curDirectory = getDirectory();
                File logDir = new File(curDirectory);
                logDir.mkdir();
                writer = new FileWriter(curDirectory + "/main.log");
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed To Write File", e);
                return;
            }

            PLog.info("Logger","Started Logging at " + curDirectory);

            for (Map.Entry<String,Callable<Object>> func : m_ToLog) {
                headers += func.getKey() + ",";
            }
            try {
                writerCSV = new FileWriter(curDirectory + "/values.csv", true);
                writerCSV.write(headers + "\n");
                writerCSV.close();
            } catch (Exception e) {

            }
            while (true) {
                String time = getSimpleCurrentTimeFormatted();
                m_ToWriteCSV.add(time);
                for (Map.Entry<String,Callable<Object>> func : m_ToLog) {
                    try {
                        String Name = func.getKey();
                        String Key = (func.getValue().call().toString());
                        SmartDashboard.putString(Name, Key);
                        m_ToWriteCSV.add("\"" + Key + "\"");
                    } catch (Exception e) {
                        PLog.recoverable("Logger", "Failed to log value: " + func.getKey());
                    }
                }
     
                try {
                    writerCSV = new FileWriter(curDirectory + "/values.csv", true);

                    while (m_ToWrite.size() > 0) {
                        writer.write(m_ToWrite.pop());
                    }          
                    while (m_ToWriteCSV.size() > 0) {
                        writerCSV.write(m_ToWriteCSV.pop() + ",");
                    }          
                    writerCSV.write("\n");
                    writer.flush();
                    writerCSV.close();
                }
                catch (Exception e) {
                    PLog.fatalException("Logger", "Failed to log value", e);
                    break;
                }
                Util.sleep(150);
            }
            try {
                writer.close();
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed to close Writer", e);
            }
            PLog.info("Logger", "Closing Logger");
            m_ToWrite.clear();
            m_ToWriteCSV.clear();
        }
    }

    private void updateHeaders(String dir) {
        try{
            String headers = "time,";
            for (Map.Entry<String,Callable<Object>> func : m_ToLog) {
                headers += func.getKey() + ",";
            }

            File file = new File(curDirectory + "/values.csv");
            BufferedReader reader = new BufferedReader(new FileReader(file));

            String words = "", list = "", first = null;
            while ((words = reader.readLine()) != null) {
                if (first == null) first = words;
                list += words + "\r\n";
            }
            reader.close();
            String replacedtext = list.replaceAll(first, Matcher.quoteReplacement(headers));
            FileWriter writer = new FileWriter(curDirectory + "/values.csv", false);
            writer.write(replacedtext);
            writer.close();
        } catch ( Exception e) {
            PLog.fatalException("Logger", "Failed To Update New Headers", e);
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
            return "src/main/Logs/";

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
