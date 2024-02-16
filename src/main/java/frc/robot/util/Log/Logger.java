package frc.robot.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.lang.reflect.Method;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.Date;
import java.util.Deque;
import java.util.HashMap;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.Callable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Util;

public class Logger {
    public static Logger m_Logger;

    private static DateFormat dateFormat = new SimpleDateFormat("yy_MMM_dd_hh_mm_ss_aa");

    private HashMap<String, Callable<Object>> m_ToLog = new HashMap<String, Callable<Object>>();

    private Deque<String> m_ToWrite = new ArrayDeque<String>();

    public static void log(String Name, String Value) {
        if (m_Logger == null) {
            PLog.info("Logger", "Please Start Logger First");
        }
        m_Logger.m_ToWrite.add("[" + Name + "] " + Value);
    }

    public enum type {
        none, 
        minimal, 
        debug,
    }

    private Logger(type LogType) {
        addClassToLog(LogType, Shooter.get(), Swerve.get(), Intake.get(), Climber.get());
        dateFormat.setTimeZone(TimeZone.getTimeZone("PST"));
        Thread main = new Thread(new LogThread());
        main.start();
    }

    private void addClassToLog(type logtype, Object... objectsToAdd) {
        for (Object objectToAdd : objectsToAdd) {
            for (Method method : objectToAdd.getClass().getDeclaredMethods()) {
                if (method.isAnnotationPresent(Log.class))
                    m_ToLog.put(objectToAdd.getClass().getName() + method.getName(), ()->method.invoke(objectToAdd));    
                
                if (logtype == type.debug)
                    if (method.isAnnotationPresent(Debug.class))
                        m_ToLog.put("[DEBUG] " + objectToAdd.getClass().getSimpleName() + "/" + method.getName(), ()->method.invoke(objectToAdd));    
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
            Util.sleep(2000);
            PLog.info("Logger","Started Logging");
            FileWriter writer;
            try {
                String logPath = getDirectory();
                File logDir = new File(logPath);
                logDir.mkdir();
                writer = new FileWriter(logPath + "/main.log");
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed To Write File", e);
                return;
            }


            while (true) {
                String time = dateFormat.format(new Date(System.currentTimeMillis()));
                try {
                    for (Map.Entry<String,Callable<Object>> func : m_ToLog.entrySet()) {
                        String Name = func.getKey();
                        String Key = func.getValue().call().toString();
                        SmartDashboard.putString(Name, Key);
                        m_ToWrite.add(time + " -> ["+ Name + "] " + Key + "\n");
                       
                    }
                    while (m_ToWrite.size() > 0) {
                        writer.write(m_ToWrite.pop());
                    }                    
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
            } catch (Exception e) {
                PLog.fatalException("Logger", "Failed to close Writer", e);
            }
            PLog.info("Logger", "Closing Logger");
            m_ToWrite.clear();
        }
    }

    private static String getBaseDirectory() {
        if (Robot.isSimulation()) 
            return "src/main/deploy/logs/";

        return "home/lvuser/logs";
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

        // get system time in milliseconds and convert to datetime
        Date startTime = new Date(System.currentTimeMillis());

        // format time in datetime and add to file name
        return mBaseDirectory + "/(" + maxNum.toString() + ") " + dateFormat.format(startTime);
    }

}
