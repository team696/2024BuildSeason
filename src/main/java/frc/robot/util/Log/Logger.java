package frc.robot.util.Log;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Logger {
    public static Logger m_Logger;

    HashMap<String, Callable<Object>> m_ToLog = new HashMap<String, Callable<Object>>();

    public enum type {
        none, 
        minimal, 
        debug,
    }

    private Logger(type LogType) {
        addClassToLog(LogType, Shooter.get(), Swerve.get(), Intake.get(), Climber.get());

        Thread main = new Thread(new LogThread());
        main.start();
    }

    private void addClassToLog(type logtype, Object... objectsToAdd) {
        for (Object objectToAdd : objectsToAdd) {
            for (Method method : objectToAdd.getClass().getDeclaredMethods()) {
                if (method.isAnnotationPresent(ILog.class))
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
            try {
                Thread.sleep(2000);
            } catch (Exception e) {
                Log.fatalException("Logger", "Failed To Sleep", e);
            }
            Log.info("Logger","Started Logging");
            while (true) {
                try {
                    //for (Map.Entry<String,Callable<Object>> func : m_ToLog.entrySet()) {
                    //    Log.info(func.getKey(), func.getValue().call().toString());
                    //}

                    for (Map.Entry<String,Callable<Object>> func : m_ToLog.entrySet()) {
                        SmartDashboard.putString(func.getKey(), func.getValue().call().toString());
                    }
                }
                catch (Exception e) {
                    Log.fatalException("Logger", "Failed to log value", e);
                    Log.info("Logger", "Closing Logger");
                    break;
                }
            }
        }
    }

}
