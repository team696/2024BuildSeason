package frc.lib;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.Log.PLog;

import java.util.ConcurrentModificationException;

/**
 * IterativeRobotBase implements a specific type of robot program framework, extending the RobotBase
 * class.
 *
 * <p>The IterativeRobotBase class does not implement startCompetition(), so it should not be used
 * by teams directly.
 *
 * <p>This class provides the following functions which are called by the main loop,
 * startCompetition(), at the appropriate times:
 *
 * <p>robotInit() -- provide for initialization at robot power-on
 *
 * <p>driverStationConnected() -- provide for initialization the first time the DS is connected
 *
 * <p>init() functions -- each of the following functions is called once when the appropriate mode
 * is entered:
 *
 * <ul>
 *   <li>disabledInit() -- called each and every time disabled is entered from another mode
 *   <li>autonomousInit() -- called each and every time autonomous is entered from another mode
 *   <li>teleopInit() -- called each and every time teleop is entered from another mode
 *   <li>testInit() -- called each and every time test is entered from another mode
 * </ul>
 *
 * <p>periodic() functions -- each of these functions is called on an interval:
 *
 * <ul>
 *   <li>robotPeriodic()
 *   <li>disabledPeriodic()
 *   <li>autonomousPeriodic()
 *   <li>teleopPeriodic()
 *   <li>testPeriodic()
 * </ul>
 *
 * <p>exit() functions -- each of the following functions is called once when the appropriate mode
 * is exited:
 *
 * <ul>
 *   <li>disabledExit() -- called each and every time disabled is exited
 *   <li>autonomousExit() -- called each and every time autonomous is exited
 *   <li>teleopExit() -- called each and every time teleop is exited
 *   <li>testExit() -- called each and every time test is exited
 * </ul>
 */
public abstract class IterativeRobotBase extends RobotBase {
  private enum Mode {
    None,
    Disabled,
    Autonomous,
    Teleop,
    Test
  }

  private final DSControlWord m_word = new DSControlWord();
  private Mode m_lastMode = Mode.None;
  private final double m_period;
  private boolean m_ntFlushEnabled = true;
  private boolean m_lwEnabledInTest;
  private boolean m_calledDsConnected;

  /**
   * Constructor for IterativeRobotBase.
   *
   * @param period Period in seconds.
   */
  protected IterativeRobotBase(double period) {
    m_period = period;
  }

  /** Provide an alternate "main loop" via startCompetition(). */
  @Override
  public abstract void startCompetition();

  /* ----------- Overridable initialization code ----------------- */

  /**
   * Robot-wide initialization code should go here.
   *
   * <p>Users should override this method for default Robot-wide initialization which will be called
   * when the robot is first powered on. It will be called exactly one time.
   *
   * <p>Warning: the Driver Station "Robot Code" light and FMS "Robot Ready" indicators will be off
   * until RobotInit() exits. Code in RobotInit() that waits for enable will cause the robot to
   * never indicate that the code is ready, causing the robot to be bypassed in a match.
   */
  public void robotInit() {}

  /**
   * Code that needs to know the DS state should go here.
   *
   * <p>Users should override this method for initialization that needs to occur after the DS is
   * connected, such as needing the alliance information.
   */
  public void driverStationConnected() {}

  /**
   * Robot-wide simulation initialization code should go here.
   *
   * <p>Users should override this method for default Robot-wide simulation related initialization
   * which will be called when the robot is first started. It will be called exactly one time after
   * RobotInit is called only when the robot is in simulation.
   */
  public void simulationInit() {}

  /**
   * Initialization code for disabled mode should go here.
   *
   * <p>Users should override this method for initialization code which will be called each time the
   * robot enters disabled mode.
   */
  public void disabledInit() {}

  /**
   * Initialization code for autonomous mode should go here.
   *
   * <p>Users should override this method for initialization code which will be called each time the
   * robot enters autonomous mode.
   */
  public void autonomousInit() {}

  /**
   * Initialization code for teleop mode should go here.
   *
   * <p>Users should override this method for initialization code which will be called each time the
   * robot enters teleop mode.
   */
  public void teleopInit() {}

  /**
   * Initialization code for test mode should go here.
   *
   * <p>Users should override this method for initialization code which will be called each time the
   * robot enters test mode.
   */
  public void testInit() {}

  /* ----------- Overridable periodic code ----------------- */

  private boolean m_rpFirstRun = true;

  /** Periodic code for all robot modes should go here. */
  public void robotPeriodic() {
    if (m_rpFirstRun) {
      System.out.println("Default robotPeriodic() method... Override me!");
      m_rpFirstRun = false;
    }
  }

  private boolean m_spFirstRun = true;

  /**
   * Periodic simulation code should go here.
   *
   * <p>This function is called in a simulated robot after user code executes.
   */
  public void simulationPeriodic() {
    if (m_spFirstRun) {
      System.out.println("Default simulationPeriodic() method... Override me!");
      m_spFirstRun = false;
    }
  }

  private boolean m_dpFirstRun = true;

  /** Periodic code for disabled mode should go here. */
  public void disabledPeriodic() {
    if (m_dpFirstRun) {
      System.out.println("Default disabledPeriodic() method... Override me!");
      m_dpFirstRun = false;
    }
  }

  private boolean m_apFirstRun = true;

  /** Periodic code for autonomous mode should go here. */
  public void autonomousPeriodic() {
    if (m_apFirstRun) {
      System.out.println("Default autonomousPeriodic() method... Override me!");
      m_apFirstRun = false;
    }
  }

  private boolean m_tpFirstRun = true;

  /** Periodic code for teleop mode should go here. */
  public void teleopPeriodic() {
    if (m_tpFirstRun) {
      System.out.println("Default teleopPeriodic() method... Override me!");
      m_tpFirstRun = false;
    }
  }

  private boolean m_tmpFirstRun = true;

  /** Periodic code for test mode should go here. */
  public void testPeriodic() {
    if (m_tmpFirstRun) {
      System.out.println("Default testPeriodic() method... Override me!");
      m_tmpFirstRun = false;
    }
  }

  /**
   * Exit code for disabled mode should go here.
   *
   * <p>Users should override this method for code which will be called each time the robot exits
   * disabled mode.
   */
  public void disabledExit() {}

  /**
   * Exit code for autonomous mode should go here.
   *
   * <p>Users should override this method for code which will be called each time the robot exits
   * autonomous mode.
   */
  public void autonomousExit() {}

  /**
   * Exit code for teleop mode should go here.
   *
   * <p>Users should override this method for code which will be called each time the robot exits
   * teleop mode.
   */
  public void teleopExit() {}

  /**
   * Exit code for test mode should go here.
   *
   * <p>Users should override this method for code which will be called each time the robot exits
   * test mode.
   */
  public void testExit() {}

  /**
   * Enables or disables flushing NetworkTables every loop iteration. By default, this is enabled.
   *
   * @param enabled True to enable, false to disable
   */
  public void setNetworkTablesFlushEnabled(boolean enabled) {
    m_ntFlushEnabled = enabled;
  }

  private boolean m_reportedLw;

  /**
   * Sets whether LiveWindow operation is enabled during test mode. Calling
   *
   * @param testLW True to enable, false to disable. Defaults to false.
   * @throws ConcurrentModificationException if this is called during test mode.
   */
  public void enableLiveWindowInTest(boolean testLW) {
    if (isTestEnabled()) {
      throw new ConcurrentModificationException("Can't configure test mode while in test mode!");
    }
    if (!m_reportedLw && testLW) {
      HAL.report(tResourceType.kResourceType_SmartDashboard, tInstances.kSmartDashboard_LiveWindow);
      m_reportedLw = true;
    }
    m_lwEnabledInTest = testLW;
  }
  /**
   * Whether LiveWindow operation is enabled during test mode.
   *
   * @return whether LiveWindow should be enabled in test mode.
   */
  public boolean isLiveWindowEnabledInTest() {
    return m_lwEnabledInTest;
  }

  /**
   * Gets time period between calls to Periodic() functions.
   *
   * @return The time period between calls to Periodic() functions.
   */
  public double getPeriod() {
    return m_period;
  }

  /** Loop function. */
  protected void loopFunc() {
    DriverStation.refreshData();

    m_word.refresh();

    Timer timer = new Timer();

    // Get current mode
    Mode mode = Mode.None;
    if (m_word.isDisabled()) {
      mode = Mode.Disabled;
    } else if (m_word.isAutonomous()) {
      mode = Mode.Autonomous;
    } else if (m_word.isTeleop()) {
      mode = Mode.Teleop;
    } else if (m_word.isTest()) {
      mode = Mode.Test;
    }

    if (!m_calledDsConnected && m_word.isDSAttached()) {
      m_calledDsConnected = true;
      driverStationConnected();
    }

    if (m_lastMode != mode) {
        switch (m_lastMode) {
            case Disabled:
                disabledExit();
                break;
            case Autonomous:
                autonomousExit();
                break;
            case Teleop:
                teleopExit();
                break;
            case Test:
                if (m_lwEnabledInTest) {
                    LiveWindow.setEnabled(false);
                    Shuffleboard.disableActuatorWidgets();
                }
                testExit();
                break;
            case None:
                break;
        }
        timer.mark(m_lastMode.toString() + " Exit");
        switch (mode) {
            case Disabled:
                disabledInit();
                break;
            case Autonomous:
                autonomousInit();
                break;
            case Teleop:
                teleopInit();
                break;
            case Test:
                if (m_lwEnabledInTest) {
                    LiveWindow.setEnabled(true);
                    Shuffleboard.enableActuatorWidgets();
                }
                testInit();
                break;
            case None:
                break;
        }

        timer.mark(mode.toString() + " Init");

        m_lastMode = mode;
    }

    switch (mode) {
        case Disabled:
            DriverStationJNI.observeUserProgramDisabled();
            disabledPeriodic();
            break;
        case Autonomous:
            DriverStationJNI.observeUserProgramAutonomous();
            autonomousPeriodic();
            break;
        case Teleop:
            DriverStationJNI.observeUserProgramTeleop();
            teleopPeriodic();
            break;
        default:
            DriverStationJNI.observeUserProgramTest();
            testPeriodic();
            break;
    }

    timer.mark(mode.toString() + " Periodic");

    robotPeriodic();

    timer.mark("Robot Periodic");

    SmartDashboard.updateValues();
    timer.mark("SmartDashboard.updateValues");
    LiveWindow.updateValues();
    timer.mark("LiveWindow.updateValues");
    Shuffleboard.update();
    timer.mark("Shuffleboard.update");

    if (isSimulation()) {
      HAL.simPeriodicBefore();
      simulationPeriodic();
      HAL.simPeriodicAfter();
      timer.mark("Simulation Periodic");
    }

    // Flush NetworkTables
    if (m_ntFlushEnabled) {
      NetworkTableInstance.getDefault().flushLocal();
    }

    if (timer.overrun(0.2)) {
        PLog.unusual("Robot", "Loop Overrun" + timer.print());
    }
  }
}
