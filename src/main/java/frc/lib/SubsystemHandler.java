package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A base for subsystems that handles registration in the constructor, and provides a more intuitive
 * method for setting the default command.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public abstract class SubsystemHandler implements Subsystem, Sendable {
  /** Constructor. Telemetry/log name defaults to the classname. */
  @SuppressWarnings("this-escape")
  public SubsystemHandler() {
    String name = this.getClass().getSimpleName();
    name = name.substring(name.lastIndexOf('.') + 1);
    SendableRegistry.addLW(this, name, name);
    CommandHandler.getInstance().registerSubsystem(this);
  }

  /**
   * Constructor.
   *
   * @param name Name of the subsystem for telemetry and logging.
   */
  @SuppressWarnings("this-escape")
  public SubsystemHandler(String name) {
    SendableRegistry.addLW(this, name, name);
    CommandHandler.getInstance().registerSubsystem(this);
  }

  /**
   * Gets the name of this Subsystem.
   *
   * @return Name
   */
  @Override
  public String getName() {
    return SendableRegistry.getName(this);
  }

  /**
   * Sets the name of this Subsystem.
   *
   * @param name name
   */
  public void setName(String name) {
    SendableRegistry.setName(this, name);
  }

  /**
   * Gets the subsystem name of this Subsystem.
   *
   * @return Subsystem name
   */
  public String getSubsystem() {
    return SendableRegistry.getSubsystem(this);
  }

  /**
   * Sets the subsystem name of this Subsystem.
   *
   * @param subsystem subsystem name
   */
  public void setSubsystem(String subsystem) {
    SendableRegistry.setSubsystem(this, subsystem);
  }

  /**
   * Associates a {@link Sendable} with this Subsystem. Also update the child's name.
   *
   * @param name name to give child
   * @param child sendable
   */
  public void addChild(String name, Sendable child) {
    SendableRegistry.addLW(child, getSubsystem(), name);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Subsystem");

    builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
    builder.addStringProperty(
        ".default",
        () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
        null);
    builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
    builder.addStringProperty(
        ".command",
        () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
        null);
  }
}
