package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class Controls {
    public static final Joystick joystickPanel = new Joystick(0);
    public static final Joystick operatorPanel = new Joystick(1);
    public static final Joystick operatorPanelB = new Joystick(2);

    public static final CommandXboxController controller = new CommandXboxController(5);

    public static final JoystickButton leftJoy = new JoystickButton(joystickPanel, 1);
    public static final JoystickButton rightJoy = new JoystickButton(joystickPanel, 2);

    public static final JoystickButton Shoot = new JoystickButton(operatorPanel, 1);
    public static final JoystickButton Amp = new JoystickButton(operatorPanel, 2);
    public static final JoystickButton ExtraA = new JoystickButton(operatorPanel, 3);
    public static final JoystickButton Trap = new JoystickButton(operatorPanel, 4);
    public static final JoystickButton ExtraB = new JoystickButton(operatorPanel, 6);
    public static final JoystickButton Ground = new JoystickButton(operatorPanel, 7);
    public static final JoystickButton Source = new JoystickButton(operatorPanel, 8);
    public static final JoystickButton Rollers = new JoystickButton(operatorPanel, 9);
    public static final JoystickButton Drop = new JoystickButton(operatorPanel, 10);
    public static final JoystickButton ExtraC = new JoystickButton(operatorPanel,11);

    public static final JoystickButton Climb = new JoystickButton(operatorPanelB, 2);
    public static final JoystickButton OhShit = new JoystickButton(operatorPanelB,   3);
    public static final JoystickButton Rightest = new JoystickButton(operatorPanelB, 4);
    public static final JoystickButton Right = new JoystickButton(operatorPanelB, 5);
    public static final JoystickButton Left = new JoystickButton(operatorPanelB, 6);
    public static final JoystickButton Gyro = new JoystickButton(operatorPanelB, 7);
    public static final JoystickButton Leftest = new JoystickButton(operatorPanelB, 8);
}
