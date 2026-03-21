package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

//Declare most of robot here (Subsystems, Commands, Buttons, etc). Limit logic to Robot.Java
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    /*private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;*/

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 5);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private static final Swerve s_Swerve = new Swerve();

    public static Swerve getSwerve() {
        return s_Swerve; 
    }

    /* Shooter Button */
    //private final JoystickButton ShootyMcShootface = new JoystickButton(driver, XboxController.Button.kB.value);

    //Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        
        //Sets Axis of Driving & Field Relative
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                /*s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis),
                () -> driver.getRawAxis(rotationAxis),
                () -> false)); */
                  s_Swerve, 
                () -> driver.getRawAxis(1), 
                () -> driver.getRawAxis(0),
                () -> driver.getRawAxis(2),
                () -> false,
                () -> driver.getRawAxis(3))); 

        configureButtonBindings();
    }

    //Configures our one and only button binding
    private void configureButtonBindings() {
        //Button for resetting the Gyro
        zeroGyro.onTrue(s_Swerve.zeroHeading());
    }

    //Autonomous Command (In progress)
    public Command getAutonomousCommand()
    {
        try{
            PathPlannerPath MethuselahPathTraj = PathPlannerPath.fromChoreoTrajectory("Methuselah");
            return AutoBuilder.followPath(MethuselahPathTraj);
        } catch (Exception e) {
            DriverStation.reportError("Error: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}