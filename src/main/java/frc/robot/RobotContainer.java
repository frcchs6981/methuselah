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

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 2);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private static final Swerve s_Swerve = new Swerve();
    public static Swerve getSwerve() { return s_Swerve; }


    /* Shooter Button */
    //private final JoystickButton ShootyMcShootface = new JoystickButton();

    //Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        
        //Sets Axis of Driving & Field Relative
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                  s_Swerve, 
                () -> driver.getRawAxis(1), //FB move
                () -> driver.getRawAxis(0), //LR move
                () -> driver.getRawAxis(4), //Rot
                () -> false, //RobotCentric
                () -> driver.getRawAxis(3))); //Speed 

        zeroGyro.onTrue(s_Swerve.zeroHeading());
    }

    //TODO: Autonomous Command
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