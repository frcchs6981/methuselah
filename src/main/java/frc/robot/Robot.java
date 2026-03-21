// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Shooter;

//Automatically set to run. See old versions of code for additional details.
public class Robot extends TimedRobot {
  
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private Shooter m_Shooter;
  private RobotContainer m_robotContainer;

  public Robot() {
    super(); 
  }

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    // Instantiates our Shooter
    m_Shooter = new Shooter();
    m_Shooter.shooterInit();
  }
  //Runs at every robot packet in all modes (at the end of mode specific periodic), for diagnostics.
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler (polls buttons, adds, runs, removes, commands, and runns subsystem periodic() methods). Must be called
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    //Cancels Auto
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }
  @Override
  public void teleopPeriodic() {
   m_Shooter.shooterPeriodic();
  }

  @Override
  public void autonomousInit() {
    // This runs the autonomous command selected by your RobotContainer class.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  @Override
  public void autonomousPeriodic() {
    m_Shooter.shooterAutoPeriodic();
  }

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() { CommandScheduler.getInstance().cancelAll(); } // Cancels all running commands at the start of test mode.
  @Override
  public void testPeriodic() {}
}