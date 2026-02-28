package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter {
  
  public class Robot extends TimedRobot {

  private GenericHID ControlHandlerD;

    boolean aPressed;
    private Timer timer = new Timer();
  private Timer delay = new Timer();
  private int G = 0;

    
    public void Intake() {
    aPressed = !aPressed;
  }

  private final MotorController m_leftMotor1 = new WPI_VictorSPX(2);
  private final MotorController m_rightMotor1 = new WPI_VictorSPX(3);
  private final MotorController m_leftMotor2 = new WPI_VictorSPX(0);
  private final MotorController m_rightMotor2 = new WPI_VictorSPX(1);
  
  SparkMax m_launchWheel;
  SparkMax m_feedWheel;

  SparkMaxConfig c_launchWheel = new SparkMaxConfig();
  SparkMaxConfig c_feedWheel = new SparkMaxConfig();

@Override
  public void robotInit() {
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);
    ControlHandlerD = new GenericHID(0);

    c_launchWheel.smartCurrentLimit(80);
    c_feedWheel.smartCurrentLimit(80);

    m_launchWheel = new SparkMax(5, MotorType.kBrushed);
    m_feedWheel = new SparkMax(6, MotorType.kBrushed);

    m_GrabberL = new SparkMax(7, MotorType.kBrushless);
    m_GrabberR = new SparkMax(8, MotorType.kBrushless);
    
   // AmpArm = new SparkMax(9, MotorType.kBrushed);


@Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    G = (int)(3+(Math.random()*4));

    if(timer.get() == 15){timer.reset();}

    if(ControlHandlerD.getRawButton(1)){
      m_feedWheel.set(1); 
      if(ControlHandlerD.getRawButtonPressed(1)){
      delay.reset();
      delay.start();
      }
      if(delay.get() >= 1.5 && (ControlHandlerD.getRawButton(1))) {m_launchWheel.set(1);}
    }
    else if(ControlHandlerD.getRawButton(2)){m_feedWheel.set(-0.5); m_launchWheel.set(-1);}
    else {m_feedWheel.set(0); m_launchWheel.set(0);}
