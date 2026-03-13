package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.CANSparkLowLevel.MotorType;


public class Shooter {

  private GenericHID ControlHandlerD;

  SparkMax m_launchWheel;
  SparkMax m_feedWheel;
  SparkMax m_GrabberL;
  SparkMax m_GrabberR;
 // SparkMax AmpArm;

  SparkMaxConfig c_launchWheel = new SparkMaxConfig();
  SparkMaxConfig c_feedWheel = new SparkMaxConfig(); 
  //SparkMaxConfig c_GrabberL;
  //SparkMaxConfig c_GrabberR;
  //SparkMaxConfig c_AmpArm;

  public void shooterInit() {
    ControlHandlerD = new GenericHID(0);

    c_launchWheel.smartCurrentLimit(80);
    c_feedWheel.smartCurrentLimit(80);

    m_launchWheel = new SparkMax(1, MotorType.kBrushless);
    m_feedWheel = new SparkMax(2, MotorType.kBrushless);
  }

  public void teleopPeriodic() {

    //Shooter Command 
     m_launchWheel.set(ControlHandlerD.getRawAxis(2)*0.8);

    if(ControlHandlerD.getRawButton(1))
    {
      m_feedWheel.set(0.65); 
    }
    else {m_feedWheel.set(0);}
  }
}
