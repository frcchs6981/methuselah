package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Shooter {
  //Lifted from "Splinter"

  private GenericHID ControlHandlerD;
  public boolean ShooterAutonmous;

  SparkMax m_launchWheel;
  SparkMax m_feedWheel;
  SparkMaxConfig c_launchWheel = new SparkMaxConfig();
  SparkMaxConfig c_feedWheel = new SparkMaxConfig(); 

  //Sets up all needed configs and ports
  public void shooterInit() {
    ControlHandlerD = new GenericHID(0);

    c_launchWheel.smartCurrentLimit(80);
    c_feedWheel.smartCurrentLimit(80);

    m_launchWheel = new SparkMax(1, MotorType.kBrushless);
    m_feedWheel = new SparkMax(2, MotorType.kBrushless);
  }

  public void shooterPeriodic() {
    //Sets Launch Speed based on Current Left Trigger
    //m_launchWheel.set(ControlHandlerD.getRawAxis(2)*0.8);
 
    //Sets Launch Speed based on three stages (HI, MED, LO)
    if(ControlHandlerD.getRawButton(7))
    {
      m_launchWheel.set(0.85); 
    }
    else if(ControlHandlerD.getRawButton(9))
    {
      m_launchWheel.set(0.7);
    }
    else if(ControlHandlerD.getRawButton(11))
    {
      m_launchWheel.set(0.6);
    }
    else
    {
      m_launchWheel.set(0);
    }

    //Sets Toggle to A button
    if(ControlHandlerD.getRawButton(1))
    {
      m_feedWheel.set(0.65); 
    }
    else 
    {
      m_feedWheel.set(0);
    }
  }
  public void shooterAutoPeriodic() {

    //Sets Feed Wheel and Launch Wheel speed if ShooterAutonomous is true
    if(ShooterAutonmous)
    {
      m_launchWheel.set(0.8);
      m_feedWheel.set(0.65); 
    }
    else 
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
    }
  }
}