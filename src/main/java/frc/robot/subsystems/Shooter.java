package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;


public class Shooter {
  //Lifted from "Splinter"

  private GenericHID ControlHandlerD;

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
 
    //Sets Launch Speed
    m_launchWheel.set((0.721738 * Math.log10(16.66 * ControlHandlerD.getRawAxis(2))));

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
  public void shooterAutoPeriodic(boolean top, boolean bottom) {

    //Sets Feed Wheel and Launch Wheel speed if ShooterAutonomous is true
    if(top)
    {
      m_launchWheel.set(1);
      
      if(bottom){
      m_feedWheel.set(0.7); 
      }
    }
    else 
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
    }
  }
}