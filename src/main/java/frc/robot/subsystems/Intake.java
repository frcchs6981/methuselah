package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class Intake {
  //Lifted from "Splinter"

  private GenericHID ControlHandlerD;

  SparkMax m_Rintake1;
  SparkMax m_Rintake2;
  SparkMaxConfig c_Rintake1 = new SparkMaxConfig();
  SparkMaxConfig c_Rintake2 = new SparkMaxConfig(); 

  //Sets up all needed configs and ports
  public void shooterInit() {
    ControlHandlerD = new GenericHID(0);

    c_Rintake1.smartCurrentLimit(80);
    c_Rintake2.smartCurrentLimit(80);

    m_Rintake1 = new SparkMax(1, MotorType.kBrushless);
    m_Rintake2 = new SparkMax(2, MotorType.kBrushless);
  }

  public void shooterPeriodic() {
    //Sets Launch Speed based on Current Left Trigger
     m_Rintake1.set(ControlHandlerD.getRawAxis(2)*0.8);

    //Sets Toggle to A button
    if(ControlHandlerD.getRawButton(1))
    {
      m_Rintake2.set(0.65); 
    }
    else 
    {
      m_Rintake2.set(0);
    }
  }
  public void shooterAutoPeriodic() {
    //Sets Launch Speed to 0.8
     m_Rintake1.set(0.8);
  }
}