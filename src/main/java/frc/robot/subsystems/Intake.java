package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.spark.SparkMax;

public class Intake {
  //Lifted from "Splinter" & "Timber"

private GenericHID ControlHandlerD;
private final Solenoid m_solenoid1 = new Solenoid(PneumaticsModuleType.REVPH, 0);
private final Solenoid m_solenoid2 = new Solenoid(PneumaticsModuleType.REVPH, 1);
private final Solenoid m_solenoid3 = new Solenoid(PneumaticsModuleType.REVPH, 2);
boolean SolenoidActive;

  SparkMax m_Rintake1;
  SparkMax m_Rintake2;
  SparkMaxConfig c_Rintake1 = new SparkMaxConfig();
  SparkMaxConfig c_Rintake2 = new SparkMaxConfig(); 

  //Sets up all needed configs and ports
  public void intakeInit() {
    ControlHandlerD = new GenericHID(0);

    c_Rintake1.smartCurrentLimit(80);
    c_Rintake2.smartCurrentLimit(80);

    m_Rintake1 = new SparkMax(15, MotorType.kBrushless);
    m_Rintake2 = new SparkMax(16, MotorType.kBrushless);
  }

  public void intakePeriodic() {

    //Sets Toggle to A button
    if(ControlHandlerD.getRawButton(5))
    {
        m_Rintake1.set(0.65); 
        m_Rintake2.set(0.65);
        SolenoidActive = true;
    }
    else
    {
        m_Rintake1.set(0); 
        m_Rintake2.set(0);
        SolenoidActive = false;
    }
    m_solenoid1.set(SolenoidActive);
    m_solenoid2.set(SolenoidActive);
    m_solenoid3.set(SolenoidActive);
  }

  /*public void intakeAutoPeriodic() {
    //Sets Launch Speed to 0.8
     m_Rintake1.set(0.8);
  }*/
}