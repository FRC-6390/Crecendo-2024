package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.IRBBSensor;

 
public class Climber extends SubsystemBase {
 
  public static TalonFX climbMotorLeft;
  public static TalonFX climbMotorRight;

 
 
  public Climber()
  {

  }
 
  static
  {
    
  climbMotorLeft = new TalonFX(10, Constants.DRIVETRAIN.CANBUS);
  climbMotorRight = new TalonFX(16, Constants.DRIVETRAIN.CANBUS);
  //intakeBeamBreak = new DigitalInput(0);
  TalonFXConfiguration config = new TalonFXConfiguration();
  CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
  curr.SupplyCurrentLimitEnable = true;
  curr.SupplyCurrentLimit = 30;
  
  
  config.withCurrentLimits(curr);

  climbMotorLeft.getConfigurator().apply(config);
  climbMotorRight.getConfigurator().apply(config);

  climbMotorLeft.setNeutralMode(NeutralModeValue.Brake);
  climbMotorRight.setNeutralMode(NeutralModeValue.Brake);
  
  }
 
  //Get value of the intake lift limit switch



 
  
  //Sets the intake rollers
  public void setRollers(double speed){
  climbMotorLeft.set(speed);
  climbMotorRight.set(-speed);

 
  }
 
  //Sets the lift to a certain speed
 
 
  //Gets lift position
 
 
  //Gets roller position
 
 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return climbMotorLeft.getSupplyCurrent();
  }
 
  //Resets lift encoder
 
 
  //Resets roller encoder
 
 
  //Get value of the intake lift limit switch
 
 
  @Override
  public void periodic()
  {
    //System.out.println(getLowerBeamBreak());
  }
}