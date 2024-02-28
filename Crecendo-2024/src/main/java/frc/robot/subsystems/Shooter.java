package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Shooter extends SubsystemBase {
 

  public static TalonFX feedingRollers;
  public static TalonFX leftShooterMotor;
  public static TalonFX rightShooterMotor;
  public static VelocityVoltage vel;
  public static Slot0Configs configs;
  public double setpoint = 0;

 
 
  public Shooter()
  {
   

  }
 
  static
  {
  leftShooterMotor = new TalonFX(Constants.SHOOTER.LEFT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  rightShooterMotor = new TalonFX(Constants.SHOOTER.RIGHT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  vel = new VelocityVoltage(0);
  configs = new Slot0Configs();

  configs.kV = 0.12;
  configs.kP = 0.11;
  configs.kI = 0.48;
  configs.kD = 0.01;

  leftShooterMotor.getConfigurator().apply(configs, 0.050);
  rightShooterMotor.getConfigurator().apply(configs, 0.050);
  }
  public static void setRollers(double speed){

    leftShooterMotor.set(speed);
    rightShooterMotor.set(speed);
    }

  public void setPID(double desireSpeed)
  {
    setpoint = desireSpeed;
    vel.Slot = 0;
    leftShooterMotor.setControl(vel.withVelocity(desireSpeed));
    rightShooterMotor.setControl(vel.withVelocity(desireSpeed));
  }

  public double getRotorVelocity()
  {
    return leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble();
  }
  public boolean atSetpoint()
  {
    return leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble() >= setpoint; 
  }

  public static void stopShooter()
  {
    vel.Slot = 0;
    leftShooterMotor.setControl(vel.withVelocity(0));
    rightShooterMotor.setControl(vel.withVelocity(0));
  }

 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return feedingRollers.getSupplyCurrent();
  }
 
 
 
  @Override
  public void periodic()
  {
  if(setpoint > 0 )
  {
    SmartDashboard.putBoolean("Shooter Ready?", atSetpoint());
  }

  else if(setpoint < 0 )
  {
    SmartDashboard.putBoolean("Shooter Ready?", !atSetpoint());
  }
  }
}