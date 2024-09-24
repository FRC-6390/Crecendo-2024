package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import com.fasterxml.jackson.databind.module.SimpleAbstractTypeResolver;
// import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
  public boolean atSetpoint;
  //public static Follower rightShooterMotor = new Follower(Constants.SHOOTER.LEFT_SHOOTER_MOTOR, true);
 
 
  public Shooter()
  {
   

  }
 
  static
  {
  leftShooterMotor = new TalonFX(Constants.SHOOTER.LEFT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  
  rightShooterMotor = new TalonFX(Constants.SHOOTER.RIGHT_SHOOTER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  //rightShooterMotor.setControl(new Follower(Constants.SHOOTER.LEFT_SHOOTER_MOTOR, true));
  vel = new VelocityVoltage(0);
  
  configs = new Slot0Configs();





  // //0.12
  // configs.kV = 0.12;
  // //0.11
  // configs.kP = 0.5;
  // //0.48
  // configs.kI = 0.00;
  // //0.01
  // configs.kD = 0.01;

  // leftShooterMotor.getConfigurator().apply(configs, 0.050);
  // rightShooterMotor.getConfigurator().apply(configs, 0.050);
  }
  
  public void setRollers(double speed){

    leftShooterMotor.set(speed);
    rightShooterMotor.set(-speed);
  }

  public void setPID(double desireSpeed)
  {
    setpoint = desireSpeed;
    vel.Slot = 0;
    leftShooterMotor.setControl(vel.withVelocity(desireSpeed));
    rightShooterMotor.setControl(vel.withVelocity(-desireSpeed));
    
  }

  public double getRotorVelocity()
  {
    return leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble();
  }
  public boolean atSetpoint()
  {
    return Math.abs(leftShooterMotor.getRotorVelocity().refresh().getValueAsDouble()) >= Math.abs(setpoint); 
  }

  public void stopShooter()
  {
    vel.Slot = 0;
    leftShooterMotor.set(0);
    rightShooterMotor.set (0);
    System.out.println("//------------------SHOOTER IS STOPPED ------------------///");
  }

 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return feedingRollers.getSupplyCurrent();
  }
 
 
 
  @Override
  public void periodic(){
    // SmartDashboard.putNumber("Shooter Velocity", getRotorVelocity());
  }
}