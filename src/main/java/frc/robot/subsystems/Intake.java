package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.controller.DebouncedJoystick;
import frc.robot.utilities.sensors.DistanceSensor;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Intake extends SubsystemBase {
 
  public static TalonFX centerIntakeRoller;
  public static TalonFX fullWidthIntakeRoller;
  public static TalonFX feedingRollers;
  // public static IRBBSensor lowerIntakeBeamBreak;
  public static IRBBSensor upperIntakeBeamBreak;
  public static DistanceSensor distanceSensor;

 
 
  public Intake()
  {

  }
 
  static
  {
    
  centerIntakeRoller = new TalonFX(Constants.INTAKE.CENTER_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  fullWidthIntakeRoller = new TalonFX(Constants.INTAKE.FULL_WIDTH_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  // lowerIntakeBeamBreak = new IRBBSensor(Constants.INTAKE.BEAM_BREAK);
  upperIntakeBeamBreak = new IRBBSensor(4);
  feedingRollers = new TalonFX(Constants.INTAKE.FEEDNG_ROLLER_MOTOR, Constants.DRIVETRAIN.CANBUS);
 
  TalonFXConfiguration con2 =  new TalonFXConfiguration();
  CurrentLimitsConfigs curr = new CurrentLimitsConfigs();
  curr.SupplyCurrentLimitEnable = true;
  curr.SupplyCurrentLimit = 40;
  con2.withCurrentLimits(curr);

  feedingRollers.getConfigurator().apply(con2);
  fullWidthIntakeRoller.getConfigurator().apply(con2);
  centerIntakeRoller.getConfigurator().apply(con2);
  distanceSensor = new DistanceSensor(Port.kOnboard);
  distanceSensor.setEnabled(true);
  distanceSensor.setDistanceUnits(Unit.kMillimeters);
  distanceSensor.setAutomaticMode(true);
}
 
  // //Get value of the intake lift limit switch
  // public static boolean getLowerBeamBreak()
  // {
  //   //false for triggered, otherwise true
  // return lowerIntakeBeamBreak.isBroken();
  // }
 
   public static boolean getUpperBeamBreak(){
 
    return upperIntakeBeamBreak.isBroken();
   }

  public void feed(double speed)
  {
    feedingRollers.set(speed);
  }
  public void centerIntake(double speed)
  {
    centerIntakeRoller.set(speed);
  }
 
  public void fullWidth(double speed)
  {
    fullWidthIntakeRoller.set(0);
  }
  public void fullWidthMove(double speed)
  {
    fullWidthIntakeRoller.set(speed);
  }
  //Sets the intake rollers
  public void setRollers(double speed, int num)
  {
    if(num == 1)
    {
    centerIntakeRoller.set(0);
    fullWidthIntakeRoller.set(0);
    feedingRollers.set(0);
   //Beam is BROKEN
    }
    else if(num == 2)
    {
    centerIntakeRoller.set(speed - 0.2);
    fullWidthIntakeRoller.set(speed);
    feedingRollers.set(speed + 0.4);
   // System.out.println("_____________________________-Beam is INTACT -___________________________________");
    }
 
  }
 

  public void setRollersRPS(double speed)
  {
    VelocityDutyCycle vel = new VelocityDutyCycle(speed);
    centerIntakeRoller.setControl(vel);
  }

 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return centerIntakeRoller.getSupplyCurrent();
  }
  public double getRange()
  {
    return distanceSensor.getRange();
  }
 

  @Override
  public void periodic()
  {
   SmartDashboard.putBoolean("Game Piece", getUpperBeamBreak());
   SmartDashboard.putNumber("Distance", distanceSensor.getRange());
  //  SmartDashboard.putNumber("Full Width", fullWidthIntakeRoller.getSupplyCurrent().refresh().getValueAsDouble());
  //  SmartDashboard.putNumber("Center", centerIntakeRoller.getSupplyCurrent().refresh().getValueAsDouble());
  //  SmartDashboard.putNumber("Feeder", feedingRollers.getSupplyCurrent().refresh().getValueAsDouble());
  }
}