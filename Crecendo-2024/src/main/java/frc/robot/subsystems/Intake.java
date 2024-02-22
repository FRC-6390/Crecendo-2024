
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.IRBBSensor;

public class Intake extends SubsystemBase {

  public static TalonFX centerIntakeRoller;
  public static TalonFX fullWidthIntakeRoller;
  public static IRBBSensor intakeBeamBreak;


  public Intake() {

  }

  static{

  centerIntakeRoller = new TalonFX(Constants.INTAKE.CENTER_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  fullWidthIntakeRoller = new TalonFX(Constants.INTAKE.FULL_WIDTH_INTAKE_MOTOR, Constants.DRIVETRAIN.CANBUS);
  intakeBeamBreak = new IRBBSensor(Constants.INTAKE.BEAM_BREAK); 
}

  //Get value of the intake lift limit switch
public static boolean getBeamBreak(){
    //false for triggered, otherwise true
  return intakeBeamBreak.isBroken();
}
  @Override
  public void periodic() {
    
  }

  //Sets the intake rollers
  public static void setRollers(double speed){
    centerIntakeRoller.set(speed);
    fullWidthIntakeRoller.set(speed);

  }

  //Sets the lift to a certain speed
  

  //Gets lift position


  //Gets roller position


  public static StatusSignal<Double> getRollerCurrent(){
    return centerIntakeRoller.getSupplyCurrent();
  }

  //Resets lift encoder


  //Resets roller encoder


  //Get value of the intake lift limit switch

}
