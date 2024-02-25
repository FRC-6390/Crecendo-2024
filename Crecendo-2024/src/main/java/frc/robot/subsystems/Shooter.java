package frc.robot.subsystems;
 
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.sensors.IRBBSensor;
 
public class Shooter extends SubsystemBase {
 

  public static TalonFX feedingRollers;

 
 
  public Shooter()
  {

  }
 
  static
  {
 
  feedingRollers = new TalonFX(Constants.INTAKE.FEEDNG_ROLLER_MOTOR, Constants.DRIVETRAIN.CANBUS);
  //intakeBeamBreak = new DigitalInput(0);
  }
 
  //Get value of the intake lift limit switch

 
 
  //Sets the intake rollers
  public static void setRollers(double speed)
  {
    feedingRollers.set(speed);
   // System.out.println("_____________________________-Beam is INTACT -___________________________________");
    }
 

 
  //Sets the lift to a certain speed
 
 
  //Gets lift position
 
 
  //Gets roller position
 
 
  public static StatusSignal<Double> getRollerCurrent()
  {
    return feedingRollers.getSupplyCurrent();
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