package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.utilities.drivetrain.SwerveDrivetrain;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModule;
// import frc.robot.utilities.telemetry.SwerveTelemetry;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightHelpers;
 
public class DriveTrain extends SwerveDrivetrain {
  private static SwerveModule[] swerveModules = new SwerveModule[4];
  public static SwerveDrivetrain drivetrain = new SwerveDrivetrain(swerveModules, DRIVETRAIN.PIGEON);

  static
  {
    swerveModules[0] = new
    SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG);
    swerveModules[1] = new
    SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG);
    swerveModules[2] = new
    SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG);
    swerveModules[3] = new
    SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG);
    drivetrain = new SwerveDrivetrain(swerveModules, DRIVETRAIN.PIGEON);
  }
  public DriveTrain()
  {
    super(swerveModules, DRIVETRAIN.PIGEON);
  }
 
@Override
public void periodic() 
{
drivetrain.update();
}

  @Override
  public void simulationPeriodic() {

  }
  
}