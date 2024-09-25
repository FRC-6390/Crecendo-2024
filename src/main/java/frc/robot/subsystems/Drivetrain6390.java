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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.sensors.Button;
import frc.robot.utilities.swerve.SwerveModule;
// import frc.robot.utilities.telemetry.SwerveTelemetry;
import frc.robot.utilities.vission.LimeLight;
import frc.robot.utilities.vission.LimelightHelpers;
import frc.robot.utilities.vission.LimelightHelpers.PoseEstimate;
 
public class Drivetrain6390 extends SubsystemBase{

  private static SwerveModule[] swerveModules;
  private static Boolean isRed;
  private static PowerDistribution pdh;
  private static Pigeon2 gyro;
  private static ChassisSpeeds chassisSpeeds, feedbackSpeeds;
  // private static SwerveTelemetry tele;
  public static SwerveDriveKinematics kinematics;
  private static SwerveDriveOdometry odometry;
  private static Pose2d pose;
  private static ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
  private static Field2d gameField;
  private static Field2d gameFieldVision;
  private static Field2d gameFieldVision2;
  private static double desiredHeading;
  public static ReplanningConfig c;
  public static Button button;

  //0.1
  private static PIDConfig driftCorrectionPID = new PIDConfig(5, 0,0).setContinuous(-Math.PI, Math.PI);
  private static Pose2d visionPose;
  private static PID pid;
  public LimeLight limeLight;

  // public static Orchestra orchestra = new Orchestra();
  
  public SwerveDrivePoseEstimator estimator = 
  new SwerveDrivePoseEstimator(
    kinematics, 
    getRotation2d(), 
    getModulePostions(), 
    new Pose2d(), 
    VecBuilder.fill(0.1,0.1,Units.degreesToRadians(3)), 
    VecBuilder.fill(0.5,0.5,99999));

  public Drivetrain6390(LimeLight limelight)
  {
    this.limeLight = limelight;
    AutoBuilder.configureHolonomic
    (
      this::getVisionPose,
      this::resetOdometryVision,
      this::getSpeeds,
      this::drive,
      //0.85 translation 3.125 rotation
      new HolonomicPathFollowerConfig(new PIDConstants(0.9), new PIDConstants(3.125), Constants.SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND, Constants.DRIVETRAIN.SWERVE_MODULE_LOCATIONS[0].getNorm(), new ReplanningConfig()),
      this::isRed,
      this
    );
    
  
  }

  static {
    button = new Button(new DigitalInput(9));  
    gameField = new Field2d();
    gameFieldVision = new Field2d();
    gameFieldVision2 = new Field2d();
    
   
    swerveModules = new SwerveModule[4];
    swerveModules[0] = new
    SwerveModule(DRIVETRAIN.FRONT_LEFT_MODULE_CONFIG,tab);
    swerveModules[1] = new
    SwerveModule(DRIVETRAIN.FRONT_RIGHT_MODULE_CONFIG, tab);
    swerveModules[2] = new
    SwerveModule(DRIVETRAIN.BACK_LEFT_MODULE_CONFIG, tab);
    swerveModules[3] = new
    SwerveModule(DRIVETRAIN.BACK_RIGHT_MODULE_CONFIG, tab);
    gyro = new Pigeon2(DRIVETRAIN.PIGEON, DRIVETRAIN.CANBUS);

    pdh = new PowerDistribution(DRIVETRAIN.REV_PDH, ModuleType.kRev);
    chassisSpeeds = new ChassisSpeeds();
    feedbackSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] SwervePositions =
    {swerveModules[0].getPostion(), swerveModules[1].getPostion(),
    swerveModules[2].getPostion(), swerveModules[3].getPostion()};

    kinematics = new SwerveDriveKinematics(DRIVETRAIN.SWERVE_MODULE_LOCATIONS);
    odometry = new SwerveDriveOdometry(kinematics,
    Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()), SwervePositions);
    pose = new Pose2d();
    visionPose = new Pose2d();

    pid = new PID(driftCorrectionPID).setMeasurement(() ->
    pose.getRotation().getDegrees());
}

  public void init(){
    pdh.clearStickyFaults();
    zeroHeading();
    resetOdometry(new Pose2d(0,0,getRotation2d()));
    isRed = DriverStation.getAlliance().get().equals(Alliance.Red);

    //shuffleboard();
  }

  public static void updateSide(){
    isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  public void zeroHeading(){
    gyro.setYaw(0);
    resetOdometry(pose);
  }
  public void setOdometryVision(){
    resetOdometryVision(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
  }
  public double getRate(){
    return gyro.getRate();
  }

  public  void resetHeading(){
    gyro.setYaw(0);
  }

  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll().refresh().getValueAsDouble(), 360);
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch().refresh().getValueAsDouble(),360);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void driftCorrection(ChassisSpeeds speeds){
    if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) desiredHeading =
pose.getRotation().getDegrees();
    else speeds.omegaRadiansPerSecond += pid.calculate(desiredHeading);
  }

  public Boolean isRed()
  {
    return isRed; 
  }

  public void drive(ChassisSpeeds speeds){
    chassisSpeeds = speeds;
    //System.out.println(speeds);
  }

  public Pose2d getPose(){
    return pose;
  }

  public Pose2d getVisionPose(){
   return visionPose;
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(getRotation2d(), getModulePostions(), pose);
    estimator.resetPosition(getRotation2d(), getModulePostions(), visionPose);
  }
   public void resetOdometryVision(Pose2d pose)
  {
    estimator.resetPosition(getRotation2d(), getModulePostions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states,
SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(states[i]);
      //SmartDashboard.putNumber("Module " + i+"",swerveModules[i].getEncoderRadians());
    }
    
  }

  private SwerveModulePosition[] getModulePostions(){
    SwerveModulePosition[] positions = new
SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPostion();
    }
    return positions;
  }

  public void feedbackDrive(ChassisSpeeds speeds){
    feedbackSpeeds = speeds;
  }

  public void stopWheels(){
    for(int i = 0; i < swerveModules.length; i++){
      swerveModules[i].stop();
     // System.out.println("/////////////////////////////////////|||||||||||||||||||||||||");
    }
  }

  public void lockWheels(){

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].lock();
    }
  }

  public void unlockWheels(){
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].unlock();
    }
  }

 private void updateOdometry(){
      odometry.update(getRotation2d(), getModulePostions());
      pose = odometry.getPoseMeters();

      estimator.update(getRotation2d(), getModulePostions());
      LimelightHelpers.SetRobotOrientation("limelight", estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(mt2 != null){
      if(Math.abs(gyro.getRate()) < 720 && mt2.tagCount > 0) 
      {
        // System.out.println("UPDATE");
        estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        estimator.addVisionMeasurement(mt2.pose,mt2.timestampSeconds);
        // estimator.setVisionMeasurementStdDevs(VecBuilder.fill
        // (
        // 0.1 * limeLight.getDistanceFromTarget(0,0,0),
        // 0.1 * limeLight.getDistanceFromTarget(0,0,0),
        // 999999999
        // ));
      }
      visionPose = estimator.getEstimatedPosition();
      
      gameField.setRobotPose(pose);
      gameFieldVision2.setRobotPose(mt2.pose);
      gameFieldVision.setRobotPose(visionPose);
    }
  }
 

    

  public ChassisSpeeds getSpeeds()
  {
    return chassisSpeeds;
  }

  public double maxAccel = 0;
  
  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      if (button.isPressed()){
        unlockWheels();
      }
    else {
      lockWheels();
    }
    }

   
    double xSpeed = chassisSpeeds.vxMetersPerSecond +
feedbackSpeeds.vxMetersPerSecond;
    double ySpeed = chassisSpeeds.vyMetersPerSecond +
feedbackSpeeds.vyMetersPerSecond;
    double thetaSpeed = chassisSpeeds.omegaRadiansPerSecond +
feedbackSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds speed = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

    setModuleStates(states);

    updateOdometry();

    if(DriverStation.isTeleop())
    {
    driftCorrection(speed);
    }
    SmartDashboard.putData("Vision Pose", gameFieldVision);
    SmartDashboard.putData("Vision Pose Raw", gameFieldVision2);

    if(gyro.getAccelerationX().getValueAsDouble() > maxAccel)
    {
      maxAccel = gyro.getAccelerationX().getValueAsDouble();
    }
  }

  @Override
  public void simulationPeriodic() {

  }
  
}