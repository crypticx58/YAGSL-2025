package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utils.ArmPreset;
import frc.robot.Utils.ReefTarget;
import frc.robot.commands.GoToArmPreset;
import frc.robot.commands.GoToReefBasedOnPoseEstimation;
import frc.robot.commands.ZeroArm;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.config.PIDConstants;

public class SwerveSubsystem extends SubsystemBase {
  /**
   * The Singleton instance of this SwerveSubsystem. Code should use
   * the {@link #getInstance()} method to get the single instance (rather
   * than trying to construct an instance of this class.)
   */
  private static SwerveSubsystem INSTANCE;
  public final SwerveDrive swerveDrive;
  public final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  public final SwerveController swerveController;
  final PIDController translationalPidController = new PIDController(3.7, 0, 0);
  final PIDController rotationalPidController = new PIDController(2.75, 0.00, 0);
  public Pose2d swervePoseSetpoint;

  /**
   * Returns the Singleton instance of this SwerveSubsystem. This static method
   * should be used, rather than the constructor, to get the single instance
   * of this class. For example: {@code SwerveSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static SwerveSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SwerveSubsystem();
    }
    return INSTANCE;
  }

  /**
   * Creates a new instance of this SwerveSubsystem. This constructor
   * is private since this class is a Singleton. Code should use
   * the {@link #getInstance()} method to get the singleton instance.
   */
  private SwerveSubsystem() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    // in the constructor or in the robot coordination class, such as
    // RobotContainer.
    // Also, you can call addChild(name, sendableChild) to associate sendables with
    // the subsystem
    // such as SpeedControllers, Encoders, DigitalInputs, etc.
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "mk4i")).createSwerveDrive(4.5);
      swerveController = swerveDrive.getSwerveController();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    //swerveDrive.setAngularVelocityCompensation(true, false, -0.15);
    //swerveDrive.pushOffsetsToEncoders();

    rotationalPidController.enableContinuousInput(-180, 180);
    translationalPidController.setTolerance(Units.inchesToMeters(2));
    rotationalPidController.setTolerance(2);
    zeroGyro();
    setupPathPlanner();

  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  private void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(4.1, 0.0, 0),
              // Translation PID constants
              new PIDConstants(3.25, 0.0, 0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            // var alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            //   return alliance.get() == DriverStation.Alliance.Red;
            // }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command pathfindToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, new PathConstraints(swerveDrive.getMaximumChassisVelocity(), 4.5,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)));
  }
  public Command pathfindToReefTarget(ReefTarget reefTarget, boolean offset) {
    Pose3d targetPose = FieldConstants.aprilTagFieldLayout.getTagPose(reefTarget.ApriltagId).get();
    Pose2d pathfindSwervePoseSetpoint = targetPose.plus(
      new Transform3d(
        new Translation3d(offset?ArmConstants.OffsetReefWallDistance:ArmConstants.AgainstReefWallDistance, 0,0),
        new Rotation3d(0,0,Math.PI)
      )
    ).toPose2d();
  public Command pathfindToFieldTarget(FieldTarget fieldTarget, boolean isOffset) {
    Pose3d targetPose = fieldTarget.getTargetPose();//FieldConstants.aprilTagFieldLayout.getTagPose(reefTarget.ApriltagId).get();
    Transform3d offset = new Transform3d();
    currentFieldTarget = fieldTarget;
    offset = FieldConstants.getFieldTargetOffset(fieldTarget, isOffset);
    Pose2d pathfindSwervePoseSetpoint = targetPose.plus(offset).toPose2d();
    return this.pathfindToPose(pathfindSwervePoseSetpoint);
  }
  public SequentialCommandGroup getBaseAutonSequentialCommandGroup(){
    return new SequentialCommandGroup(
      // new GoToReefBasedOnPoseEstimation(false),
      // new GoToArmPreset(ArmPreset.LowAlgae),
      Commands.runOnce(()->intakeSubsystem.toggleIntake(0.19)),
      new GoToReefBasedOnPoseEstimation(true),
      Commands.waitSeconds(1.5),
      new GoToReefBasedOnPoseEstimation(false),
      new ZeroArm(),
      this.pathfindToProcessor(true),
      new GoToArmPreset(ArmPreset.Processor),
      this.pathfindToProcessor(false),
      Commands.runOnce(()->intakeSubsystem.toggleOutake(-0.25)),
      Commands.waitSeconds(0.5),
      Commands.runOnce(()->intakeSubsystem.turnOffIntake()),
      new ZeroArm()
      );
  }
  public Command pathfindToProcessor(boolean offset) {
    Pose3d targetPose = FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceProcessorId()).get();
    Pose2d pathfindSwervePoseSetpoint = targetPose.plus(
      new Transform3d(
        new Translation3d(offset?ArmConstants.OffsetProcessorScoringDistance:ArmConstants.ProcessorScoringDistance, 0,0),
        new Rotation3d(0,0,Math.PI)
      )
    ).toPose2d();
    return this.pathfindToPose(pathfindSwervePoseSetpoint);
  }

  // public Command pathfindToSetRange(Pose2d targetPose, double rangeMeters, boolean frontFacingTarget) {
  //   Pose2d robotPose = swerveDrive.getPose();
  //   Translation2d targetToRobot = robotPose.getTranslation().minus(targetPose.getTranslation());
  //   // unit vec
  //   Translation2d setPointTranslation = targetPose.getTranslation()
  //       .plus(targetToRobot.div(targetToRobot.getNorm()).times(rangeMeters));
  //   Rotation2d targetAngle = robotPose.getRotation()
  //       .plus(PhotonUtils.getYawToPose(robotPose, targetPose))
  //       .plus(Rotation2d.fromDegrees(frontFacingTarget ? 0 : 180));

  //   return AutoBuilder.pathfindToPose(new Pose2d(setPointTranslation, targetAngle),
  //       new PathConstraints(swerveDrive.getMaximumChassisVelocity(), 4.5,
  //           swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)));
  // }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
  public Command drive(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }
  public ChassisSpeeds chassisSpeedsForSwerveSetpointWithPID(){
    return chassisSpeedsForSwerveSetpointWithPID(this.swervePoseSetpoint);
  }
  public ChassisSpeeds chassisSpeedsForSwerveSetpointWithPID(Pose2d swervePoseSetpoint){
    return chassisSpeedsForSwerveSetpointWithPID(swervePoseSetpoint, translationalPidController, rotationalPidController);
  }
  public ChassisSpeeds chassisSpeedsForSwerveSetpointWithPID(Pose2d swervePoseSetpoint, PIDController translationalPIDController, PIDController rotationalPIDController){
    Pose2d robotPose = swerveDrive.getPose();
    Vector<N2> robotVec = robotPose.getTranslation().toVector();
    Vector<N2> targetPoseRelativeToRobotPose = swervePoseSetpoint.getTranslation().toVector().minus(robotVec);
    double distanceFromTarget = targetPoseRelativeToRobotPose.norm();

    Vector<N2> traversalVector = new Vector<N2>(Nat.N2());
    traversalVector.set(0,0,targetPoseRelativeToRobotPose.get(0,0));
    traversalVector.set(1,0,targetPoseRelativeToRobotPose.get(1,0));
    traversalVector = traversalVector.unit().times(-translationalPIDController.calculate(distanceFromTarget));

    Vector<N2> robotForwardVec = robotPose.transformBy(new Transform2d(1, 0, new Rotation2d())).getTranslation().toVector().minus(robotVec);
    Vector<N2> robotLateralVec = robotPose.transformBy(new Transform2d(0, 1, new Rotation2d())).getTranslation().toVector().minus(robotVec);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        robotForwardVec.norm()*traversalVector.dot(robotForwardVec),
        robotLateralVec.norm()*traversalVector.dot(robotLateralVec),
        Units.degreesToRadians(rotationalPIDController.calculate(robotPose.getRotation().getDegrees()))         
    );

    return chassisSpeeds;
  }
  public boolean swerveSetpointReached(){
    return translationalPidController.atSetpoint() && rotationalPidController.atSetpoint();
  }
  public void setSwervePoseSetpoint(Pose2d swervePoseSetpoint){
    rotationalPidController.reset();
    translationalPidController.reset();

    translationalPidController.setSetpoint(0);
    rotationalPidController.setSetpoint(swervePoseSetpoint.getRotation().getDegrees());
    this.swervePoseSetpoint = swervePoseSetpoint;
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  // Changes the "forward" for field oriented drive with affecting odometry
  // public void zeroFieldOrientedHeading(){
  //   swerveDrive.setFieldOrientedHeadingOffset(swerveDrive.getOdometryHeading());
  // }
  public void zeroFieldOrientedHeading(SwerveInputStream swerveInputStream){
    swerveInputStream.translationHeadingOffset(true).translationHeadingOffset(swerveDrive.getOdometryHeading());
  }
}