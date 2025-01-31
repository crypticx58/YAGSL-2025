package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

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
  public final SwerveController swerveController;

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
    //SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "mk4i")).createSwerveDrive(3.5);
      swerveController = swerveDrive.getSwerveController();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(true);
    //swerveDrive.pushOffsetsToEncoders();
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

      final boolean enableFeedforward = false;
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
              new PIDConstants(4.25, 0.0, 0),
              // Translation PID constants
              new PIDConstants(4.65, 0.0, 0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
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

  public Command pathfindToSetRange(Pose2d targetPose, double rangeMeters, boolean frontFacingTarget) {
    Pose2d robotPose = swerveDrive.getPose();
    Translation2d targetToRobot = robotPose.getTranslation().minus(targetPose.getTranslation());
    // unit vec
    Translation2d setPointTranslation = targetPose.getTranslation()
        .plus(targetToRobot.div(targetToRobot.getNorm()).times(rangeMeters));
    Rotation2d targetAngle = robotPose.getRotation()
        .plus(PhotonUtils.getYawToPose(robotPose, targetPose))
        .plus(Rotation2d.fromDegrees(frontFacingTarget ? 0 : 180));

    return AutoBuilder.pathfindToPose(new Pose2d(new Translation2d(1, 1), new Rotation2d()),
        new PathConstraints(swerveDrive.getMaximumChassisVelocity(), 4.5,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720)));
  }

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

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }
}