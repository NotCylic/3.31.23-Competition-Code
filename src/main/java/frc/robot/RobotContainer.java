// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
/*import edu.wpi.first.math.controller.PIDController;
* import edu.wpi.first.math.controller.ProfiledPIDController;
* import edu.wpi.first.math.geometry.Pose2d;
* import edu.wpi.first.math.geometry.Rotation2d;
* import edu.wpi.first.math.geometry.Translation2d;
* import edu.wpi.first.math.trajectory.Trajectory;
* import edu.wpi.first.math.trajectory.TrajectoryConfig;
* import edu.wpi.first.math.trajectory.TrajectoryGenerator;
*/
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmDown;
import frc.robot.commands.ArmMidRow;
import frc.robot.commands.ArmTopRow;
import frc.robot.commands.ArmTravel;
import frc.robot.commands.AutonomousC;
import frc.robot.commands.DropObject;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.IntakeIdle;
import frc.robot.commands.IntakeStop;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final DriveSubsystem autoDrive;
  private final Intake intake;
  private final Arm arm;
  private final IntakeIdle intakeIdle;
  private final IntakeCube intakeCube; 
  private final IntakeCone intakeCone;
  private final IntakeStop intakeStop;
  private final DropObject dropObject;
  private final ArmDown armDown;
  private final ArmMidRow armMidRow;
  private final ArmTopRow armTopRow;
  private final AutonomousC autonomousC;
  private final ArmTravel armTravel;
  //private static double translateValue;
  //private static double translateValue2;
  
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.startAutomaticCapture(0);
        m_robotDrive = new DriveSubsystem();
        autoDrive = new DriveSubsystem();
        intake = new Intake();
        arm = new Arm();
        armTravel = new ArmTravel(arm);
        armTravel.addRequirements(arm);
        intakeIdle = new IntakeIdle(intake);
        intakeIdle.addRequirements(intake);
        intakeCube = new IntakeCube(intake);
        intakeCube.addRequirements(intake);
        intakeCone = new IntakeCone(intake);
        intakeCone.addRequirements(intake);
        intakeStop = new IntakeStop(intake);
        intakeStop.addRequirements(intake);
        dropObject = new DropObject(intake);
        dropObject.addRequirements(intake);
        armDown = new ArmDown(arm);
        armDown.addRequirements(arm);
        armMidRow = new ArmMidRow(arm);
        armMidRow.addRequirements(arm);
        armTopRow = new ArmTopRow(arm);
        armTopRow.addRequirements(arm);
        autonomousC = new AutonomousC(autoDrive, intake);
        autonomousC.addRequirements(autoDrive, intake);

    // Configure the button bindings
    configureButtonBindings();

    


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    new JoystickButton(m_operatorController, 5)
        .whileTrue(new IntakeCube(intake))
        .onFalse(new IntakeIdle(intake));

    new JoystickButton(m_operatorController, 6)
        .whileTrue(new IntakeCone(intake))
        .onFalse(new IntakeIdle(intake));

    new JoystickButton(m_operatorController, 3)
        .whileTrue(new DropObject(intake))
        .onFalse(new IntakeStop(intake));

    new JoystickButton(m_operatorController, 1)
        .onTrue(new ArmDown(arm));

    new JoystickButton(m_operatorController, 2)
        .onTrue(new ArmMidRow(arm));

    new JoystickButton(m_operatorController, 4)
        .onTrue(new ArmTopRow(arm));
  
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(new ArmTravel(arm));
  }

  public void setArmDown() {
    arm.armDown();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() {
 /*  // Create config for trajectory
  translateValue2 = 2;
  translateValue = 1;
  TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(translateValue, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(translateValue2, 0, new Rotation2d(0)),
    config);

var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    exampleTrajectory,
    m_robotDrive::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    new PIDController(AutoConstants.kPXController, 0, 0),
    new PIDController(AutoConstants.kPYController, 0, 0),
    thetaController,
    m_robotDrive::setModuleStates,
    m_robotDrive);

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
  */
// Run path following command, then stop at the end.
return autonomousC;
  }
}
