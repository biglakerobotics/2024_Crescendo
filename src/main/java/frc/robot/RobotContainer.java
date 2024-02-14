// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
// import com.pathplanner.lib.auto.NamedCommands;
// import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimeLightCommands.LimeLightTestCommand;
import frc.robot.generated.LimeLight;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Commands.BottomShootCommand;
import frc.robot.generated.Commands.IntakeCommand;
import frc.robot.generated.Commands.InverseIntakeCommand;
import frc.robot.generated.Commands.ShootCommand;
import frc.robot.generated.Commands.SlowShootCommand;
import frc.robot.generated.Commands.TopShootCommand;
import frc.robot.generated.Manipulators.BottomShooter;
import frc.robot.generated.Manipulators.Intake;
import frc.robot.generated.Manipulators.Shooter;
import frc.robot.generated.Manipulators.TopShooter;

public class RobotContainer {

  /*BUTTONS!!!
   * XBOX:
   * A = Intake
   * B = Shoot High
   * X = Limelight Test
   * Y = Shoot normal?
   * Rbumper = Shoot Bottom? Also brake mode for swerve
   * Lbumper = Reset Gyro
   * 
   * 
   * 
   * 
   * 
   */

  private final XboxController mXboxController = new XboxController(0);

  private final Shooter mShooter = new Shooter();
  private final BottomShooter mBottomShooter = new BottomShooter();
  private final TopShooter mTopShooter = new TopShooter();
  private final Intake mIntake = new Intake();
  private final LimeLight mLimelight = new LimeLight();

  private final ShootCommand mShootCommand = new ShootCommand(mShooter);
  private final BottomShootCommand mBottomShootCommand = new BottomShootCommand(mBottomShooter);
  private final TopShootCommand mTopShootCommand = new TopShootCommand(mTopShooter);
  private final IntakeCommand mIntakeCommand = new IntakeCommand(mIntake);
  private final InverseIntakeCommand mInverseIntakeCommand = new InverseIntakeCommand(mIntake);
  private final LimeLightTestCommand mLimeLightTestCommand = new LimeLightTestCommand(mLimelight);
  

  private JoystickButton shootButton = new JoystickButton(mXboxController, 4);
  private JoystickButton intakeButton = new JoystickButton(mXboxController, 1);
  private JoystickButton bottomShootButton = new JoystickButton(mXboxController, 6);
  private JoystickButton topShootButton = new JoystickButton(mXboxController, 2);
  private JoystickButton limeLightButton = new JoystickButton(mXboxController, 10);
  private JoystickButton inverseIntakeButton = new JoystickButton(mXboxController, 3);


  SendableChooser<Command> m_chooser;
  final double MaxSpeed = 2; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  public RobotContainer() {

    NamedCommands.registerCommand("PrintCookie",Commands.print("You have a cookie"));
    NamedCommands.registerCommand("ShootSpeaker", mTopShootCommand);
    // NamedCommands.registerCommand("StopShooting",);
    


    configureBindings();
    autoChooser.addOption("ScoreTestAuto",drivetrain.getAutoPath("ScoreTestAuto"));
    autoChooser.setDefaultOption("AutoForAsher",drivetrain.getAutoPath("AutoForAsher"));
    SmartDashboard.putData("Auto Chooser", autoChooser);


    // m_chooser.setDefaultOption("Test Auto", new SwerveAutoBuilder());


  }

  public Command getAutonomousCommand() {
    

    return autoChooser.getSelected();


  }

  /* Setting up bindings for necessary control of the swerve drive platform */
  CommandXboxController joystick = new CommandXboxController(0); // My joystick
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
  .withDriveRequestType(DriveRequestType.Velocity)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // I want field-centric
                                                                                            // driving in open loop
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  private Command runAuto = drivetrain.getAutoPath("ScoreTestAuto");
 
  

  private void configureBindings() {
    shootButton.whileTrue(mShootCommand);
    intakeButton.whileTrue(mIntakeCommand);
    inverseIntakeButton.whileTrue(mInverseIntakeCommand);
    bottomShootButton.whileTrue(mBottomShootCommand);
    topShootButton.whileTrue(mTopShootCommand);
    limeLightButton.whileTrue(mLimeLightTestCommand);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.rightTrigger().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putData("driveQforward", drivetrain.driveQ(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("driveQreverse", drivetrain.driveQ(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("driveDforward", drivetrain.driveD(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("driveDreverse", drivetrain.driveD(SysIdRoutine.Direction.kReverse));

    
    
  }

}
