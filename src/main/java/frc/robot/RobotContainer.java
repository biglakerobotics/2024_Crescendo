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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.generated.Commands.TrapShootCommand;
import frc.robot.generated.Commands.IntakeWithIndexerCommand;
import frc.robot.generated.Commands.IntakeWithoutIndexerCommand;
import frc.robot.generated.Commands.InverseIntakeCommand;
import frc.robot.generated.Commands.ShootCommand;
import frc.robot.generated.Commands.AmpShootCommand;
import frc.robot.generated.Commands.SpeakerShootCommand;
import frc.robot.generated.Commands.StopShootCommand;
import frc.robot.generated.Manipulators.TrapShooter;
import frc.robot.generated.Manipulators.AmpShooter;
import frc.robot.generated.Manipulators.Intake;
import frc.robot.generated.Manipulators.Shooter;
import frc.robot.generated.Manipulators.SpeakerShooter;

public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

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

  private final XboxController mDriverController = new XboxController(0);
  private final XboxController mManipulatorController = new XboxController(1);
  private final XboxController mMasterController = new XboxController(2);

  private final Shooter mShooter = new Shooter();
  private final TrapShooter mTrapShooter = new TrapShooter();
  private final SpeakerShooter mSpeakerShooter = new SpeakerShooter();
  private final AmpShooter mAmpShooter = new AmpShooter();
  private final Intake mIntake = new Intake();
  private final LimeLight mLimelight = new LimeLight();

  private final ShootCommand mShootCommand = new ShootCommand(mShooter);
  private final AmpShootCommand mAmpShootCommand = new AmpShootCommand(mAmpShooter);
  private final TrapShootCommand mTrapShootCommand = new TrapShootCommand(mTrapShooter);
  private final SpeakerShootCommand mSpeakerShootCommand = new SpeakerShootCommand(mSpeakerShooter);
  private final IntakeWithIndexerCommand mIntakeWithIndexerCommand = new IntakeWithIndexerCommand(mIntake);
  private final IntakeWithoutIndexerCommand mIntakeWithoutIndexerCommand = new IntakeWithoutIndexerCommand(mIntake);
  private final InverseIntakeCommand mInverseIntakeCommand = new InverseIntakeCommand(mIntake);
  private final LimeLightTestCommand mLimeLightTestCommand = new LimeLightTestCommand(mLimelight);
  private final StopShootCommand mStopShootingCommand = new StopShootCommand(mShooter);
  

  private JoystickButton intakeWIndexerButton = new JoystickButton(mMasterController, 1); //A
  private JoystickButton intakeWOIndexerButton = new JoystickButton(mMasterController, 2); //B
  private JoystickButton speakerShootButton = new JoystickButton(mMasterController, 3); //X
  private JoystickButton ampShootButton = new JoystickButton(mMasterController, 4); //Y
  private JoystickButton trapShootButton = new JoystickButton(mMasterController, 7); //BackButton
  private JoystickButton inverseIntakeButton = new JoystickButton(mMasterController, 8); //StartButton

  private JoystickButton driverSlowSpeedButton = new JoystickButton(mDriverController, 6); //RB

  private JoystickButton manipulatorIntakeWIndexerButton = new JoystickButton(mManipulatorController, 5); //LB
  private JoystickButton manipulatorIntakeWOIndexerButton = new JoystickButton(mManipulatorController, 6); //RB
  private JoystickButton manipulatorSpeakerShootButton = new JoystickButton(mManipulatorController, 1); //A
  private JoystickButton manipulatorAmpShootButton = new JoystickButton(mManipulatorController, 2); //B
  private JoystickButton manipulatorTrapShootButton = new JoystickButton(mManipulatorController, 4); //Y
  private JoystickButton manipulatorInverseIntakeButton = new JoystickButton(mManipulatorController, 7); //BackButton
  // private JoystickButton limeLightButton = new JoystickButton(mMasterController, 10);


  SendableChooser<Command> m_chooser;
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI*2; // Half a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  public RobotContainer() {

    NamedCommands.registerCommand("PrintCookie",Commands.print("You have a cookie"));
    NamedCommands.registerCommand("ShootSpeaker", mSpeakerShootCommand);
    NamedCommands.registerCommand("StopShooting", mStopShootingCommand);
    NamedCommands.registerCommand("IntakeFromFloor", mIntakeWithIndexerCommand);
    NamedCommands.registerCommand("IntakeForShooting", mIntakeWithoutIndexerCommand);
    // NamedCommands.registerCommand("StopShooting",);
    


    configureBindings();
    autoChooser.addOption("ScoreTestAuto",drivetrain.getAutoPath("ScoreTestAuto"));
    autoChooser.setDefaultOption("AutoForAsher",drivetrain.getAutoPath("AutoForAsher"));
    SmartDashboard.putData("Auto Chooser", autoChooser);


    // m_chooser.setDefaultOption("Test Auto", new SwerveAutoBuilder());


  }
  public static RobotContainer getInstance() {
    return m_robotContainer;
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
 
  private SlewRateLimiter m_strafeX;
  private SlewRateLimiter m_strafeY;


  private void configureBindings() {
    // intakeWIndexerButton.whileTrue(mIntakeWithIndexerCommand);
    // intakeWOIndexerButton.whileTrue(mIntakeWithoutIndexerCommand);
    // speakerShootButton.whileTrue(mSpeakerShootCommand);
    // ampShootButton.whileTrue(mAmpShootCommand);
    // trapShootButton.whileTrue(mTrapShootCommand);
    // inverseIntakeButton.whileTrue(mInverseIntakeCommand);

    manipulatorIntakeWIndexerButton.whileTrue(mIntakeWithIndexerCommand);
    manipulatorIntakeWOIndexerButton.whileTrue(mIntakeWithoutIndexerCommand);
    manipulatorSpeakerShootButton.whileTrue(mSpeakerShootCommand);
    manipulatorAmpShootButton.whileTrue(mAmpShootCommand);
    manipulatorTrapShootButton.whileTrue(mTrapShootCommand);
    manipulatorInverseIntakeButton.whileTrue(mInverseIntakeCommand);

    driverSlowSpeedButton.whileTrue(mAmpShootCommand);

    // shootButton.whileTrue(mShootCommand);
    // intakeButton.whileTrue(mIntakeCommand);
    // bottomShootButton.whileTrue(mBottomShootCommand);
    // topShootButton.whileTrue(mTopShootCommand);
    // limeLightButton.whileTrue(mLimeLightTestCommand);
    
    m_strafeX = new SlewRateLimiter(5);
    m_strafeY = new SlewRateLimiter(5);

    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
    //         .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(m_strafeX.calculate(MathUtil.applyDeadband(-joystick.getLeftY(),
                0.1)) * MaxSpeed) // Drive forward with
            .withVelocityY(m_strafeY.calculate(MathUtil.applyDeadband(-joystick.getLeftX(),
            0.1)) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with negative X (left)
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
