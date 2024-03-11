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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.generated.Commands.LeftClimbCommandIn;
import frc.robot.generated.Commands.ShootCommand;
import frc.robot.generated.Commands.AmpShootCommand;
import frc.robot.generated.Commands.AutoIntakeWithIndexerCommand;
// import frc.robot.generated.Commands.AutoIntakeWithIndexerCommand;
import frc.robot.generated.Commands.AutoIntakeWithoutIndexerCommand;
import frc.robot.generated.Commands.AutoSpeakerShootCommand;
import frc.robot.generated.Commands.AutoSpeakerShootOnlyCommand;
import frc.robot.generated.Commands.AutoStopIntakeCommand;
import frc.robot.generated.Commands.AutoStopShootCommand;
import frc.robot.generated.Commands.RightClimbCommandIn;
import frc.robot.generated.Commands.RightClimbCommandOut;
import frc.robot.generated.Commands.LeftClimbCommandOut;
import frc.robot.generated.Commands.SpeakerShootCommand;
import frc.robot.generated.Commands.StopIntakingCommand;
import frc.robot.generated.Commands.StopShootCommand;
import frc.robot.generated.Commands.SuperClimbCommand;
import frc.robot.generated.Manipulators.TrapShooter;
import frc.robot.generated.Manipulators.AmpShooter;
import frc.robot.generated.Manipulators.Climber;
import frc.robot.generated.Manipulators.Intake;
import frc.robot.generated.Manipulators.Shooter;
import frc.robot.generated.Manipulators.SpeakerShooter;

public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  private final XboxController mDriverController = new XboxController(0);
  private final XboxController mManipulatorController = new XboxController(1);
  private final XboxController mMasterController = new XboxController(2);

  private final Shooter mShooter = new Shooter();
  private final TrapShooter mTrapShooter = new TrapShooter();
  private final SpeakerShooter mSpeakerShooter = new SpeakerShooter();
  private final AmpShooter mAmpShooter = new AmpShooter();
  private final Intake mIntake = new Intake();
  private final LimeLight mLimelight = new LimeLight();
  private final Climber mClimber = new Climber();

  private final ShootCommand mShootCommand = new ShootCommand(mShooter);
  private final StopIntakingCommand mStopIntakingCommand = new StopIntakingCommand(mIntake);
  private final AmpShootCommand mAmpShootCommand = new AmpShootCommand(mAmpShooter);
  private final TrapShootCommand mTrapShootCommand = new TrapShootCommand(mTrapShooter);
  private final SpeakerShootCommand mSpeakerShootCommand = new SpeakerShootCommand(mSpeakerShooter);
  private final IntakeWithIndexerCommand mIntakeWithIndexerCommand = new IntakeWithIndexerCommand(mIntake);
  private final IntakeWithoutIndexerCommand mIntakeWithoutIndexerCommand = new IntakeWithoutIndexerCommand(mIntake);
  private final InverseIntakeCommand mInverseIntakeCommand = new InverseIntakeCommand(mIntake);
  private final LimeLightTestCommand mLimeLightTestCommand = new LimeLightTestCommand(mLimelight);
  private final StopShootCommand mStopShootingCommand = new StopShootCommand(mShooter);
  private final RightClimbCommandIn mRightClimbCommandIn = new RightClimbCommandIn(mClimber);
  private final RightClimbCommandOut mRightClimbCommandOut = new RightClimbCommandOut(mClimber);
  private final LeftClimbCommandIn mLeftClimbCommandIn = new LeftClimbCommandIn(mClimber);
  private final LeftClimbCommandOut mLeftClimbCommandOut = new LeftClimbCommandOut(mClimber);
  private final SuperClimbCommand mSuperClimbCommand = new SuperClimbCommand(mClimber);
  

  private final AutoStopIntakeCommand mAutoStopIntakeCommand = new AutoStopIntakeCommand(mIntake);
  private final AutoIntakeWithIndexerCommand mAutoIntakeWithIndexerCommand = new AutoIntakeWithIndexerCommand(mIntake);
  private final AutoIntakeWithoutIndexerCommand mAutoIntakeWithoutIndexerCommand = new AutoIntakeWithoutIndexerCommand(mIntake);
  private final AutoSpeakerShootCommand mAutoSpeakerShootCommand = new AutoSpeakerShootCommand(mSpeakerShooter);
  private final AutoSpeakerShootOnlyCommand mAutoSpeakerShootWithIntakeCommand = new AutoSpeakerShootOnlyCommand(mSpeakerShooter, mIntake);
  private final AutoStopShootCommand mAutoStopShootCommand = new AutoStopShootCommand(mShooter);
  
// MasterControl need to swap to port zero and ungrey button bindings to commands at the bottom
  private JoystickButton intakeWIndexerButton = new JoystickButton(mMasterController, 1); //A
  private JoystickButton intakeWOIndexerButton = new JoystickButton(mMasterController, 2); //B
  private JoystickButton speakerShootButton = new JoystickButton(mMasterController, 3); //X
  private JoystickButton ampShootButton = new JoystickButton(mMasterController, 4); //Y
  private JoystickButton trapShootButton = new JoystickButton(mMasterController, 7); //BackButton
  private JoystickButton inverseIntakeButton = new JoystickButton(mMasterController, 8); //StartButton
// 

// Manipulator Buttons
  private JoystickButton manipulatorIntakeWIndexerButton = new JoystickButton(mManipulatorController, 5); //LB
  private JoystickButton manipulatorIntakeWOIndexerButton = new JoystickButton(mManipulatorController, 6); //RB
  private JoystickButton manipulatorSpeakerShootButton = new JoystickButton(mManipulatorController, 1); //A
  private JoystickButton manipulatorAmpShootButton = new JoystickButton(mManipulatorController, 2); //B
  private JoystickButton manipulatorTrapShootButton = new JoystickButton(mManipulatorController, 4); //Y
  private JoystickButton manipulatorInverseIntakeButton = new JoystickButton(mManipulatorController, 3); //X
//


  // private JoystickButton limeLightButton = new JoystickButton(mMasterController, 10);


  SendableChooser<Command> m_chooser;
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI*2; // Half a rotation per second max angular velocity
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  
  public RobotContainer() {

    //PUT AUTO COMMANDS HERE
    NamedCommands.registerCommand("ShootSpeaker", mAutoSpeakerShootCommand.withTimeout(0.5));
    // NamedCommands.registerCommand("ShootSpeakerWithIntake", mAutoSpeakerShootWithIntakeCommand.withTimeout(2));
    NamedCommands.registerCommand("StopShooting", mAutoStopShootCommand.withTimeout(0.1));
    NamedCommands.registerCommand("IntakeFromFloor", mAutoIntakeWithIndexerCommand.withTimeout(3));
    NamedCommands.registerCommand("IntakeForShooting", mAutoIntakeWithoutIndexerCommand.withTimeout(.5));
    NamedCommands.registerCommand("StopIntake", mAutoStopIntakeCommand.withTimeout(0.1));



    //PUT AUTOS HERE

    autoChooser.setDefaultOption("BoringAuto", drivetrain.getAutoPath("BoringAuto"));


    autoChooser.addOption("4PieceLeft", drivetrain.getAutoPath("4PieceLeft"));
    autoChooser.addOption("2pieceLeft", drivetrain.getAutoPath("2pieceLeft"));
    autoChooser.addOption("3pieceLeftFar", drivetrain.getAutoPath("3pieceLeftFar"));

    autoChooser.addOption("2pieceCenter", drivetrain.getAutoPath("2pieceCenter"));
    autoChooser.addOption("3pieceCenter", drivetrain.getAutoPath("3pieceCenter"));
    autoChooser.addOption("4pieceCenter", drivetrain.getAutoPath("4pieceCenter"));
    autoChooser.addOption("3pieceCenterNOTOP", drivetrain.getAutoPath("3pieceCenterNOTOP"));
    autoChooser.addOption("5pieceCenter", drivetrain.getAutoPath("5pieceCenter"));

    autoChooser.addOption("1pieceRight", drivetrain.getAutoPath("1pieceRight"));
    autoChooser.addOption("2pieceRight", drivetrain.getAutoPath("2pieceRight"));
    autoChooser.addOption("3PieceRight(far)", drivetrain.getAutoPath("3PieceRight"));
    autoChooser.addOption("4pieceRight", drivetrain.getAutoPath("4pieceRight"));




    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
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

 
  private SlewRateLimiter m_strafeX;
  private SlewRateLimiter m_strafeY;


  private void configureBindings() {
    // Master Control Bindings V
    // intakeWIndexerButton.whileTrue(mIntakeWithIndexerCommand);
    // intakeWOIndexerButton.whileTrue(mIntakeWithoutIndexerCommand);
    // speakerShootButton.whileTrue(mSpeakerShootCommand);
    // ampShootButton.whileTrue(mAmpShootCommand);
    // trapShootButton.whileTrue(mTrapShootCommand);
    // inverseIntakeButton.whileTrue(mInverseIntakeCommand);
    //

    // Manipulator Driver Bindings V
    manipulatorIntakeWIndexerButton.whileTrue(mIntakeWithIndexerCommand); //LB(5)
    manipulatorIntakeWOIndexerButton.whileTrue(mIntakeWithoutIndexerCommand); //RB(6)
    manipulatorSpeakerShootButton.whileTrue(mSpeakerShootCommand); //A(1)
    manipulatorAmpShootButton.whileTrue(mAmpShootCommand); //B(2)
    manipulatorTrapShootButton.whileTrue(mTrapShootCommand); //Y(4)
    manipulatorInverseIntakeButton.whileTrue(mInverseIntakeCommand); //X(3)
    //

    // // Driver Bindings V
    // driverRightClimbInButton.whileTrue(mRightClimbCommandIn); //RB(6)
    // driverLeftClimbInButton.whileTrue(mLeftClimbCommandIn); //LB(5)
    joystick.rightBumper().whileTrue(mRightClimbCommandIn);
    joystick.leftBumper().whileTrue(mLeftClimbCommandIn);
    joystick.rightTrigger(.5).whileTrue(mRightClimbCommandOut);
    joystick.leftTrigger(.5).whileTrue(mLeftClimbCommandOut);

    joystick.back().whileTrue(mSuperClimbCommand);

    //

    
    m_strafeX = new SlewRateLimiter(3);
    m_strafeY = new SlewRateLimiter(3);

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

    joystick.y().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.rightTrigger().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putData("turnQforward", drivetrain.turnQ(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("turnQreverse", drivetrain.turnQ(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("turnDforward", drivetrain.turnD(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("turnDreverse", drivetrain.turnD(SysIdRoutine.Direction.kReverse));

    
    
  }

}
