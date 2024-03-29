package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(20).withKI(0).withKD(.2)
        .withKS(0.14641).withKV(0.0031).withKA(0.0024);
        // .withKS(0.14641).withKV(2.4461).withKA(0.1524);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.2).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 12;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = true;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "CANivore";
    private static final int kPigeonId = 0;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static class ShooterConstants{

        //New Shooter Speeds
        //STS = Speaker Top Shooter
        //SBS = Speaker Bottom Shooter
        public static final double STS = 1;
        public static final double SBS = -.4;
        //AMPTS = Amp Top Shooter
        //AMPBS = Amp Bottom Shooter
        public static final double ATS = .025;
        public static final double ABS = -.15;
        //TTS = Trap Top Shooter
        //TBS = Trap Bottom Shooter
        public static final double TTS = 0;
        public static final double TBS = 0;
        //ITS = Inverse Top Shooter
        //IBS = Inverse Bottom Shooter
        public static final double ITS = -.35;
        public static final double IBS = .35;
        //

        //Testing needed V (Y)
        public static final double TOPSHOOTERSPEED = .2;
        public static final double BOTTOMSHOOTERSPEED = -.5;
        // //Not effective V (B)
        public static final double SLOWTOPSHOOTERSPEED = .15;
        public static final double SLOWBOTTOMSHOOTERSPEED = -.15;
        //Testing needed V (B)
        public static final double TTOPSHOOTERSPEED = .35;
        public static final double TBOTTOMSHOOTERSPEED = -.35;
        //Amp shooter V (RB)
        public static final double BTOPSHOOTERSPEED = .025;
        public static final double BBOTTOMSHOOTERSPEED = -.15;
        
    }
        public static class IntakeConstants{
                public static final double INTAKESPEED = .2;
        }

//     public static class SwerveConstants {
//         public static final double TRACK_WIDTH_METER = Units.inchesToMeters(24.75);
//         public static final double WHEEL_BASE_METER = Units.inchesToMeters(24.75);
//         public static final Translation2d[] POSITIONS = new Translation2d[] {
//                 new Translation2d(WHEEL_BASE_METER / 2, TRACK_WIDTH_METER /2),
//                 new Translation2d(WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER /2),
//                 new Translation2d(-WHEEL_BASE_METER / 2, TRACK_WIDTH_METER /2),
//                 new Translation2d(-WHEEL_BASE_METER / 2, -TRACK_WIDTH_METER /2)
//         };
//     }

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 12;
    private static final int kFrontLeftSteerMotorId = 13;
    private static final int kFrontLeftEncoderId = 13;
    private static final double kFrontLeftEncoderOffset = -0.4130859375;

    private static final double kFrontLeftXPosInches = 12.375;
    private static final double kFrontLeftYPosInches = 12.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = 2;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 3;
    private static final double kFrontRightEncoderOffset = 0.024169921875;

    private static final double kFrontRightXPosInches = 12.375;
    private static final double kFrontRightYPosInches = -12.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = 14;
    private static final int kBackLeftSteerMotorId = 15;
    private static final int kBackLeftEncoderId = 15;
    private static final double kBackLeftEncoderOffset = -0.376708984375;

    private static final double kBackLeftXPosInches = -12.375;
    private static final double kBackLeftYPosInches = 12.375;

    // Back Right
    private static final int kBackRightDriveMotorId = 1;
    private static final int kBackRightSteerMotorId = 0;
    private static final int kBackRightEncoderId = 0;
    private static final double kBackRightEncoderOffset = -0.46923828125;

    private static final double kBackRightXPosInches = -12.375;
    private static final double kBackRightYPosInches = -12.375;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide)
            .withSteerMotorInverted(false);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}
