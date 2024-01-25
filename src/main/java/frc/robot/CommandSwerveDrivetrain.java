package frc.robot;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SimSwerveDrivetrain.SimSwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // private SwerveModule [] m_swerveModules;
    // private CommandSwerveDrivetrain m_odometry;
    // private SwerveModulePosition [] getModulePositions(){
    //     ArrayList<SwerveModulePosition> modulePositions = new ArrayList<>();

    //     for (SwerveModule swerveModule :  m_swerveModules) {
    //         modulePositions.add(swerveModule.getPosition(true));
    //     }
    // }
    // private SwerveDriveKinematics m_Kinematics;
    // public CommandSwerveDrivetrain() {
    //    m_Kinematics = new SwerveDriveKinematics(null);
    //    m_odometry = new SwerveDriveOdometry(m_Kinematics,getPigeon2(),getModule);
    // }

    private static class DriveSetVoltageRequest implements SwerveRequest {

        public double voltage = 0;

        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            SwerveModuleState state = new SwerveModuleState();
            for (int i = 0; i < modulesToApply.length; i++) {
                try {
                    Field setter = modulesToApply[i].getClass().getDeclaredField("m_velocityVoltageSetter");
                    setter.setAccessible(true);
                    VelocityVoltage request = (VelocityVoltage) setter.get(modulesToApply[i]);
                    request.FeedForward = voltage;
                    modulesToApply[i].apply(state, DriveRequestType.Velocity);
                } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            return StatusCode.OK;
        }

    }

    private static class TurnSetVoltageRequest implements SwerveRequest {

        public double voltage = 0;

        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            SwerveModuleState state = new SwerveModuleState();
            try {
                Field setter = modulesToApply[0].getClass().getDeclaredField("m_angleVoltageSetter");
                setter.setAccessible(true);
                MotionMagicVoltage request = (MotionMagicVoltage) setter.get(modulesToApply[0]);
                request.FeedForward = voltage;
                modulesToApply[0].apply(state, DriveRequestType.Velocity, SteerRequestType.MotionMagic);
            } catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            return StatusCode.OK;
        }

    }

    private final SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::setAllMotorVoltage, null, this)
    );

    private final StatusSignal<Double> m_turnPos = this.Modules[0].getCANcoder().getPosition().clone();
    private final StatusSignal<Double> m_turnVel = this.Modules[0].getCANcoder().getVelocity().clone();
    private final StatusSignal<Double> m_turnVoltage = this.Modules[0].getSteerMotor().getMotorVoltage().clone();
    
    private final MutableMeasure<Angle> m_turnPosMeasure = MutableMeasure.mutable(Units.Rotations.of(m_turnPos.getValueAsDouble()));
    private final MutableMeasure<Velocity<Angle>> m_turnVelMeasure = MutableMeasure.mutable(Units.RotationsPerSecond.of(m_turnVel.getValueAsDouble()));
    private final MutableMeasure<Voltage> m_turnVoltageMeasure = MutableMeasure.mutable(Units.Volts.of(m_turnVoltage.getValueAsDouble()));
    private final SysIdRoutine turnRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::setModuleVolt, (log) -> {
            BaseStatusSignal.refreshAll(m_turnPos, m_turnVel, m_turnVoltage);
            log.motor("FL-Module-Steer")
                .angularPosition(m_turnPosMeasure.mut_replace(m_turnPos.getValueAsDouble(), Units.Rotations))
                .angularVelocity(m_turnVelMeasure.mut_replace(m_turnVel.getValueAsDouble(), Units.RotationsPerSecond))
                .voltage(m_turnVoltageMeasure.mut_replace(m_turnVoltage.getValueAsDouble(), Units.Volts));
        }, this)
    );

    private final DriveSetVoltageRequest driveVoltReq = new DriveSetVoltageRequest();
    private final TurnSetVoltageRequest turnVoltReq = new TurnSetVoltageRequest();
    
    


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);


    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    private void setAllMotorVoltage(Measure<Voltage> voltage) {
        driveVoltReq.voltage = voltage.magnitude(); 
        // this.setControl(driveVoltReq);
    }

    private void setModuleVolt(Measure<Voltage> voltage) {
        turnVoltReq.voltage = voltage.magnitude(); 
        // this.setControl(turnVoltReq);
    }
    
    public Command turnQ(SysIdRoutine.Direction direction) {
        return turnRoutine.quasistatic(direction);
    }

    public Command turnD(SysIdRoutine.Direction direction) {
        return turnRoutine.dynamic(direction);
    }

    public Command driveQ(SysIdRoutine.Direction direction) {
        return driveRoutine.quasistatic(direction);
    }

    public Command driveD(SysIdRoutine.Direction direction) {
        return driveRoutine.dynamic(direction);
    }
    // class SimSwerveModule {
    //     private SwerveModulePosition currenntPosition = new SwerveModulePosition();
    //     private SwerveModuleState currenState = new SwerveModuleState();
    //     public SwerveModulePosition getPosition() {
    //         return currenntPosition;
    //     }
    //     public SwerveModuleState getState(){
    //         return currenState;
    //     }
    //     public void setTargetState(SwerveModuleState targetState) {
            
    //     }
    // }
}
