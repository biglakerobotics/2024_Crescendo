package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.generated.TunerConstants.ShooterConstants;

public class SlowShooter {
    private final TalonFX ShootMotorTop = new TalonFX(20);
    private final TalonFX ShootMotorBottom = new TalonFX(21);

    public void SlowShootCommand(){
        ShootMotorTop.set(ShooterConstants.SLOWTOPSHOOTERSPEED);
        ShootMotorBottom.set(ShooterConstants.SLOWBOTTOMSHOOTERSPEED);
    }

    public void StopShootingCommand(){
        ShootMotorTop.set(0);
        ShootMotorBottom.set(0);
    }
    
}
