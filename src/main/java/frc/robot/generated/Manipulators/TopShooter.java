package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.generated.TunerConstants.ShooterConstants;

public class TopShooter {
    private final TalonFX ShootMotorTop = new TalonFX(20);
    private final TalonFX ShootMotorBottom = new TalonFX(21);

    public void ShootCommand(){
        ShootMotorTop.set(ShooterConstants.TTOPSHOOTERSPEED);
        ShootMotorBottom.set(ShooterConstants.TBOTTOMSHOOTERSPEED);
    }

    public void StopShootingCommand(){
        ShootMotorTop.set(0);
        ShootMotorBottom.set(0);
    }
    
}
