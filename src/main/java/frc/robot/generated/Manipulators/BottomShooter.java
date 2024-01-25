package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.generated.TunerConstants.ShooterConstants;

public class BottomShooter {
    private final TalonFX ShootMotorTop = new TalonFX(0);
    private final TalonFX ShootMotorBottom = new TalonFX(1);

    public void ShootCommand(){
        ShootMotorTop.set(ShooterConstants.BTOPSHOOTERSPEED);
        ShootMotorBottom.set(ShooterConstants.BBOTTOMSHOOTERSPEED);
    }

    public void StopShootingCommand(){
        ShootMotorTop.set(0);
        ShootMotorBottom.set(0);
    }
    
}
