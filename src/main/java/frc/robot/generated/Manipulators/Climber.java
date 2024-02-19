package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.generated.TunerConstants.ShooterConstants;

public class Climber {
    private final TalonFX ShootMotorTop = new TalonFX(20);
    private final TalonFX ShootMotorBottom = new TalonFX(21);

    public void ClimbCommand(){
        ShootMotorTop.set(1);
        ShootMotorBottom.set(1);
    }

    public void NoClimbCommand(){
        ShootMotorTop.set(0);
        ShootMotorBottom.set(0);
    }
    
}
