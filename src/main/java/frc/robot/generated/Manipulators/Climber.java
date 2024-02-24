package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


public class Climber {
    private final TalonFX LeftClimber = new TalonFX(4);
    private final TalonFX RightClimber = new TalonFX(5);

    public void ClimbCommand(){
        LeftClimber.set(-.15);
        RightClimber.set(.15);
    }

    public void LeftClimbCommandIn(){
        LeftClimber.set(-.15);
    }

    public void LeftClimbCommandout(){
        LeftClimber.set(.15);
    }
    
    public void RightClimbCommandIn(){
        RightClimber.set(-.15);
    }
    
    public void RightClimbCommandOut(){
        RightClimber.set(.15);
    }

    public void ClimbCommandReversed(){
        LeftClimber.set(.15);
        RightClimber.set(-.15);
    }

    public void NoClimbCommand(){
        LeftClimber.set(0);
        RightClimber.set(0);
    }
    
}
