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
        LeftClimber.set(-.5);
    }

    public void LeftClimbCommandout(){
        LeftClimber.set(.75);
    }
    
    public void RightClimbCommandIn(){
        RightClimber.set(-.5);
    }
    
    public void RightClimbCommandOut(){
        RightClimber.set(.75);
    }

    public void ClimbCommandReversed(){
        LeftClimber.set(.15);
        RightClimber.set(-.15);
    }
    public void SuperClimbCommand(){
        LeftClimber.set(1);
        RightClimber.set(1);
    }

    public void NoClimbCommand(){
        LeftClimber.set(0);
        RightClimber.set(0);
    }
    
}
