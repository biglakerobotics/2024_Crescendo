package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Climber;

public class LeftClimbCommandOut extends Command{

    private final Climber mClimber;

    public LeftClimbCommandOut(Climber climber){
        mClimber = climber;
    }
    
    @Override
    public void execute(){
        mClimber.LeftClimbCommandout();
    }
    @Override
    public void end(boolean interrupted){
        mClimber.NoClimbCommand();
    }

}
