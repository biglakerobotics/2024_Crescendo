package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Climber;

public class LeftClimbCommandIn extends Command{

    private final Climber mClimber;

    public LeftClimbCommandIn(Climber climber){
        mClimber = climber;
    }
    
    @Override
    public void execute(){
        mClimber.LeftClimbCommandIn();
    }
    @Override
    public void end(boolean interrupted){
        mClimber.NoClimbCommand();
    }

}
