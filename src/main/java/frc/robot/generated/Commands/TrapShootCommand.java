package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.TrapShooter;

public class TrapShootCommand extends Command{

    private final TrapShooter mShooter;

    public TrapShootCommand(TrapShooter shooter){
        mShooter = shooter;
    }
    
    @Override
    public void execute(){
        mShooter.TrapShootCommand();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
