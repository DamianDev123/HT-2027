package org.firstinspires.ftc.teamcode.Solvers.Controllers;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import java.util.function.BooleanSupplier;


// Thanks Powercube from Watt-sUP 16166, we copied verbatim

/**
 * Allows you to run a PathChain or a Path (which is then converted into a PathChain) by scheduling it.
 * holdEnd is set to true by default, so you only need to give it your instance of follower and the Path to follow.
 * <p>
 * To see an example usage of this command, look at <a href="https://github.com/FTC-23511/SolversLib/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/PedroCommandSample/FollowPedroSample.java">https://github.com/FTC-23511/SolversLib/blob/master/examples/src/main/java/org/firstinspires/ftc/teamcode/PedroCommandSample/FollowPedroSample.java</a>
 *
 * @author Arush - FTC 23511
 * @author Saket - FTC 23511
 *
 */
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private boolean holdEnd;
    private double maxPower = 1.0;

    public FollowPathCommand(Follower follower, PathChain pathChain) {
        this(follower, pathChain, true);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this(follower, pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, double maxPower) {
        this(follower, pathChain, true, maxPower);
    }

    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    public FollowPathCommand(Follower follower, Path pathChain) {
        this(follower, pathChain, true);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd) {
        this(follower, pathChain, holdEnd, 1.0);
    }

    public FollowPathCommand(Follower follower, Path pathChain, double maxPower) {
        this(follower, pathChain, true, maxPower);
    }

    public FollowPathCommand(Follower follower, Path pathChain, boolean holdEnd, double maxPower) {
        this.follower = follower;
        this.pathChain = new PathChain(pathChain);
        this.holdEnd = holdEnd;
        this.maxPower = maxPower;
    }

    /**
     * Sets Global Maximum Power for Follower, and overwrites maxPower in constructor
     *
     * @param globalMaxPower The new globalMaxPower
     * @return This command for compatibility in command groups
     */
    public FollowPathCommand setGlobalMaxPower(double globalMaxPower) {
        follower.setMaxPower(globalMaxPower);
        maxPower = globalMaxPower;
        return this;
    }

    @Override
    public void initialize() {
        if (maxPower != 1.0) {
            follower.followPath(pathChain, maxPower, holdEnd);
        }
        follower.followPath(pathChain, holdEnd);
    }
    public BooleanSupplier supplier = new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return follower.getCurrentTValue()>0.35 ;
        }
    };
    public Command halfWay(Command... parallel) {
        ParallelCommandGroup group = new ParallelCommandGroup(this);
        group.addCommands(new SequentialCommandGroup(new WaitUntilCommand(supplier),new SequentialCommandGroup(parallel)));
        return group;
    }
    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue()>0.7 ;
    }
}