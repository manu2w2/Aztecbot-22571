package org.firstinspires.ftc.teamcode.opModes.Autonomos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.CloseTopeCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.OpenTopeCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetHoodPositionCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetIntakeVelocityCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SetTurretPositionCommand;
import org.firstinspires.ftc.teamcode.COMANDOS_AUTONOMOS.SpinUpShooterCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import Subsistemas.HoodSubsystem;
import Subsistemas.IntakeSubsystem_Autonomous;
import Subsistemas.ShooterSubsystem;
import Subsistemas.TopeSubsystem;
import Subsistemas.TurretSubsystem_Autonomous;

@Autonomous(name = "Autonomo azul lejos", group = "Autonomous")
@Configurable
public class Autonomo_azul_lejos extends CommandOpMode {
    private Follower follower;


    private final Pose startPose = new Pose(53.803, 9.112210200927344, Math.toRadians(0));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2;

    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        waitForStart();

        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        ciclo1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(53.803, 9.405),
                                new Pose(56.000, 18.723)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 18.723),
                                new Pose(45.235, 34.959)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(45.235, 34.959),
                                new Pose(13.676, 35.284)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.676, 35.284),
                                new Pose(56.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.000, 18.723),
                                new Pose(15.5, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(15.5, 8.894),
                                new Pose(17.200, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(17.200, 8.894),
                                new Pose(15.5, 8.894)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.5, 8.894),
                                new Pose(56.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(
                new FollowPathCommand(follower, ciclo1, true, 1)
        );

    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}