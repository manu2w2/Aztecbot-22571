package org.firstinspires.ftc.teamcode.opModes.Autonomos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Autonomo rojo lejos", group = "Autonomous")
@Configurable
public class Autonomo_rojo_lejos extends CommandOpMode {
    private Follower follower;
    private IntakeSubsystem_Autonomous intake;
    private TurretSubsystem_Autonomous turret;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private TopeSubsystem Tope;

    private final Pose startPose = new Pose(87.716, 9.112, Math.toRadians(180));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain Ciclo1, Ciclo2, Ciclo2part2, Salir, Ciclo3parte2;

    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new IntakeSubsystem_Autonomous(hardwareMap);
        turret = new TurretSubsystem_Autonomous(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);
        Tope = new TopeSubsystem(hardwareMap);

        register(intake, turret, shooter, hood, Tope);

        buildPaths();

        waitForStart();

        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        Ciclo1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.716, 9.112),
                                new Pose(87.716, 18.400)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        Ciclo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.716, 18.400),
                                new Pose(98.766, 34.959)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(98.766, 34.959),
                                new Pose(130.325, 35.284)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Ciclo2part2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.325, 35.284),
                                new Pose(88.000, 18.723)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Salir = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 18.723),
                                new Pose(108.144, 13.518)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter, 1470, 20),
                        new SetTurretPositionCommand(turret, -251),
                        new FollowPathCommand(follower, Ciclo1, true, 1),
                        new SetHoodPositionCommand(hood,0.7),
                        new CloseTopeCommand(Tope)

                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 950),
                        new OpenTopeCommand(Tope)
                ),
                new WaitCommand(2700),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo2, true, 1),
                        new SpinUpShooterCommand(shooter, 1490, 20),
                        new SetTurretPositionCommand(turret, -279),
                        new CloseTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake,1600)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo2part2, true, 1),
                        new SetIntakeVelocityCommand(intake,0)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new OpenTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake,950)
                ),
                new WaitCommand(2700),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake,0),
                        new CloseTopeCommand(Tope),
                        new FollowPathCommand(follower, Salir, true, 1),
                        new SetTurretPositionCommand(turret,0),
                        new SpinUpShooterCommand(shooter,0,20)
                )


        );

    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}