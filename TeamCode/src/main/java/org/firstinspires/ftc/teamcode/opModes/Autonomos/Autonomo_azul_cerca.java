package org.firstinspires.ftc.teamcode.opModes.Autonomos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
@Autonomous(name = "Autonomo azul cerca", group = "Autonomous")
@Configurable
public class Autonomo_azul_cerca extends CommandOpMode {
    private Follower follower;
    public static double Velocity1= 1160;
    public static double TurretPosition1 = 197;
    public static double TurretPosition2 = 42;
    private IntakeSubsystem_Autonomous intake;
    private TurretSubsystem_Autonomous turret;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private TopeSubsystem Tope;
    private final Pose startPose = new Pose(24.223050486238442, 126.05918501060792, Math.toRadians(53.5));

    // PathChains (sin cambiar nombres ni trayectorias)
    private PathChain ciclo1, ciclo2, ciclo2part2, Ciclo3, Ciclo3parte2, Ciclo4,Ciclo4parte2,Ciclo5,CIclo5part2;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        intake = new IntakeSubsystem_Autonomous(hardwareMap);
        turret = new TurretSubsystem_Autonomous(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap);
        Tope = new TopeSubsystem(hardwareMap);

        buildPaths();
        waitForStart();
        if (opModeIsActive()) {
            schedule(createAutonomousSequence());
        }
    }

    public void buildPaths() {
        // Mantengo exactamente tus paths originales
        ciclo1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(24.223050486238442, 126.05918501060792), new Pose(56.425, 82.888)))
                .setLinearHeadingInterpolation(Math.toRadians(53.5), Math.toRadians(180))
                .build();

        ciclo2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.425, 82.888),new Pose(45.725, 58.763)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierLine(new Pose(45.725, 058.763),new Pose(21.076, 58.702)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        ciclo2part2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(21.076, 58.702), new Pose(48.936, 60.047), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Ciclo3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(56.425, 82.888), new Pose(37.253, 55.456), new Pose(15.704, 57.332)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150.5))
                .build();

        Ciclo3parte2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(15.704, 57.332), new Pose(44.406, 62.236), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(150.5))
                .build();

        Ciclo4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(56.425, 82.888), new Pose(37.253, 55.456), new Pose(15.704, 57.332)))
                .setConstantHeadingInterpolation(Math.toRadians(150.5))
                .build();

        Ciclo4parte2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(15.704, 57.332), new Pose(44.406, 62.236), new Pose(56.425, 82.888)))
                .setLinearHeadingInterpolation(Math.toRadians(150.5), Math.toRadians(180))
                .build();

        Ciclo5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.425, 82.888), new Pose(18.653, 82.522)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        CIclo5part2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.653, 82.522), new Pose(56.425, 82.888)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }

    private SequentialCommandGroup createAutonomousSequence() {
        return new SequentialCommandGroup(

                // === Primer disparo + movimiento inicial ===
                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter, Velocity1, 20),
                        new SetTurretPositionCommand(turret, TurretPosition1),
                        new FollowPathCommand(follower, ciclo1, true, 1),
                        new CloseTopeCommand(Tope),
                        new SetHoodPositionCommand(hood, 0.24)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1700),
                        new OpenTopeCommand(Tope)
                ),
                new WaitCommand(1350),

                // === Ciclo 2 ===
                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter, 400, 20),
                        new FollowPathCommand(follower, ciclo2, true, 1),
                        new CloseTopeCommand(Tope)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, ciclo2part2, true, 1),
                        new SpinUpShooterCommand(shooter, Velocity1, 20),
                        new SetIntakeVelocityCommand(intake, 0)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1700),
                        new OpenTopeCommand(Tope)
                ),
                new WaitCommand(1300),
                new CloseTopeCommand(Tope),

                // === Ciclo 3 y 4 (parte problemática corregida) ===
                new ParallelCommandGroup(
                        new SetTurretPositionCommand(turret, TurretPosition2),
                        new SpinUpShooterCommand(shooter, 400, 20),
                        new FollowPathCommand(follower, Ciclo3, true, 1),
                        new SetIntakeVelocityCommand(intake, 1400)
                ),
                new WaitCommand(1200),

                // Secuencia de retorno e intake (usamos Sequential para evitar conflictos)
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo3parte2, true, 1),
                        new SpinUpShooterCommand(shooter, Velocity1, 20),
                        new SetIntakeVelocityCommand(intake, 1700)
                ),
                new WaitCommand(200),
                new OpenTopeCommand(Tope),
                new WaitCommand(1300),

                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1500),
                        new SpinUpShooterCommand(shooter, 400, 20),
                        new FollowPathCommand(follower, Ciclo4, true, 1),
                        new CloseTopeCommand(Tope)
                ),
                new WaitCommand(1500),

                new ParallelCommandGroup(
                        new SpinUpShooterCommand(shooter, Velocity1, 20),
                        new FollowPathCommand(follower, Ciclo4parte2, true, 1),
                        new SetIntakeVelocityCommand(intake, 0)
                ),

                new WaitCommand(200),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 1700),
                        new OpenTopeCommand(Tope)
                ),
                new WaitCommand(1300),

                // === Ciclo final ===
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, Ciclo5, true, 1),
                        new CloseTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake, 1500),
                        new SetTurretPositionCommand(turret, TurretPosition1)
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, CIclo5part2, true, 1),
                        new SetIntakeVelocityCommand(intake, 0)
                ),
                new WaitCommand(200),

                new ParallelCommandGroup(
                        new OpenTopeCommand(Tope),
                        new SetIntakeVelocityCommand(intake, 1700)
                ),
                new WaitCommand(1350),
                new ParallelCommandGroup(
                        new SetIntakeVelocityCommand(intake, 0),
                        new SpinUpShooterCommand(shooter, 0, 20),
                        new SetTurretPositionCommand(turret, 0),
                        new CloseTopeCommand(Tope)
                )
        );
    }

    @Override
    public void run() {
        follower.update();
        CommandScheduler.getInstance().run();
    }
}