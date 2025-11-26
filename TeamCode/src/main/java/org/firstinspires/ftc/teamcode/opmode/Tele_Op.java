package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

// FTCLib Command Framework
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

// FTCLib Gamepad
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

// PedroPathing
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

// Team Hardware + Utilities
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotData;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathingConstants;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;

// Telemetry
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.function.Supplier;

/*
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactInCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.ArtifactShootCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakeOutCommand;
import org.firstinspires.ftc.teamcode.commands.advancedcommand.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.IntakeStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystem.ShooterStateCommand;
*/

@Disabled
@TeleOp(name = "TeleOpV2")
public class Tele_Op extends CommandOpMode {

    // Robot + Data
    private final Robot robot = Robot.getInstance();
    private final RobotData data = Robot.getInstance().data;

    // Gamepad
    private GamepadEx g1;

    // Pathing + Telemetry
    public static Pose startingPose;
    private TelemetryManager telemetryM;
    private Supplier<PathChain> goShootPath;
    private boolean automatedDrive;

    // State flags
    private boolean teleOpEnabled = false;
    private boolean grabConfirmed = false;
    private double fieldCentricOffset;

    // Last button states
    private boolean lastLeftTrigger, lastRightTrigger;
    private boolean lastA, lastB, lastX, lastY;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastRightStickButton, lastLeftStickbutton;
    private boolean lastPS, lastStart, lastBack;

    @Override
    public void initialize() {
        g1 = new GamepadEx(gamepad1);
        Globals.IS_AUTO = false;

        // Lazy Curve Generation
        goShootPath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, Math.toRadians(135), 0.8))
                .build();

        robot.initialize(hardwareMap, telemetry);
    }

    @Override
    public void run() {
        if (teleOpEnabled) {
            CommandScheduler.getInstance().run();

            robot.periodic();
            robot.updateData();
            robot.write();

            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x / 2,
                    Constants.robotCentric // Robot Centric
            );
        }

        // Button inputs
        boolean a = g1.getButton(GamepadKeys.Button.A);
        boolean b = g1.getButton(GamepadKeys.Button.B);
        boolean x = g1.getButton(GamepadKeys.Button.X);
        boolean y = g1.getButton(GamepadKeys.Button.Y);
        boolean leftBumper = g1.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rightBumper = g1.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        boolean dpadUp = g1.getButton(GamepadKeys.Button.DPAD_UP);
        boolean dpadDown = g1.getButton(GamepadKeys.Button.DPAD_DOWN);
        boolean dpadLeft = g1.getButton(GamepadKeys.Button.DPAD_LEFT);
        boolean dpadRight = g1.getButton(GamepadKeys.Button.DPAD_RIGHT);
        boolean rightStickButton = g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean leftStickButton = g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
        boolean ps = gamepad1.ps;
        boolean start = g1.getButton(GamepadKeys.Button.START);
        boolean back = g1.getButton(GamepadKeys.Button.BACK);

        // Toggle robot-centric mode
        if (!lastX && x) {
            Constants.robotCentric = !Constants.robotCentric;
            gamepad1.rumble(500);
            if (Constants.robotCentric)
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            else
                gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // Enable TeleOp
        if (!lastStart && start) {
            teleOpEnabled = true;
            gamepad1.rumble(2000);
        }
    /*
    // Shooter + Intake Commands
        // Right Bumper: Sort Shoot
        if (rightBumper && !lastRightBumper) {
            CommandScheduler.getInstance().schedule(
                    new ShooterStateCommand(shooterSubsystem, ShooterSubsystem.ShootState.SORTSHOOT) // made-up action
            );
        }



        // Left Bumper: Stop Intake
        if (leftBumper && !lastLeftBumper) {
            CommandScheduler.getInstance().schedule(
                    new IntakeStateCommand(IntakeSubsystem.IntakeState.STOP)
            );
        }

        // B: Stop Shooter
        if (b && !lastB) {
            CommandScheduler.getInstance().schedule(
                    new ShooterStateCommand(shooterSubsystem, ShooterSubsystem.ShootState.STOP)
            );
        }

        // A: Intake Reverse
        if (a && !lastA) {
            CommandScheduler.getInstance().schedule(
                    new IntakeStateCommand(IntakeSubsystem.IntakeState.OUT)
            );
        }
        */

        // Update last button states
        lastA = a; lastB = b; lastX = x; lastY = y;
        lastLeftBumper = leftBumper; lastRightBumper = rightBumper;
        lastDpadUp = dpadUp; lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft; lastDpadRight = dpadRight;
        lastRightStickButton = rightStickButton; lastLeftStickbutton = leftStickButton;
        lastPS = ps; lastStart = start;

        // Trigger inputs
        boolean leftTrigger = gamepad1.left_trigger > .5;
        boolean rightTrigger = gamepad1.right_trigger > .5;

        /* Trigger Commands
        // Right Trigger: Speed Shoot
        if (rightTrigger && !lastRightTrigger) {
            CommandScheduler.getInstance().schedule(
                    new ShooterStateCommand(shooterSubsystem, ShooterSubsystem.ShootState.SPEEDSHOOT) // made-up action
            );
        }
        // Left Trigger: Start Intake
        if (leftTrigger && !lastLeftTrigger) {
            CommandScheduler.getInstance().schedule(
                    new IntakeStateCommand(IntakeSubsystem.IntakeState.IN)
            );
        }
    */
        // Touchpad reset pose
        if (gamepad1.touchpad) {
            robot.follower.setPose(new Pose());
            gamepad1.rumble(1000);
            gamepad1.setLedColor(0, 1, 0, 2000);
        }
    }

    private void scheduleCommand(boolean lastPress, boolean currPress, Command command) {
        if (currPress && !lastPress) {
            CommandScheduler.getInstance().schedule(command);
        }
    }
}