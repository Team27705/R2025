package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagPose;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;


/**
 * Gamepad 1(A) controls driving
 * Gamepad 2(B) controls intake
 *
 * */

@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends LinearOpMode {
    private RobotHardware robot;
    private boolean rbPressedA = false;
    private boolean lbPressedA = false;

    private boolean lbPressedB = false;
    private boolean rbPressedB = false;
    private long lastSpeedChangeTimeA = 0;
    private long lastSpeedChangeTimeB = 0;
    private static final long debounceTime = 250;

    private static final double servoIncrement = 10.0 / 150;

    private boolean armMoving = false;
    private long lastArmMoveTime = 0;

    private static final long armDebounceTime = 100;

    private boolean xPressed = false;
    private boolean yPressed = false;
    private long lastServoChangeTime = 0;
    private static final long servoDebounceTime = 250; // 250ms debounce time


    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Combine telemetry with dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.init();

        // Enable camera stream in Driver Station app
        if (robot.vision.getVisionPortal() != null) {
            robot.vision.getVisionPortal().setProcessorEnabled(robot.vision.getAprilTagProcessor(), true);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick = Drive + Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriveControls();
            handleSpeedControls();
            handleUtilityControls();
            handleIntakeControls();
            updateTelemetry();
        }
        robot.drivetrain.stop();
        robot.intake.hold();
        robot.vision.close();
    }

    private void handleDriveControls() {
        // Get joystick values
        double drive = gamepad1.left_stick_y;   // Forward/back
        double strafe = gamepad1.left_stick_x;   // Left/right
        double turn = gamepad1.right_stick_x;    // Turning

        // Apply deadband
        if (Math.abs(drive) < ControllerConstants.STICK_DEADBAND) drive = 0;
        if (Math.abs(strafe) < ControllerConstants.STICK_DEADBAND) strafe = 0;
        if (Math.abs(turn) < ControllerConstants.STICK_DEADBAND) turn = 0;

        // Apply speed multiplier and drive
        robot.drivetrain.setMecanumPower(
                drive * DriveConstants.SPEED_MULTIPLIER,
                strafe * DriveConstants.SPEED_MULTIPLIER,
                turn * DriveConstants.SPEED_MULTIPLIER
        );
    }

    private void handleIntakeControls() {
        double turn = gamepad2.right_stick_y;

        long currentTime = System.currentTimeMillis();

        if (gamepad2.x && !xPressed && currentTime - lastServoChangeTime > servoDebounceTime) {
            robot.intake.servoControl(servoIncrement);
            lastServoChangeTime = currentTime;
            xPressed = true;
        } else if (!gamepad2.x) {
            xPressed = false;
        }

        if (gamepad2.y && !yPressed && currentTime - lastServoChangeTime > servoDebounceTime) {
            robot.intake.servoControl(-servoIncrement);
            lastServoChangeTime = currentTime;
            yPressed = true;
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        if (turn >= .5 ) {
            turn = .15;
            robot.intake.armMotorControl(turn * Constants.IntakeConstants.SPEED_MULTIPLIER);
        }
        else if (turn <= -.5) {
            turn = -.15;
            robot.intake.armMotorControl(turn * Constants.IntakeConstants.SPEED_MULTIPLIER);
        }
        else {
            robot.intake.armMotorControl(0);
        }
        //handle debouncing

        if (gamepad2.x){
            robot.intake.servoControl(servoIncrement);
        }
        if (gamepad2.y) {
            robot.intake.servoControl(-servoIncrement);
        }
        if (gamepad2.a){
            robot.intake.resetMotor();
        }
        if (gamepad2.b){
            robot.intake.hold();
        }
        if (gamepad2.left_bumper) {
            robot.intake.dialServo();
        }
    }

    private void handleSpeedControls() {
        // Adjust speed multiplier with bumpers
        // need to implement debouncing
        long currentTimeA = System.currentTimeMillis();
        long currentTimeB = System.currentTimeMillis();

        if (gamepad1.right_bumper && !rbPressedA && currentTimeA - lastSpeedChangeTimeA > debounceTime) {
            if (DriveConstants.SPEED_MULTIPLIER < 1.0){
                DriveConstants.SPEED_MULTIPLIER += 0.25;
                lastSpeedChangeTimeA = currentTimeA;
            }
            rbPressedA = true;
        } else if (!gamepad1.right_bumper){
            rbPressedA = false;
        }

        if (gamepad1.left_bumper && !lbPressedA && currentTimeA - lastSpeedChangeTimeA > debounceTime) {
            if (DriveConstants.SPEED_MULTIPLIER > 0.25){
                DriveConstants.SPEED_MULTIPLIER -= 0.25;
                lastSpeedChangeTimeB = currentTimeA;
            }
            lbPressedA = true;
        } else if (!gamepad1.left_bumper){
            lbPressedA = false;
        }

        if (gamepad2.left_bumper && !lbPressedB && currentTimeB - lastSpeedChangeTimeB > debounceTime) {
            if (Constants.IntakeConstants.SPEED_MULTIPLIER > 1) {
                Constants.IntakeConstants.SPEED_MULTIPLIER -= .25;
                lastSpeedChangeTimeA = currentTimeB;
            }
            lbPressedB = true;
        }
        else if (!gamepad2.left_bumper) {
            lbPressedB = false;
        }
        if (gamepad2.right_bumper && !rbPressedB && currentTimeB - lastSpeedChangeTimeB > debounceTime) {
            if (DriveConstants.SPEED_MULTIPLIER < 1.0){
                DriveConstants.SPEED_MULTIPLIER += 0.25;
                lastSpeedChangeTimeA = currentTimeA;
            }
            rbPressedB = true;
        } else if (!gamepad2.right_bumper){
            rbPressedB = false;
        }
    }

    private void handleUtilityControls() {
        // Reset encoders with Y button
        if (gamepad1.y) {
            robot.drivetrain.resetEncoders();
            robot.intake.resetEncoders();
        }
        if (gamepad1.a) {
            robot.drivetrain.stop();
        }


    }
    // http://192.168.43.1:8080/dash
    private void updateTelemetry() {
        telemetry.addData("=== DRIVER CONTROLS ===", "");
        telemetry.addData("Drive Power", "%.2f", -gamepad1.left_stick_y * DriveConstants.SPEED_MULTIPLIER);
        telemetry.addData("Turn Power", "%.2f", gamepad1.left_stick_x * DriveConstants.SPEED_MULTIPLIER);
        telemetry.addData("Drivetrain Speed Multiplier", "%.2f", DriveConstants.SPEED_MULTIPLIER);

        telemetry.addData("== OPERATOR CONTORLS ==", "");
        telemetry.addData("Arm Speed Multiplier", "%.2f", Constants.IntakeConstants.SPEED_MULTIPLIER);
        // Add AprilTag pose information
        TagPose pose = robot.vision.getRelativePose();
        telemetry.addData("\n=== APRILTAG DATA ===", "");
        if (pose != null) {
            telemetry.addData("Tag X", "%.2f in", pose.x);
            telemetry.addData("Tag Y", "%.2f in", pose.y);
            telemetry.addData("Tag Z", "%.2f in", pose.z);
            telemetry.addData("Tag Heading", "%.2f°", pose.heading);
            telemetry.addData("Tag ID", "%d", pose.tagID);
            telemetry.addData("Distance from Tag", "%.2f in",
                    Math.sqrt(pose.x * pose.x + pose.y * pose.y));
        } else {
            telemetry.addData("Tag Status", "No tag detected");
        }
        telemetry.addData("\n=== ENCODER DATA ===", "");
        telemetry.addData("\n=== CONTROLS ===", "");
        telemetry.addData("Drive", "Left Stick = Move + Turn");
        telemetry.addData("Speed", "Bumpers = Adjust Speed");
        telemetry.addData("Utility", "Y = Reset Encoders");

        telemetry.addData("\nArm Rotate", "Right Joystick");
        telemetry.addData("X and Y turn the servo Clockwise and counterclockwise", "");
        telemetry.addData("B for ascension A to return to normal state", "");
        telemetry.addData("left bumper to reset servo", "");

        telemetry.update();
    }
}