package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Vision.TagPose;

@TeleOp(name = "Arcade Drive", group = "Drive")
public class ArcadeDrive extends LinearOpMode {
    private RobotHardware robot;
    private double speedMultiplier = 1.0;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Combine telemetry with dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Left Stick = Drive + Turn");
        telemetry.addData("Speed Control", "Bumpers = Adjust Speed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDriveControls();
            handleSpeedControls();
            handleUtilityControls();
            updateTelemetry();
        }

        robot.drivetrain.stop();
    }

    private void handleDriveControls() {
        // Get joystick values and apply deadband
        double turn = gamepad1.left_stick_y;
        double drive = -gamepad1.left_stick_x;

        // Apply deadband
        if (Math.abs(drive) < ControllerConstants.STICK_DEADBAND) drive = 0;
        if (Math.abs(turn) < ControllerConstants.STICK_DEADBAND) turn = 0;

        // Apply speed multiplier and drive
        robot.drivetrain.setPower(drive * speedMultiplier, turn * speedMultiplier);
    }

    private void handleSpeedControls() {
        // Adjust speed multiplier with bumpers
        if (gamepad1.right_bumper && speedMultiplier < 1.0) {
            speedMultiplier += 0.25;
        }
        if (gamepad1.left_bumper && speedMultiplier > 0.25) {
            speedMultiplier -= 0.25;
        }
    }

    private void handleUtilityControls() {
        // Reset encoders with Y button
        if (gamepad1.y) {
            robot.drivetrain.resetEncoders();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("=== DRIVER CONTROLS ===", "");
        telemetry.addData("Drive Power", "%.2f", -gamepad1.left_stick_y * speedMultiplier);
        telemetry.addData("Turn Power", "%.2f", gamepad1.left_stick_x * speedMultiplier);
        telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);

        // Add AprilTag pose information
        TagPose pose = robot.vision.getRelativePose();
        telemetry.addData("\n=== APRILTAG DATA ===", "");
        if (pose != null) {
            telemetry.addData("Tag X", "%.2f in", pose.x);
            telemetry.addData("Tag Y", "%.2f in", pose.y);
            telemetry.addData("Tag Z", "%.2f in", pose.z);
            telemetry.addData("Tag Heading", "%.2f°", pose.heading);
            telemetry.addData("Distance from Tag", "%.2f in",
                    Math.sqrt(pose.x * pose.x + pose.y * pose.y));
        } else {
            telemetry.addData("Tag Status", "No tag detected");
        }

        telemetry.addData("\n=== ENCODER DAT A ===", "");
        telemetry.addData("Left Position", robot.drivetrain.getLeftPosition());
        telemetry.addData("Right Position", robot.drivetrain.getRightPosition());

        telemetry.addData("\n=== CONTROLS ===", "");
        telemetry.addData("Drive", "Left Stick = Move + Turn");
        telemetry.addData("Speed", "Bumpers = Adjust Speed");
        telemetry.addData("Utility", "Y = Reset Encoders");

        telemetry.update();
    }
}