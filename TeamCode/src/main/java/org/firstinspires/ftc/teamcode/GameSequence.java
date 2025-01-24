package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class GameSequence {
    private final DriveSubsystem drive;
    private final Vision vision;
    private final LinearOpMode opMode;

    public GameSequence(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.drive = new DriveSubsystem(hardwareMap);
        this.vision = new Vision(hardwareMap);
    }

    public void execute() {
        opMode.telemetry.addLine("Starting Game Sequence");
        opMode.telemetry.update();

        // Detect AprilTag and decide strategy
        Vision.TagPose tagPose = detectAprilTag();
        if (tagPose != null) {
            opMode.telemetry.addData("Tag ID", tagPose.tagID);
            opMode.telemetry.addData("Tag Position", "(x: %.2f, y: %.2f, z: %.2f)", tagPose.x, tagPose.y, tagPose.z);
            opMode.telemetry.addData("Tag Heading", "%.2f", tagPose.heading);
            opMode.telemetry.update();

            // Example: Different actions based on detected tag ID
            switch (tagPose.tagID) {
                case 1:
                    driveToLocation(30, 0, 0); // Go to position A
                    break;
                case 2:
                    driveToLocation(0, 30, 0); // Go to position B
                    break;
                case 3:
                    driveToLocation(-30, 0, 0); // Go to position C
                    break;
                default:
                    opMode.telemetry.addLine("Unknown Tag ID");
                    opMode.telemetry.update();
                    break;
            }
        } else {
            opMode.telemetry.addLine("No Tag Detected - Defaulting to Parking");
            opMode.telemetry.update();
            driveToLocation(0, 0, 0); // Default parking
        }

        // Perform scoring (optional)
        scoreElement();

        // Park at final location
        driveToLocation(0, 0, 0);

        opMode.telemetry.addLine("Game Sequence Complete");
        opMode.telemetry.update();
    }

    public Vision.TagPose detectAprilTag() {
        Vision.TagPose tagPose = vision.getRelativePose();
        if (tagPose != null) {
            opMode.telemetry.addData("Detected Tag ID", tagPose.tagID);
            opMode.telemetry.update();
        } else {
            opMode.telemetry.addLine("No Tags Detected");
            opMode.telemetry.update();
        }
        return tagPose;
    }

    public void driveToLocation(double x, double y, double heading) {
        drive.driveTo(new Pose2d(x, y, heading)); // Assuming DriveSubsystem handles this
        while (drive.isBusy() && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Driving to", "(%.2f, %.2f, %.2f)", x, y, heading);
            opMode.telemetry.update();
        }
    }

    public void scoreElement() {
        // Add logic to control mechanisms for scoring (e.g., arm, claw)
        opMode.telemetry.addData("Action", "Scoring Element");
        opMode.telemetry.update();
    }

    public void stop() {
        vision.close(); // Release camera resources
        opMode.telemetry.addLine("Resources Released");
        opMode.telemetry.update();
    }
}
