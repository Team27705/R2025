package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision {
    private final HardwareMap hardwareMap;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initAprilTag();
    }

    private void initAprilTag() {
        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(
                        VisionConstants.FX, VisionConstants.FY, VisionConstants.CX, VisionConstants.CY
                )
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setNumThreads(1)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, VisionConstants.WEBCAM_NAME))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(VisionConstants.RESOLUTION_WIDTH,
                        VisionConstants.RESOLUTION_HEIGHT))
                .enableLiveView(true)
                .setLiveViewContainerId(cameraMonitorViewId)
                .setAutoStopLiveView(false)
                .setAutoStartStreamOnBuild(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    public TagPose getRelativePose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections != null && !currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);
            
            if (detection != null && detection.ftcPose != null) {
                return new TagPose(
                        detection.id,
                        detection.robotPose,
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z,
                        detection.ftcPose.yaw
                );
            }
        }
        return null;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public static class TagPose {
        public int tagID;
        public Pose3D robotPose;
        public double x;
        public double y;
        public double z;
        public double heading;

        public TagPose(int tagID, Pose3D robotPose, double x, double y, double z, double heading) {
            this.tagID = tagID;
            this.robotPose = robotPose;
            this.x = x;
            this.y = y;
            this.z = z;
            this.heading = heading;
        }
    }
} 