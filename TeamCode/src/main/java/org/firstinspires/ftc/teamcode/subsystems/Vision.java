package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

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

        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, VisionConstants.WEBCAM_NAME),
                cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(camera, 60); 

        // Start camera stream to dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 60);

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
    }

    public TagPose getRelativePose() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        if (!currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);

            return new TagPose(
                    detection.ftcPose.x,
                    detection.ftcPose.y,
                    detection.ftcPose.z,
                    detection.ftcPose.yaw
            );
        }
        return null;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public static class TagPose {
        public double x, y, z, heading;

        public TagPose(double x, double y, double z, double heading) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.heading = heading;
        }
    }
} 