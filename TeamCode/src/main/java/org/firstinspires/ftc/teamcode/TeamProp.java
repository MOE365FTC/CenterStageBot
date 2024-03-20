
package org.firstinspires.ftc.teamcode;

import static org.opencv.features2d.Features2d.DrawMatchesFlags_DRAW_RICH_KEYPOINTS;
import static org.opencv.features2d.Features2d.drawKeypoints;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeamProp extends LinearOpMode{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode){

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline{
        boolean viewportPaused;


        @Override
        public Mat processFrame(Mat input){
            SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();
            params.set_filterByColor(false);

            params.set_minThreshold(10f);
            params.set_maxThreshold(200f);

            params.set_filterByArea(true);
            params.set_minArea(2000);
            params.set_maxArea(100000);

            params.set_filterByCircularity(true);
            params.set_minCircularity(0.1f);

            params.set_filterByConvexity(true);
            params.set_minConvexity(0.3f);

            params.set_filterByInertia(true);
            params.set_minInertiaRatio(0.05f);


            SimpleBlobDetector detector = SimpleBlobDetector.create(params);
            MatOfKeyPoint keyPoints = new MatOfKeyPoint();

            detector.detect(input, keyPoints);

//            for(int i = 0; i < keyPoints.size(0)){
//                keyPoints.get(0, 0);
//            }

            List<KeyPoint> convPoints = keyPoints.toList();
            List<Double> xPoints = new ArrayList<Double>();
            List<Double> yPoints = new ArrayList<Double>();

            int propPos;

            for(int i = 0; i < keyPoints.size(0); i++){
                xPoints.add(convPoints.get(i).pt.x);
                yPoints.add(convPoints.get(i).pt.y);
            }

            for(int i = 0; i < keyPoints.size(0); i++){
                if (xPoints.get(i) < 600) propPos = 1;
                else if (xPoints.get(i) > 1400) propPos = 3;
                else propPos = 2;
                drawKeypoints(input, keyPoints, input, new Scalar(0, 255, 0), DrawMatchesFlags_DRAW_RICH_KEYPOINTS);

                telemetry.addData("Prop Pos", propPos);
                telemetry.update();
            }

            return input;
        }

        @Override
        public void onViewportTapped(){
            viewportPaused = !viewportPaused;

            if(viewportPaused){
                webcam.pauseViewport();
            }else{
                webcam.resumeViewport();
            }
        }
    }
}