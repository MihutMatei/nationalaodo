package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;

@Autonomous(name = "AUTONOMOUS_bluext")
public class auto_bluext extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx slider;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private Servo rotire;
    boolean bCameraOpened = false;
    private ColorSensor color;

    //unfinished

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        carusel = hardwareMap.get(DcMotor.class, "carusel");
        cuva = hardwareMap.get(Servo.class,"cuva");
        rotire = hardwareMap.get(Servo.class,"rotire");
        color = hardwareMap.get(ColorSensor.class, "color");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);



        //----------------------------------------------------------------------------------------------

        //traiectorii blueside extern
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        Trajectory turnToShipLevel3 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-9,0,Math.toRadians(215)))
                .build();

        Trajectory turnToShipLevel2 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-15,-10,Math.toRadians(215)))
                .addTemporalMarker(0.1,()->{
                    rotire.setPosition(0.15);
                })
                .build();

        Trajectory turnToShipLevel1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-15,-10,Math.toRadians(215)))
                .addTemporalMarker(0.1,()->{
                    rotire.setPosition(0.1);
                })
                .build();


        //x -9  y 7

        // x -1 y 20 pt duck

        // x -19 y 24 pt duck scan

        // x -37 y12 perpedincular cu shiphubu pt punerea duck

        // x -23 y 29 parking

        //----------------------------------------------------------------------------------------------


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                bCameraOpened = true;
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        detectionPipeline.setGridSize(2);

        double left_avg,right_avg;

        int zone = 0;
        sleep(5000);
        while (!opModeIsActive() && !isStopRequested()) {
            //telemetry.addData("Zona", pipeline.getZone());
            cuva.setPosition(0.09);
            rotire.setPosition(1);


                left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
                right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;

                if (left_avg <= 125)
                    zone = 1;
                else if (right_avg <= 125)
                    zone = 2;
                else
                    zone = 3;

                telemetry.addData("Zone", zone);
                telemetry.addData("Left", left_avg);
                telemetry.addData("Right", right_avg);

            telemetry.update();
        }
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pose2d endPose = turnToShipLevel1.end();

        if (opModeIsActive()) {
            if (zone == 1 || zone == 0) {
                drive.followTrajectory(turnToShipLevel1);



                sleep(500);
                cuva.setPosition(0.5);
                sleep(1000);

                rotire.setPosition(1);

                sleep(500);
                endPose = turnToShipLevel1.end();
            }
            if (zone == 2) {
                drive.followTrajectory(turnToShipLevel2);


                sleep(500);
                cuva.setPosition(0.5);
                sleep(1000);

                rotire.setPosition(1);

                sleep(500);

                endPose = turnToShipLevel2.end();
            }
            if (zone == 3) {
                drive.followTrajectory(turnToShipLevel3);

                endPose = turnToShipLevel3.end();


                slider.setTargetPosition(-1200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.7);

                rotire.setPosition(0.2);

                while(slider.isBusy())
                {

                }
                sleep(500);
                cuva.setPosition(0.5);

                sleep(1000);

                slider.setTargetPosition(0);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(-0.7);
                rotire.setPosition(1);

                sleep(500);
            }
            cuva.setPosition(0.09);

            Trajectory turnDuck = drive.trajectoryBuilder(endPose)
                    .lineToSplineHeading(new Pose2d(-2,30,Math.toRadians(-90)))
                    .build();

            Trajectory toStorage = drive.trajectoryBuilder(turnDuck.end())
                    .lineToSplineHeading(new Pose2d(-30,30,Math.toRadians(180)))
                    .build();


            drive.followTrajectory(turnDuck);

            runtime2.reset();
            while (runtime2.time() < 4) {

                carusel.setPower(0.35);

            }

           /* webcam.setPipeline(detectionPipeline);
            detectionPipeline.setGridSize(7);*/

            /*sleep(1000);
            drive.followTrajectory(toStorage);

            final double degrees = 3.5; // should be 3.5 but put it to 4 due to wheel error

            int duckZone = detectionPipeline.getDuckZone();
            int column = detectionPipeline.getColumn(duckZone);

            int degreesMult = column - 5;
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(0.6);
            Trajectory pickupDuck = drive.trajectoryBuilder(toStorage.end())
                    .lineToSplineHeading(new Pose2d(-29,18,Math.toRadians(180-degrees*degreesMult)))
                    .build();
            Trajectory goDuck = drive.trajectoryBuilder(pickupDuck.end())
                    .back(28)
                    .build();*/

           /* drive.followTrajectory(pickupDuck);*/
            sleep(200);
           /* drive.followTrajectory(goDuck);*/

            Trajectory goPark = drive.trajectoryBuilder(turnDuck.end())
                    .lineToSplineHeading(new Pose2d(-25,30,Math.toRadians(-90)))
                    .build();

            sleep(500);

         /*   if(color.red() > 30 && color.green() > 30) {
                cuva.setPosition(0.09);
                intake.setPower(0);
                //traiectorie spre ship
            }
            else*/ drive.followTrajectory(goPark);





        }
    }
}