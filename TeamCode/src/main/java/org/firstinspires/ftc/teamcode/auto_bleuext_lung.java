package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Disabled
@Autonomous(name = "BLUEXT_lung")
public class auto_bleuext_lung extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx slider;
    private DcMotorEx intake;
    private DcMotor carusel;
    private Servo cuva;
    private DcMotorEx rotire;
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
        rotire = hardwareMap.get(DcMotorEx.class,"rotire");
        color = hardwareMap.get(ColorSensor.class, "color");

        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        rotire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();
        webcam.setPipeline(detectionPipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime2 = new ElapsedTime(0);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        Trajectory turnDuck = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-2,25,Math.toRadians(-90)))
                .build();

        Trajectory allignWithHub = drive.trajectoryBuilder(turnDuck.end())
                .lineToSplineHeading(new Pose2d(-40,25,Math.toRadians(-90)))
                .build();


        Trajectory forwardToHub = drive.trajectoryBuilder(allignWithHub.end())
                .forward(17)

                .build();
        Trajectory forwardToHub2 = drive.trajectoryBuilder(allignWithHub.end())
                .forward(27)

                .build();


       /* TrajectorySequence parkWarehouse = drive.trajectorySequenceBuilder(parkStorage.end())
                .strafeLeft(5)
                .lineToSplineHeading(new Pose2d(0,0,Math.toRadians(-90)))
                .forward(80)
                .build();*/
        //

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
        sleep(3000);
        while (!opModeIsActive() && !isStopRequested()) {
            //telemetry.addData("Zona", pipeline.getZone());
            cuva.setPosition(0.09);

            left_avg = (detectionPipeline.getZoneLuminosity(1) + detectionPipeline.getZoneLuminosity(2)) / 2;
            right_avg = (detectionPipeline.getZoneLuminosity(3) + detectionPipeline.getZoneLuminosity(4)) / 2;

            if (left_avg <= 123.5)
                zone = 1;
            else if (right_avg <= 123.5)
                zone = 2;
            else
                zone = 3;

            telemetry.addData("Zone", zone);
            telemetry.addData("Left", left_avg);
            telemetry.addData("Right", right_avg);

            telemetry.update();
        }
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (!opModeIsActive()) return;

        sleep(3000);
        drive.followTrajectory(turnDuck);
        runtime2.reset();
        while(runtime2.time()<3.5)
        {
            carusel.setPower(0.4);
        }
        carusel.setPower(0);
        sleep(500);
        drive.followTrajectory(allignWithHub);
        sleep(500);

        switch (zone)
        {
            case 1:
                slider.setTargetPosition(-200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(0.6);
                rotire.setTargetPosition(-1800);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);

                break;
            case 2:
                slider.setTargetPosition(-1200);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1850);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);
                break;

            case 3:
                slider.setTargetPosition(-1500);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1550);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);
                break;
        }
        sleep(2000);
//        if(zone==1) {
//            slider.setTargetPosition(0);
//            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slider.setPower(-0.3);
//            sleep(500);
//            drive.followTrajectory(forwardToHub2);
//
//        }
        if(zone==1) drive.followTrajectory(forwardToHub2);

        else drive.followTrajectory(forwardToHub);

        cuva.setPosition(0.5); // drop cube

        sleep(800);
        cuva.setPosition(0.09);
        sleep(200);
        Pose2d final_pose=drive.getPoseEstimate();

        int finalZone = zone;
        TrajectorySequence parkStorage = drive.trajectorySequenceBuilder(final_pose)
                .lineToLinearHeading(new Pose2d(-40,18,Math.toRadians(180)))
                .addTemporalMarker(0.5,()->{
                    if(finalZone==1||finalZone==2) {
                        slider.setTargetPosition(-1500);
                        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        slider.setPower(0.8);
                        rotire.setTargetPosition(-1550);
                        rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rotire.setPower(-0.8);
                    }
                })

                .build();
        Trajectory endTraj = drive.trajectoryBuilder(parkStorage.end())
                .lineToLinearHeading(new Pose2d(-24,28,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(parkStorage);

        sleep(200);

        slider.setTargetPosition(0);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        slider.setPower(-0.8);
        rotire.setTargetPosition(5);
        rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotire.setPower(0.7);

        drive.followTrajectory(endTraj);

        TrajectorySequence parkInWarehouse = drive.trajectorySequenceBuilder(endTraj.end())
                .lineToLinearHeading(new Pose2d(4,0,Math.toRadians(90)))
                .back(82)
                .build();

        drive.followTrajectorySequence(parkInWarehouse);
        sleep(200);

        int counter = 0;
        ElapsedTime runtime = new ElapsedTime(0);;
        runtime.startTime();

        while(opModeIsActive()) {
            Pose2d curPos = drive.getPoseEstimate();

            allignWithHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-5, 65, Math.toRadians(150)))

                    .build();

            if (counter > 0) {
                drive.followTrajectory(allignWithHub);

                slider.setTargetPosition(-1500);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                slider.setPower(0.6);
                rotire.setTargetPosition(-1470);
                rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotire.setPower(-0.8);


                forwardToHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(10 - counter * 1.5)
                        .build();

            }
            if (zone == 1 && counter == 0) {
                slider.setTargetPosition(-100);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slider.setPower(-0.3);
                forwardToHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(7 - counter)
                        .build();
            }

            drive.followTrajectory(forwardToHub);

            cuva.setPosition(0.5); // drop cube

            intake.setPower(0);

            sleep(600);
            cuva.setPosition(0.09);
            sleep(300);

            slider.setTargetPosition(0);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            slider.setPower(-0.7);
            rotire.setTargetPosition(-10);
            rotire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotire.setPower(0.6);

            cuva.setPosition(0.5);
            sleep(600);

            TrajectorySequence goToWarehouse = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(3.5 + counter * 2, 65, Math.toRadians(90)))
                    .back(30.5)
                    .addTemporalMarker(0.5, () ->
                    { // intake
                        intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        intake.setPower(0.6);
                    })
                    .build();

            drive.followTrajectorySequence(goToWarehouse);

//            if(counter == 1 && runtime.time() > 23)
//                break;
            if(counter <1)
            {
                boolean breakfrom = false;

                while (opModeIsActive() && !breakfrom) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -0.1,
                                    0,
                                    0
                            )
                    );

                    drive.update();
                    int c = 0;
                    while (c < 30) {
                        if (color.red() > 60 && color.green() > 60) {
                            cuva.setPosition(0.09);
                            intake.setDirection(DcMotorSimple.Direction.FORWARD);
                            intake.setPower(0.7);
                            breakfrom = true;
                            break;
                        }
                        sleep(5);
                        c++;
                    }
                }
                intake.setPower(0);

                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );


                TrajectorySequence backward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(Math.abs(drive.getPoseEstimate().getY())-60)
                        .build();



                drive.followTrajectorySequence(backward);
            }

            if(counter==1)
            {    intake.setPower(0);
                //traiectorie

//                TrajectorySequence endtraj=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .strafeLeft(28)
//                        .turn(Math.toRadians(90))
//                        .strafeLeft(25)
//                        .back(4)
//                        .build();
//                drive.followTrajectorySequence(endtraj);
                break;
            }
            counter++;
        }

    }
}