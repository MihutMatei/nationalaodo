package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.drive.advanced.DetectionPipeline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.SamplePipeline;
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
@Autonomous(name = "AUTONOMOUS_blueint")
public class auto_blueint extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    DetectionPipeline detectionPipeline;
    private DcMotorEx cremaliera;
    private DcMotorEx cascade;
    private DcMotorEx intake;
    private DcMotor carusel;
    private CRServo ruleta;
    private CRServo ruleta_x;
    private CRServo ruleta_z;
    static int zona=0;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        cremaliera =hardwareMap.get(DcMotorEx.class,"cremaliera");
        cascade =hardwareMap.get(DcMotorEx.class,"cascade");
        intake =hardwareMap.get(DcMotorEx.class,"intake");
        carusel=hardwareMap.get(DcMotor.class, "carusel");
        carusel.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);



        pipeline = new SamplePipeline();
        detectionPipeline = new DetectionPipeline();

        webcam.setPipeline(pipeline);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime runtime1 = new ElapsedTime(0);
        ElapsedTime runtime2 = new ElapsedTime(0);
        ElapsedTime runtime3 = new ElapsedTime(0);
        ElapsedTime runtime4 = new ElapsedTime(0);


        //----------------------------------------------------------------------------------------------

        //traiectorii blueside intern
        Pose2d startPose= new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        Trajectory f1 = drive.trajectoryBuilder(startPose)

                //.lineToSplineHeading(new Pose2d(24.5,-25,Math.toRadians(-8)))
                .lineToSplineHeading(new Pose2d(24,-22,Math.toRadians(-12)))

                .build();
        Trajectory warehouse = drive.trajectoryBuilder(f1.end())
                .lineToSplineHeading(new Pose2d(1.5,0.5,Math.toRadians(90)))
                .addTemporalMarker(0.1,()->
                {   cremaliera.setTargetPosition(0);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(1);

                })
                .build();


        Trajectory f2 =drive.trajectoryBuilder(warehouse.end())
                .forward(26) // daca nu 40
                .build();

        Trajectory ia_bila_cub = drive.trajectoryBuilder(f2.end())
                .back(40)
                .build();

        Trajectory revers_card = drive.trajectoryBuilder(ia_bila_cub.end())
                .lineToLinearHeading(new Pose2d(26., -21, Math.toRadians(-12)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-1900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();

        Trajectory ionutz = drive.trajectoryBuilder(revers_card.end())
                .lineToLinearHeading(new Pose2d(0.4, -5, Math.toRadians(90)))
                .addTemporalMarker(0.1,()->
                {
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);
                })
                .build();

        Trajectory inapoi = drive.trajectoryBuilder(ionutz.end())
                .forward(40)
                .build();

        Trajectory ia_bila_cub2 = drive.trajectoryBuilder(inapoi.end())
                .back(40)
                .build();
        Trajectory revers_card2 = drive.trajectoryBuilder(ia_bila_cub2.end())
                .lineToLinearHeading(new Pose2d(25.5, -20, Math.toRadians(-12)))
                .addTemporalMarker(0.1,()->
                {
                    cremaliera.setTargetPosition(-1900);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);


                })
                .build();
        Trajectory ionutz2 = drive.trajectoryBuilder(revers_card2.end())
                .lineToLinearHeading(new Pose2d(0.4, -5, Math.toRadians(90)))
                .addTemporalMarker(0.1,()->
                {
                    cascade.setTargetPosition(-20);
                    /* cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cascade.setPower(0.4);
                    cremaliera.setTargetPosition(-20);
                    cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    cremaliera.setVelocity(3000);
                    intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    intake.setPower(0.7);
                })
                .build();
        Trajectory inapoi2 = drive.trajectoryBuilder(ionutz2.end())
                .forward(40)
                .build();


        //----------------------------------------------------------------------------------------------

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

     
//         webcam.setPipeline(detectionPipeline);
//         detectionPipeline.setGridSize(6);
        while (!opModeIsActive() && !isStopRequested()) {

               telemetry.addData("Zona", pipeline.getZone());
//             int duck = detectionPipeline.getDuckZone();
//             int column = detectionPipeline.getColumn(duck);
//             telemetry.addData("Zona", duck);
//             telemetry.addData("Column", column);

               telemetry.update();
        }

        int zone = pipeline.getZone();

        webcam.setPipeline(detectionPipeline);
        detectionPipeline.setGridSize(5);

        drive.followTrajectory(f1);

        /**sleep(500); asta era aici inainte de update dar nu cred ca e necesar idk*/

        if(zone == 1 || zone == 0)
        {   cremaliera.setTargetPosition(-1400);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy())
            {

            }


            cascade.setTargetPosition(-250);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            while (cascade.isBusy())
            {

            }
            sleep(200);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.4);
            sleep(600);
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);


        }
        if(zone==2)
        {
            cremaliera.setTargetPosition(-2100);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy()){

            }
            sleep(200);
            cascade.setTargetPosition(-600);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);

            while (cascade.isBusy())
            {

            }

            sleep(200);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.3);
            sleep(600);

            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);


        }
        if(zone==3)
        {
            cremaliera.setTargetPosition(-2950);
            cremaliera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cremaliera.setVelocity(3000);
            while(cremaliera.isBusy()){

            }
            sleep(200);
            cascade.setTargetPosition(-700);
            cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            sleep(200);
            while(cascade.isBusy())
            {

            }
            sleep(200);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.7);
            sleep(600);
            cascade.setTargetPosition(0);
            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
        }


        sleep(400);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);
        drive.followTrajectory(warehouse);
        int counter = 0;
        while(counter < 3 && !isStopRequested()) {

            drive.followTrajectory(f2);
//
            sleep(200);
            int cubeZone = detectionPipeline.getBestZone(DetectionPipeline.ZoneType.E_RIGHT);
            int column = detectionPipeline.getColumn(cubeZone);


            double colVal = column - 2;

            double colMult = 5;

            double colMove = colVal * colMult;

            if(colMove == 0.0)
                colMove = 0.1;

            telemetry.addData("Zone", cubeZone);
            telemetry.addData("ColVal", colMove);
            telemetry.update();

            //---------------------------------------------------------------------
            Trajectory cubeScan = drive.trajectoryBuilder(f2.end())
                    .strafeRight(colMove)

                    .build();
            Trajectory cubeCatch = drive.trajectoryBuilder(cubeScan.end())
                    .forward(2 + counter * 3.25)

                    .build();
            Trajectory cubeAway = drive.trajectoryBuilder(cubeCatch.end())
                    .back(2 + counter * 3.25)

                    .build();
            Trajectory positionWarehouse = drive.trajectoryBuilder(cubeAway.end())
                    .strafeLeft(colMove)

                    .build();
            Trajectory backwards = drive.trajectoryBuilder(positionWarehouse.end())
                    .back(26)

                    .build();
            Trajectory toShipp = drive.trajectoryBuilder(backwards.end())
                    .lineToLinearHeading(new Pose2d(27, -22, Math.toRadians(-17)))
                    .addTemporalMarker(0.1, () -> {
                        cremaliera.setTargetPosition(-2950);

                        cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cremaliera.setVelocity(3000);


                    })

                    .build();
            drive.followTrajectory(cubeScan);
            drive.followTrajectory(cubeCatch);
            sleep(300);
            drive.followTrajectory(cubeAway);
            drive.followTrajectory(positionWarehouse);
            drive.followTrajectory(backwards);
            drive.followTrajectory(toShipp);

            cascade.setTargetPosition(-750);

            cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascade.setPower(0.4);
            sleep(400);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(0.7);
            sleep(400);
            Trajectory warehouse2 = drive.trajectoryBuilder(toShipp.end())
                    .lineToSplineHeading(new Pose2d(1.65, 0.5, Math.toRadians(90)))
                    .addTemporalMarker(0.1, () ->
                    {

                        cremaliera.setTargetPosition(0);
                        cremaliera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cremaliera.setVelocity(3000);
                        cascade.setTargetPosition(-10);

                        cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        cascade.setPower(0.4);
                        intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        intake.setPower(1);

                    })
                    .build();
            drive.followTrajectory(warehouse2);
            f2 = drive.trajectoryBuilder(warehouse2.end())
                    .forward(26 + counter * 2.5) // daca nu 40
                    .build();
            counter++;
        }





    }
}
