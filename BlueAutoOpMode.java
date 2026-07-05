package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Blue Auto Vision (Semantic Drive)", group="Auto")
public class BlueAutoOpMode extends LinearOpMode {

    // ==========================================
    //           1. 视觉测距核心参数
    // ==========================================
    private static final int TARGET_ID = 2;
    private static final double REAL_TAG_WIDTH = 165.0;   // mm
    private static final double FOCAL_CONSTANT = 309.1;

    private static final double TARGET_DIST_MM = 1200.0;

    // ==========================================
    //           2. PID 参数
    // ==========================================
    private static final double SEARCH_SPEED = 0.25;

    private static final double DRIVE_KP = 0.0025;
    private static final double TURN_KP  = 0.005;

    private static final double DIST_TOLERANCE_MM = 30.0;
    private static final int ALIGN_TOLERANCE_X = 5;

    // ==========================================

    private DcMotor frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {

        // ================= 硬件初始化 =================
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // 电机方向（与 TeleOp 完全一致）
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HuskyLens
        if (!huskyLens.knock()) {
            telemetry.addData("Warning", "HuskyLens not responding!");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "Ready (Semantic Drive Enabled)");
        telemetry.update();

        waitForStart();

        // =======================================================
        //           阶段 1：向前冲刺搜索 Tag
        // =======================================================
        boolean tagFound = false;

        while (opModeIsActive() && !tagFound) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks.length > 0) {
                for (HuskyLens.Block block : blocks) {
                    if (block.id == TARGET_ID) {
                        tagFound = true;
                        break;
                    }
                }
            }

            if (!tagFound) {
                driveForward(SEARCH_SPEED);
                telemetry.addData("Mode", "Searching (Forward)");
            } else {
                stopDrive();
                telemetry.addData("Mode", "Tag Found - Brake");
            }
            telemetry.update();
        }

        sleep(120);

        // =======================================================
        //           阶段 2：视觉闭环锁定
        // =======================================================
        while (opModeIsActive()) {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            boolean targetInSight = false;
            double currentDistMM = 0;
            double driveError = 0;
            double turnError = 0;

            if (blocks.length > 0) {
                for (HuskyLens.Block block : blocks) {
                    if (block.id == TARGET_ID) {
                        targetInSight = true;

                        if (block.width > 0) {
                            currentDistMM = (REAL_TAG_WIDTH * FOCAL_CONSTANT) / block.width;
                        }

                        driveError = currentDistMM - TARGET_DIST_MM;
                        turnError  = block.x - 160;   // 右正左负
                        break;
                    }
                }
            }

            if (targetInSight) {

                double forwardPower = Range.clip(driveError * DRIVE_KP, -0.6, 0.6);
                double turnPower    = Range.clip(turnError  * TURN_KP,  -0.5, 0.5);

                if (Math.abs(driveError) < DIST_TOLERANCE_MM &&
                        Math.abs(turnError)  < ALIGN_TOLERANCE_X) {

                    stopDrive();
                    telemetry.addData("Status", "LOCKED");
                } else {
                    driveForwardAndTurn(forwardPower, turnPower);
                    telemetry.addData("Status", "Adjusting");
                }

                telemetry.addData("Dist", "%.0f mm (Err %.0f)", currentDistMM, driveError);
                telemetry.addData("TurnErr", "%.0f", turnError);

            } else {
                stopDrive();
                telemetry.addData("Status", "Target Lost");
            }

            telemetry.update();
        }
    }

    // =======================================================
    //           语义层运动封装（核心）
    // =======================================================

    /** 向前 / 后 */
    private void driveForward(double power) {
        moveRobot(0, power, 0);
    }

    /** 向前 + 转向 */
    private void driveForwardAndTurn(double forward, double turn) {
        moveRobot(0, forward, turn);
    }

    /** 左右平移 */
    private void strafe(double power) {
        moveRobot(power, 0, 0);
    }

    /** 原地旋转 */
    private void turn(double power) {
        moveRobot(0, 0, power);
    }

    /** 急停 */
    private void stopDrive() {
        moveRobot(0, 0, 0);
    }

    // =======================================================
    //           底层：完全一致的 TeleOp 运动学
    // =======================================================
    private void moveRobot(double drive, double strafe, double turn) {

        double fl = drive + strafe + turn;
        double fr = drive - strafe + turn;
        double rl = drive - strafe - turn;
        double rr = drive + strafe - turn;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(rl), Math.abs(rr))));

        if (max > 1.0) {
            fl /= max; fr /= max; rl /= max; rr /= max;
        }

        frontLeftDrive.setPower(fl);
        rearLeftDrive.setPower(rl);
        frontRightDrive.setPower(fr);
        rearRightDrive.setPower(rr);
    }
}
