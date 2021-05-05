/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase {

	private SupplyCurrentLimitConfiguration driveMotorCurrentConfig;

	private DifferentialDriveKinematics kinematics;
	private DifferentialDriveOdometry driveOdometry;
	// use robot characterization gets voltage to go at a certain velocity 
	// this is an estimation
	// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html
	private SimpleMotorFeedforward feedforward;
	private PIDController leftPIDController, rightPIDController;
	private RamseteController ramseteController;
	private TrajectoryConfig trajectoryConfig;
	private Pose2d pose;

	private TalonFX leftMaster;
	private TalonFX leftFollower;
	private TalonFX rightMaster;
	private TalonFX rightFollower;

	private AHRS gyro;

	public Drivetrain() {
		leftMaster = new TalonFX(2);
		leftFollower = new TalonFX(1);
		rightMaster = new TalonFX(3);
		rightFollower = new TalonFX(5);
	
		gyro = new AHRS(SPI.Port.kMXP);

		leftMaster.configFactoryDefault();
		rightMaster.configFactoryDefault();
		leftFollower.configFactoryDefault();
		rightFollower.configFactoryDefault();

		leftMaster.setInverted(false);
		leftFollower.setInverted(false);
		rightMaster.setInverted(true);
		rightFollower.setInverted(true);

		leftMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		leftFollower.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		rightMaster.configVoltageCompSaturation(10, Constants.kTimeoutMs);
		rightFollower.configVoltageCompSaturation(10, Constants.kTimeoutMs);

		leftMaster.enableVoltageCompensation(true);
		leftFollower.enableVoltageCompensation(true);
		rightMaster.enableVoltageCompensation(true);
		rightFollower.enableVoltageCompensation(true);

		driveMotorCurrentConfig = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

		leftMaster.configSupplyCurrentLimit(driveMotorCurrentConfig);
		leftFollower.configSupplyCurrentLimit(driveMotorCurrentConfig);
		rightMaster.configSupplyCurrentLimit(driveMotorCurrentConfig);
		rightFollower.configSupplyCurrentLimit(driveMotorCurrentConfig);

		leftMaster.configOpenloopRamp(.4);
		leftFollower.configOpenloopRamp(.4);
		rightMaster.configOpenloopRamp(.4);
		rightFollower.configOpenloopRamp(.4);

		leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		leftFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);
		rightFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
				Constants.kTimeoutMs);

		resetEncoder();
		configNeutralMode(NeutralMode.Coast, NeutralMode.Coast);
		gyro.reset();

		kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);
		driveOdometry = new DifferentialDriveOdometry(getHeading());
		feedforward = new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
		leftPIDController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
		rightPIDController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
		pose = new Pose2d();

		trajectoryConfig = new TrajectoryConfig(Constants.kMaxVelocityMetersPerSecond,
				Constants.kMaxAccelerationMetersPerSecondSq);
		trajectoryConfig.setReversed(false);

		ramseteController = new RamseteController();
	}

	@Override
	public void periodic() {
		driveOdometry.update(getHeading(), getLeftDistance(), getRightDistance());
		pose = driveOdometry.getPoseMeters();

		System.out.println("leftMaster: "+leftMaster.getMotorOutputPercent());
		System.out.println("rightMaster: "+rightMaster.getMotorOutputPercent());
	}

	public void resetOdometry() {
		setOdometry(new Pose2d());
	}

	public void setOdometry(Pose2d newPose) {
		pose = newPose;
		resetEncoder();
		gyro.reset();
		driveOdometry.resetPosition(newPose, getHeading());
	}

	public double getLeftDistance() {
		double circumference = Math.PI * Constants.kWheelDiameterMeters;
		double motorTicks = (leftMaster.getSelectedSensorPosition() + leftFollower.getSelectedSensorPosition()) / 2;
		return motorTicks / Constants.kFalconTicksPerRotation / Constants.kDriveGearRatio  * circumference;
	}

	public double getRightDistance() {
		double circumference = Math.PI * Constants.kWheelDiameterMeters;
		double motorTicks = (rightMaster.getSelectedSensorPosition() + rightFollower.getSelectedSensorPosition()) / 2;
		return motorTicks / Constants.kFalconTicksPerRotation / Constants.kDriveGearRatio  * circumference;
	}

	public DifferentialDriveWheelSpeeds getSpeeds() {
		double circumference = Math.PI * Constants.kWheelDiameterMeters;

		return new DifferentialDriveWheelSpeeds(
				leftMaster.getSelectedSensorVelocity() / Constants.kFalconTicksPerRotation
						/ Constants.kDriveGearRatio * circumference * 10,
				leftMaster.getSelectedSensorVelocity() / Constants.kFalconTicksPerRotation
						/ Constants.kDriveGearRatio * circumference * 10);
	}

	public void setOutput(double leftVolts, double rightVolts) {
		setLeftRightMotorOutputs(leftVolts / 10.0, rightVolts / 10.0);
	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-gyro.getAngle());
	}

	public void resetEncoder() {
		leftMaster.setSelectedSensorPosition(0);
		rightMaster.setSelectedSensorPosition(0);
		leftFollower.setSelectedSensorPosition(0);
		rightFollower.setSelectedSensorPosition(0);
	}

	public void configNeutralMode(NeutralMode _mode){
		configNeutralMode(_mode, _mode);
	}
	public void configNeutralMode(NeutralMode mode1, NeutralMode mode2) {
		leftMaster.setNeutralMode(mode1);
		rightMaster.setNeutralMode(mode1);
		leftFollower.setNeutralMode(mode2);
		rightFollower.setNeutralMode(mode2);
	}

	public void setLeftRightMotorOutputs(double left, double right) {
		leftMaster.set(ControlMode.PercentOutput, left);
		leftFollower.set(ControlMode.PercentOutput, left);
		rightFollower.set(ControlMode.PercentOutput, right);
		rightMaster.set(ControlMode.PercentOutput, right);
	}

	public void log() {
		SmartDashboard.putNumber("NavX", gyro.getAngle());
	}

	public void invertPathDirection(boolean reversed){
		trajectoryConfig.setReversed(reversed);
	}

	public PIDController getLeftPIDController() {
		return leftPIDController;
	}

	public PIDController getRightPIDController() {
		return rightPIDController;
	}

	public RamseteController getRamseteController() {
		return ramseteController;
	}

	public TrajectoryConfig getTrajectoryConfig() {
		return trajectoryConfig;
	}

	public Pose2d getPose() {
		return pose;
	}

	public SimpleMotorFeedforward getFeedForward() {
		return feedforward;
	}

	public DifferentialDriveKinematics getKinematics() {
		return kinematics;
	}
}