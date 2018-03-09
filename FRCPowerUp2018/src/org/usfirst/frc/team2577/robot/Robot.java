/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2577.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kForwardAuto = "Move Forward Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private boolean encoderLimitsEnabled = true;
	private boolean encodersCalibrated = false;
	private Joystick joy1;
	private Joystick joy2;
	private MecanumDrive drive;
	private Arm arm;
	private Hand hand;
	
	private Timer autoTimer;
	
	private UsbCamera camera1;
	private UsbCamera camera2;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kForwardAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		SmartDashboard.putBoolean("Encoder Wrist Limits", encoderLimitsEnabled);
		
		autoTimer = new Timer();
		
		camera1 = CameraServer.getInstance().startAutomaticCapture(0);
		camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera1.setResolution(320, 240);
		camera2.setResolution(320, 240);
		
		joy1 = new Joystick(0);
		joy2 = new Joystick(1);
		drive = new MecanumDrive(53, 56, 50, 51);
		arm = new Arm(57, 52, 55);
		hand = new Hand(58, 54);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoTimer.start();
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kForwardAuto:
				if(autoTimer.get() < 4000){
					drive.drive(0, 0, -0.5);
				}else{
					drive.drive(0, 0, 0);
				}
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}
	
	@Override 
	public void teleopInit(){
		encoderLimitsEnabled = SmartDashboard.getBoolean("Encoder Wrist Limits", false);
		System.out.println("Encoder limits: "+encoderLimitsEnabled);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		double direction = Math.atan2(joy1.getRawAxis(0), joy1.getRawAxis(1));
		double rotation = Math.pow(-joy1.getRawAxis(2), 3.0);  //power must be  1 + 0.4*strength   (so it has positive and negative)
		double powerLimit = (1-joy1.getRawAxis(3))/2;
		double magnitude = Math.min(1.0, Math.hypot(joy1.getRawAxis(0), joy1.getRawAxis(1)));
		
		if(joy1.getRawButton(12)){
			System.out.println("Shoulder: "+arm.getShoulderPosition());
			System.out.println("Wrist: "+arm.getWristPosition());
		}
		
		if(joy1.getRawButton(11)){
			arm.resetShoulderValues(arm.getShoulderPosition());
			arm.resetWristValues(arm.getWristPosition());
		}
		
		drive.drive(direction, rotation*powerLimit, magnitude*powerLimit);
		
		if(joy1.getRawButton(2) || joy2.getRawButton(6)){
			hand.spitOut();
		}else if(joy1.getRawButton(1) || joy2.getRawButton(5)){
			hand.suckIn();
		}else if(joy2.getRawAxis(3) > 0.4){
			hand.spitAtPower(joy2.getRawAxis(3));
		}else if(joy2.getRawAxis(2) > 0.4){
			hand.suckAtPower(joy2.getRawAxis(2));
		}else{
			hand.stop();
		}
		
		boolean movingWrist = false;
		if(joy2.getRawButton(1)){
			arm.moveShoulderTowardsTarget(arm.shoulderDownPosition);
			arm.moveWristTowardsTarget(arm.wristDownPosition);
			movingWrist = true;
		}else if(joy2.getRawButton(2)){
			arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition);
			arm.moveWristTowardsTarget(arm.wristSwitchPosition);
			movingWrist = true;
		}else if(joy2.getRawButton(4)){
			arm.moveShoulderTowardsTarget(arm.shoulderForwardPosition);
			arm.moveWristTowardsTarget(arm.wristForwardPosition);
			movingWrist = true;
		}else if(joy2.getRawButton(3)){
			arm.moveShoulderTowardsTarget(arm.shoulderUpPosition);
			arm.moveWristTowardsTarget(arm.wristUpPosition);
			movingWrist = true;
		}else if(joy1.getRawButton(3)){
			arm.moveShoulder(-1);
		}else if(joy1.getRawButton(5)){ //Raise Arm
			arm.moveShoulder(1);
		}else{
			arm.moveShoulder(-joy2.getRawAxis(1));
		}
		
		if(joy1.getRawButton(4)){
			arm.moveWrist(-1);
		}else if(joy1.getRawButton(6)){ //Raise Wrist
			arm.moveWrist(1);
		}else if(!movingWrist){
			arm.moveWrist(-joy2.getRawAxis(5));
		}
		/*
		if(encoderLimitsEnabled && !joy1.getRawButton(11) && !joy2.getRawButton(2)){
			if(arm.getShoulderPosition() < arm.shoulderWristFoldBottomLimit && arm.getShoulderPosition() > arm.shoulderWristFoldTopLimit){
				arm.moveWristTowardsTarget(arm.wristUpPosition);
			}
			
			if(arm.getShoulderPosition() > arm.shoulderDownPosition - 100){
				if(arm.getWristPosition() < arm.wristDownPosition){
					arm.moveWristTowardsTarget(arm.wristUpPosition);
				}
			}
		}
		*/
		
		
		if(joy1.getRawButton(8) || joy2.getPOV() == 180){ //Winch in
			arm.moveWinch(1);
		}else if(joy1.getRawButton(7) || joy2.getPOV() == 0){ //Winch out
			arm.moveWinch(-1);
		}else{
			arm.moveWinch(0);
		}
		Timer.delay(0.004);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	
	@Override
	public void testPeriodic() {
		
	}
}
