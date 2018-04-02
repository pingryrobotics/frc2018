/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2577.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
	private String m_autoSelected;
	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> teleopWristStart = new SendableChooser<>();
	private SendableChooser<String> teleopShoulderStart = new SendableChooser<>();
	private SendableChooser<Boolean> encoderLimits_chooser = new SendableChooser<>();
	private SendableChooser<Character> autoSide_chooser = new SendableChooser<>();
	private boolean encoderLimitsEnabled = true;
	private Alliance teamColor;
	private Joystick joy1;
	private Joystick joy2;
	
	private char autoSide;
	
	private MecanumDrive drive;
	private Arm arm;
	private Hand hand;
	
	private Timer autoTimer;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser.addDefault("Do Nothing (Auto Program)", "default");
		autoChooser.addObject("Forward Auto", "forward");
		autoChooser.addObject("Strafe Left Auto", "left");
		autoChooser.addObject("Strafe Right Auto", "right");
		autoChooser.addObject("Forward & Place Cube", "color forward");
		autoChooser.addObject("Color side", "color side");
		autoChooser.addObject("Color center", "color center");
		autoChooser.addObject("Scale Only", "scale");
		autoChooser.addObject("Test", "test");
		
		autoSide_chooser.addDefault("Right (Auto Side)", 'R');
		autoSide_chooser.addObject("Left", 'L');
		
		encoderLimits_chooser.addDefault("OFF (Encoder Limits)", false);
		encoderLimits_chooser.addObject("ON", true);
		
		teleopWristStart.addDefault("Off (Teleop Wrist Calibration)", "off");
		teleopWristStart.addObject("Top", "up");
		teleopWristStart.addObject("Bottom", "down");
		teleopShoulderStart.addDefault("Off (Teleop Shoulder Calibration)", "off");
		teleopShoulderStart.addObject("Down", "down");
		
		SmartDashboard.putData("Auto choices", autoChooser);
		SmartDashboard.putData("Encoder Wrist Limits", encoderLimits_chooser);
		SmartDashboard.putData("Teleop Wrist Start", teleopWristStart);
		SmartDashboard.putData("Teleop Shoulder Start", teleopShoulderStart);
		SmartDashboard.putData("Autonomous Side Picker", autoSide_chooser);
		
		autoTimer = new Timer();
		
		/*camera1 = CameraServer.getInstance().startAutomaticCapture(0);
		camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera1.setResolution(320, 240);
		camera2.setResolution(320, 240);*/
		
		joy1 = new Joystick(0);
		joy2 = new Joystick(1);
		drive = new MecanumDrive(53, 56, 50, 51);
		arm = new Arm(57, 52, 55);
		hand = new Hand(58, 54);
		drive.gyro.calibrate();
	}

	
	@Override
	public void autonomousInit() {
		autoTimer.start();
		drive.gyro.reset();
		m_autoSelected = autoChooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		teamColor = DriverStation.getInstance().getAlliance();
		System.out.println("Team color:" + (teamColor==DriverStation.Alliance.Blue?"BLUE":"RED"));
		autoSide = autoSide_chooser.getSelected();
		System.out.println("Auto Side: "+autoSide);
		arm.resetShoulderValues();
		arm.resetWristValuesTop(arm.getWristPosition());
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case "forward":
				if(autoTimer.get() < 4){
					drive.drive(0, 0, -0.5);
				}else{
					drive.drive(0, 0, 0);
				}
				break;
			case "left":
				if(autoTimer.get() < 4){
					drive.drive(Math.PI/2, 0, -0.5);
				}else{
					drive.drive(0, 0, 0);
				}
				break;
			case "right":
				if(autoTimer.get() < 4){
					drive.drive(-Math.PI/2, 0, -0.5);
				}else{
					drive.drive(0, 0, 0);
				}
				break;
			case "color center":
				if(autoTimer.get() < 3.5){
					if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L'){
						drive.drive(-0.6*Math.PI, 0, 0.8);
					}else {
						drive.drive(0.6*Math.PI, 0, 0.8);
					}
				}else if(autoTimer.get() < 4.5){
					arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
					arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					drive.drive(0,0,0);
				}else if(autoTimer.get() < 5.5){
					arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
					arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					drive.drive(0,0,-0.4);
				}else {
					arm.moveShoulder(0);
					arm.moveWrist(0);
					hand.spitAtPower(0.3);
				}
				break;
			case "color forward":
				if(autoTimer.get() < 2){
					arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
					arm.moveWristTowardsTarget(arm.wristSwitchPosition);
				}else if(autoTimer.get() < 6){
					arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
					arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					drive.drive(0,0,-0.5);
				}else if(autoTimer.get() < 10){
					arm.moveShoulder(0);
					arm.moveWrist(0);
					drive.drive(0,0,0);
					if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == autoSide){
						hand.spitAtPower(0.5);
					}
				}else{
					hand.spitAtPower(0);
				}
				break;
			case "color side":
				if(DriverStation.getInstance().getGameSpecificMessage().charAt(1) == autoSide){ //Go for Scale
					if(autoTimer.get() < 3.75){
						drive.driveForwardAtHeading(0, -0.8);
					}else if(autoTimer.get() < 4.25) {
						drive.drive(0,0,0);
					}else if(autoTimer.get() < 5.25){
						if(autoSide == 'R'){
							drive.driveTowardsGyro(-90, 0.6);
						}else {
							drive.driveTowardsGyro(90, 0.6);
						}
					}else if(autoTimer.get() < 7.5){
						drive.drive(0,0,0);
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristUpPosition);
					}else if(autoTimer.get() < 9) {
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					}else {
						arm.moveShoulder(0);
						arm.moveWrist(0);
						hand.spitAtPower(0.6);
					}
				}else if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == autoSide){ // Go for Switch
					if(autoTimer.get() < 1.5){
						drive.driveForwardAtHeading(0, -0.8);
					}else if(autoTimer.get() < 2.5) {
						drive.drive(0,0,0);
					}
					else if(autoTimer.get() < 3.5){
						arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
						if(autoTimer.get() > 4) {
							arm.moveWristTowardsTarget(arm.wristSwitchPosition);
						}
						if(autoSide == 'L'){
							drive.driveTowardsGyro(-90, 0.6);
						}else {
							drive.driveTowardsGyro(90, 0.6);
						}
					}else if(autoTimer.get() < 5){
						arm.moveShoulderTowardsTarget(arm.shoulderSwitchPosition - 400);
						arm.moveWristTowardsTarget(arm.wristSwitchPosition);
						drive.drive(0, 0, -0.5);
					}else if(autoTimer.get() < 6){
						arm.moveShoulder(0);
						arm.moveWrist(0);
						drive.drive(0,0,0);
						hand.spitAtPower(0.4);
					}
				}else{
					if(autoTimer.get() < 4){
						drive.drive(0, 0, -0.5);
					}else{
						drive.drive(0, 0, 0);
					}
					break;
				}
				
				break;
			case "scale":
				if(DriverStation.getInstance().getGameSpecificMessage().charAt(1) == autoSide){ //Same Side Scale
					if(autoTimer.get() < 3.75){
						drive.driveForwardAtHeading(0, -0.8);
					}else if(autoTimer.get() < 4.25) {
						drive.drive(0,0,0);
					}else if(autoTimer.get() < 5.25){
						if(autoSide == 'R'){
							drive.driveTowardsGyro(-90, 0.6);
						}else {
							drive.driveTowardsGyro(90, 0.6);
						}
					}else if(autoTimer.get() < 7.5){
						drive.drive(0,0,0);
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristUpPosition);
					}else if(autoTimer.get() < 9) {
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					}else {
						arm.moveShoulder(0);
						arm.moveWrist(0);
						hand.spitAtPower(0.6);
					}
				}else {  //Far side scale
					if(autoTimer.get() < 2.4) {
						drive.driveForwardAtHeading(0, -1);
					}else if(autoTimer.get() < 3.4) {
						if(autoSide == 'R'){
							drive.driveTowardsGyro(90, 0.6);
						}else {
							drive.driveTowardsGyro(-90, 0.6);
						}
					}else if(autoTimer.get() < 6) {
						if(autoSide == 'R'){
							drive.driveForwardAtHeading(90, -1);
						}else {
							drive.driveForwardAtHeading(-90, -1);
						}
					}else if(autoTimer.get() < 7) {
						if(autoSide == 'R'){
							drive.driveTowardsGyro(150, 0.6);
						}else {
							drive.driveTowardsGyro(-150, 0.6);
						}
					}else if(autoTimer.get() < 7.75) { //Back up
						if(autoSide == 'R'){
							drive.driveForwardAtHeading(150, 0.6);
						}else {
							drive.driveForwardAtHeading(-150, 0.6);
						}
					}else if(autoTimer.get() < 9){
						drive.drive(0,0,0);
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristUpPosition);
					}else if(autoTimer.get() < 11.5) {
						arm.moveShoulderTowardsTarget(arm.shoulderUpPosition+300);
						arm.moveWristTowardsTarget(arm.wristSwitchPosition);
					}else {
						arm.moveShoulder(0);
						arm.moveWrist(0);
						hand.spitAtPower(0.7);
					}
				}
				break;
			case "test":
				drive.driveTowardsGyro(75, 0.6);
				break;
			case "default":
			default:
				// Put default auto code here
				break;
		}
	}
	
	@Override 
	public void teleopInit(){
		String shoulderCalPosition = teleopShoulderStart.getSelected();
		if(shoulderCalPosition == "down"){
			arm.shoulderDownPosition = arm.getShoulderPosition();
			arm.resetShoulderValues();
		}
		String wristCalPosition = teleopWristStart.getSelected();
		if(wristCalPosition == "up"){
			arm.resetWristValuesTop(arm.getWristPosition());
		}else if(wristCalPosition == "down"){
			arm.wristDownPosition = arm.getWristPosition();
			arm.resetWristValues();
		}
		encoderLimitsEnabled = encoderLimits_chooser.getSelected();
		System.out.println("Encoder limits: "+encoderLimitsEnabled);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		double direction = Math.atan2(joy1.getRawAxis(0), joy1.getRawAxis(1));
		double rotation = Math.pow(-joy1.getRawAxis(2), 3);  //power must be  1 + 0.4*strength   (so it has positive and negative)
		double powerLimit = (1-joy1.getRawAxis(3))/2;
		double magnitude = Math.min(1.0, Math.hypot(joy1.getRawAxis(0), joy1.getRawAxis(1)));
		
		drive.drive(direction, rotation*powerLimit, magnitude*powerLimit);
		
		
		//Log encoder values
		if(joy1.getRawButton(12)){
			System.out.println("Shoulder: "+arm.getShoulderPosition());
			System.out.println("Wrist: "+arm.getWristPosition());
		}
		
		//Recalibrate the encoders
		if(joy1.getRawButton(11)){
			arm.shoulderDownPosition = arm.getShoulderPosition();
			arm.resetShoulderValues();
			arm.wristDownPosition = arm.getWristPosition();
			arm.resetWristValues();
		}
		
		//Hand controls
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

		
		//Move shoulder & wrist using encoder positions
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
		}
		//Move shoulder using joy1 buttons
		else if(joy1.getRawButton(3)){
			arm.moveShoulder(-1);
		}else if(joy1.getRawButton(5)){ //Raise Arm
			arm.moveShoulder(1);
		}
		//Move shoulder using joy2 joystick
		else{
			arm.moveShoulder(-joy2.getRawAxis(1));
		}
		
		//Move wrist using joy1 buttons (will override encoder commands)
		if(joy1.getRawButton(4)){
			arm.moveWrist(-1);
		}else if(joy1.getRawButton(6)){ //Raise Wrist
			arm.moveWrist(1);
		}else if(!movingWrist){ //Using else if to avoid overwriting encoder commands
			arm.moveWrist(-joy2.getRawAxis(5));
		}
		
		if(joy1.getRawButtonPressed(10)){
			encoderLimitsEnabled = !encoderLimitsEnabled;
		}
		
		//Auto wrist-raising
		if(encoderLimitsEnabled && !(joy2.getRawButton(2) || joy2.getRawButton(4))){
			if(arm.getShoulderPosition() < arm.shoulderWristFoldBottomLimit && arm.getShoulderPosition() > arm.shoulderWristFoldTopLimit){
				arm.moveWristTowardsTarget(arm.wristUpPosition);
			}
			
			if(arm.getShoulderPosition() > arm.shoulderDownPosition - 100){
				if(arm.getWristPosition() < arm.wristDownPosition){
					arm.moveWristTowardsTarget(arm.wristDownPosition);
				}
			}
		}
		
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
		//Hand controls
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
		
		//Move shoulder using joy1 buttons
		if(joy1.getRawButton(3)){
			arm.moveShoulder(-1);
		}else if(joy1.getRawButton(5)){ //Raise Arm
			arm.moveShoulder(1);
		}
		//Move shoulder using joy2 joystick
		else{
			arm.moveShoulder(-joy2.getRawAxis(1));
		}
		
		//Move wrist using joy1 buttons (will override encoder commands)
		if(joy1.getRawButton(4)){
			arm.moveWrist(-1);
		}else if(joy1.getRawButton(6)){ //Raise Wrist
			arm.moveWrist(1);
		}else { //Using else if to avoid overwriting encoder commands
			arm.moveWrist(-joy2.getRawAxis(5));
		}
	}
}
