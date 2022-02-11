// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <ctre/Phoenix.h>
#include <fmt/core.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "cameraserver/CameraServer.h"
#include <frc/PS4Controller.h>

frc::XboxController controller(1);
frc::Joystick joystick{0};
frc::PS4Controller guitar{2};

frc::VictorSP *motoMotoRight;
frc::VictorSP *motoMotoLeft;
frc::VictorSP *indexer;  // intake
rev::CANSparkMax *shooter;

void Robot::RobotInit() {
    motoMotoRight = new frc::VictorSP(1);
    motoMotoLeft = new frc::VictorSP(0);
    motoMotoRight->SetInverted(true);
    indexer = new frc::VictorSP(2);
    shooter = new rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless);


    m_chooserController = new frc::SendableChooser<std::string>;
    m_chooserController->AddOption("XBox Controller", "xbox");
    m_chooserController->SetDefaultOption("Joystick Controller", "joystick");
    m_chooserController->AddOption("Guitar", "guitar");
    frc::SmartDashboard::PutData(m_chooserController);

     frc::CameraServer::StartAutomaticCapture();
}

void Robot::TeleopInit() { shooter->Set(0); }

void Robot::TeleopPeriodic() {
    // Xbox Controller Handler
    if (m_chooserController->GetSelected() == "xbox") {

        if (controller.GetRightTriggerAxis() > .25) {
            motoMotoRight->Set(controller.GetRightTriggerAxis());
        }

        else if(controller.GetRightTriggerAxis() < .25){
            motoMotoRight->Set(0);
        }
        if (controller.GetLeftTriggerAxis() > .25) {
            motoMotoLeft->Set(controller.GetLeftTriggerAxis());
        }

        else if(controller.GetLeftTriggerAxis() < .25){
            motoMotoLeft->Set(0);
        }
        if(controller.GetRightBumper()) {
            motoMotoRight->Set(-.7);
            motoMotoLeft->Set(.7);
        }
        else if(controller.GetLeftBumper()){
            motoMotoLeft->Set(-.7);
            motoMotoRight->Set(.7);
        }
        if(controller.GetAButton()){
            motoMotoRight->Set(-.25);
            motoMotoLeft->Set(-.25);
        }



        if (abs(controller.GetLeftY()) > .25) {  
                    //  If controller's right trigger is NOT pushed down past
                    //  halfway
            shooter->Set(abs(controller.GetLeftY()));  // Keep controller off
        }
        else {
            shooter->Set(0);  
            // Else (controller's right trigger
            // IS pushed down halfway) change
            // the speed based on how much the
            // trigger is pushed down
        }

        if (controller.GetRightY() > .25) {  // If controller's right
            indexer->Set(0.7);
        }
        else if (controller.GetRightY() < -.25)
            indexer->Set(-0.7);
        else {
            indexer->Set(0);
        }
    }


    /* Robot Controls 
        -Big Joystick -> Move robot
        -Rotate Joystick -> Turn robot
        -Trigger -> Makes indexer spin forward
        -Side Gray Button (RawButton2) -> Makes indexer spin backwards
        -Base Rotate-y Thing -> Adjusts shooter speed
    */ 
   
   // Joystick Controller Handler
    else if (m_chooserController->GetSelected() == "joystick") {
        double driftBuffer = .25;           

       if (joystick.GetTrigger()) {
            indexer->Set(1);                                     
        }

        else if (joystick.GetRawButton(2)) {
            indexer->Set(-1);
        }

        else {
            indexer->Set(0);
        }

        double throttle = -(((joystick.GetThrottle() + 1.0) / 2.0) - 1.0);
        
        if (throttle > 0.15) {
            shooter->Set(throttle);                           // throttle flap handles the speed of the shooter motor (is adjustable based on flap)
        }

        else {
            shooter->Set(0);
        }

        if (joystick.GetTwist() > driftBuffer) {               // turns the robot to the right based on how much the stick is turned 
            motoMotoLeft->Set(joystick.GetTwist());
            motoMotoRight->Set(-joystick.GetTwist());            
        }

        else if (joystick.GetTwist() < -driftBuffer) {         // turns the robot to the left based on how much the stick is turned
            motoMotoLeft->Set(joystick.GetTwist());
            motoMotoRight->Set(-joystick.GetTwist());
        }

        else if (joystick.GetY() > driftBuffer) {              // moves the robot forward based on how far the stick is pushed up
            motoMotoLeft->Set(-joystick.GetY() * .8);
            motoMotoRight->Set(-joystick.GetY() * 1);        
        }

        else if (joystick.GetY() < -driftBuffer) {             // moves the robot backward based on how far the stick is pushed down
            motoMotoLeft->Set(-joystick.GetY() * .8);
            motoMotoRight->Set(-joystick.GetY() * 1);        
        }

        else {
            motoMotoLeft->Set(0);
            motoMotoRight->Set(0);
        }
    }

    else if(m_chooserController->GetSelected() == "guitar") {
        

        
        // green
        if (guitar.GetRawButton(8)) {
            motoMotoLeft->Set(0.7);
            motoMotoRight->Set(0.9);
        }
        
        // red
        else if (guitar.GetRawButton(2)) {
            motoMotoLeft->Set(-0.7);
            motoMotoRight->Set(-0.7);
        }
        
        // yellow
        else if (guitar.GetRawButton(1)) {
            motoMotoLeft->Set(-0.7);
            motoMotoRight->Set(0.7);
        }

        // blue 
        else if (guitar.GetRawButton(3)) {
            motoMotoLeft->Set(0.7);
            motoMotoRight->Set(-0.7);
        }

        else if (guitar.GetRawButton(4)) {
            shooter->Set(0.7);
        }

    if (guitar.GetRawButton(9)) {
            indexer->Set(1);
        }

        else{
            motoMotoLeft->Set(0);
            motoMotoRight->Set(0);
            shooter->Set(0);
        }
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::RobotPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
