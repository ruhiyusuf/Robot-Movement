// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  this->m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  this->m_leftFollowMotor->Follow(*this->m_leftLeadMotor, false);
  this->m_rightFollowMotor->Follow(*this->m_rightLeadMotor, false);

  this->controller = new frc::XboxController{0}; // replace with USB port number on driver station

  *m_encoderSensor_left_motor = this->m_leftLeadMotor->GetEncoder();
  *m_encoderSensor_right_motor = this->m_rightLeadMotor->GetEncoder();

  l_wheel_circum = 0, r_wheel_circum = 0;        // replace with circumference of wheels
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", left_y);
  frc::SmartDashboard::PutNumber("left wheel rotations: ", l_wheel_rotations);
  frc::SmartDashboard::PutNumber("right wheel rotations: ", r_wheel_rotations);
  frc::SmartDashboard::PutNumber("left wheel distance: ", l_wheel_dist);
  frc::SmartDashboard::PutNumber("right wheel distance: ", r_wheel_dist);
}

void Robot::AutonomousInit() {
  distance = 5.0;               // feet
  double factor = 0.0;          // replace with conversion factor from circumference of motor to circumference of wheel - for ex. 2 rots of motor = 1 rot of wheel, factor = 1/2
  m_encoderSensor_left_motor->SetPositionConversionFactor(factor);  
  m_encoderSensor_right_motor->SetPositionConversionFactor(factor);
}
void Robot::AutonomousPeriodic() {
  l_wheel_rotations = m_encoderSensor_left_motor->GetPosition();
  r_wheel_rotations = m_encoderSensor_right_motor->GetPosition();

  l_wheel_dist = l_wheel_rotations * l_wheel_circum; 
  r_wheel_dist = r_wheel_rotations * r_wheel_circum;

  if (l_wheel_dist >= (distance*12) || r_wheel_dist >= (distance*12)) {     
    this->m_leftLeadMotor->Set(0);
    this->m_rightLeadMotor->Set(0);
  }
  else {
    this->m_leftLeadMotor->Set(0.5);
    this->m_rightLeadMotor->Set(0.5);
  }
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  left_y = this->controller->GetY(frc::GenericHID::kLeftHand);

  if (left_y < 0.0) {                 // condition checks for value less than 0.0, since moving the joystick up returns a value approaching -1
    this->m_leftLeadMotor->Set(0.5);
    this->m_rightLeadMotor->Set(0.5);
  }
  else {
    this->m_leftLeadMotor->Set(0);
    this->m_rightLeadMotor->Set(0);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
