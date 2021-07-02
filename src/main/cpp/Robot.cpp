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

  this->m_robotDrive = new frc::DifferentialDrive(*this->m_leftLeadMotor, *this->m_rightLeadMotor);

  this->controller = new frc::XboxController{0}; // replace with USB port number on driver station

  *m_encoderSensor_left_motor = this->m_leftLeadMotor->GetEncoder();
  *m_encoderSensor_right_motor = this->m_rightLeadMotor->GetEncoder();

  l_motor_circum = r_motor_circum = 0;        // replace with circumference of motor in inches
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", left_y);
  frc::SmartDashboard::PutNumber("right y: ", right_y);
  frc::SmartDashboard::PutNumber("left wheel rotations: ", l_motor_rots);
  frc::SmartDashboard::PutNumber("right wheel rotations: ", r_motor_rots);
  frc::SmartDashboard::PutNumber("left wheel distance (ft): ", l_motor_dist);
  frc::SmartDashboard::PutNumber("right wheel distance (ft): ", r_motor_dist);
}

void Robot::AutonomousInit() {
  distance = 5.0;      // feet
}
void Robot::AutonomousPeriodic() {
  /**
   * GetPosition() returns the number of rotations of the motor
   * Num of rotations of motor times circumference of motor = distance
  */
  l_motor_rots = m_encoderSensor_left_motor->GetPosition();
  r_motor_rots = m_encoderSensor_right_motor->GetPosition();

  l_motor_dist = (l_motor_rots * l_motor_circum) / 12; 
  r_motor_dist = (r_motor_rots * r_motor_circum) / 12;

  if (l_motor_dist >= distance || r_motor_dist >= distance) {     
    this->m_robotDrive->TankDrive(0, 0);
  }
  else {
    this->m_robotDrive->TankDrive(max_speed, max_speed);
  }
}

void Robot::TeleopInit() {
  max_speed = 0.5;
  this->m_robotDrive->SetDeadband(0.05);
}
void Robot::TeleopPeriodic() {
  left_y = this->controller->GetY(frc::GenericHID::kLeftHand);
  right_y = this->controller->GetY(frc::GenericHID::kRightHand);

  left_y = std::clamp(left_y, -max_speed, max_speed);
  right_y = std::clamp(right_y, -max_speed, max_speed);
  
  this->m_robotDrive->TankDrive(left_y, right_y, true);
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
