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

  this->m_leftLeadMotor->RestoreFactoryDefaults();
  this->m_rightLeadMotor->RestoreFactoryDefaults();
  this->m_leftFollowMotor->RestoreFactoryDefaults();
  this->m_rightFollowMotor->RestoreFactoryDefaults();

  this->m_left = new frc::SpeedControllerGroup(*this->m_leftLeadMotor, *this->m_leftFollowMotor);
  this->m_right = new frc::SpeedControllerGroup(*this->m_rightLeadMotor, *this->m_rightFollowMotor);

  this->m_robotDrive = new frc::DifferentialDrive(*this->m_left, *this->m_right);

  this->controller = new frc::XboxController{0}; // replace with USB port number on driver station

  *m_encoderSensor_left_motor = this->m_leftLeadMotor->GetEncoder();
  *m_encoderSensor_right_motor = this->m_rightLeadMotor->GetEncoder();        
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", left_y);
  frc::SmartDashboard::PutNumber("right y: ", right_x);
  frc::SmartDashboard::PutNumber("left motor rotations: ", l_motor_rots);
  frc::SmartDashboard::PutNumber("right motor rotations: ", r_motor_rots);
  frc::SmartDashboard::PutNumber("left motor distance (ft): ", l_motor_dist);
  frc::SmartDashboard::PutNumber("right motor distance (ft): ", r_motor_dist);
}

void Robot::AutonomousInit() {
  max_speed = 0.5;
  distance = 5.0;      // feet
}
void Robot::AutonomousPeriodic() {
  /**
   * To calculate linear distance of the robot:
   * calculate the gear ratio: num of teeth on driven gear / num of teeth on driver gear
   * calculate gear to wheel proportion: gear ratio x circumference of wheel
   * calculate num of rotations of wheel: gear to wheel proportion x num of rotations outputted by encoder
   * calculate linear distance: num of rotations of wheel x circumference of wheel
  */

  // ignore the following code for autonomous periodic

  l_motor_rots = m_encoderSensor_left_motor->GetPosition();
  r_motor_rots = m_encoderSensor_right_motor->GetPosition();

  l_motor_dist = (l_motor_rots * l_motor_circum) / 12; 
  r_motor_dist = (r_motor_rots * r_motor_circum) / 12;

  if (l_motor_dist >= distance || r_motor_dist >= distance) {     
    this->m_robotDrive->TankDrive(0.0, 0.0);
  }
  else {
    this->m_robotDrive->TankDrive(max_speed, max_speed);
  }
}

void Robot::TeleopInit() {
  this->m_robotDrive->SetDeadband(0.05);
}
void Robot::TeleopPeriodic() {
  left_y = this->controller->GetY(left_analog);
  right_x = this->controller->GetX(right_analog);
  
  this->m_robotDrive->ArcadeDrive(left_y, right_x, true);
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
