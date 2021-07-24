// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

void Robot::RobotInit() {
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftLeadMotor->SetInverted(false);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(true);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  m_leftLeadMotor->GetEncoder();
  m_rightLeadMotor->GetEncoder();        
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", left_y);
  frc::SmartDashboard::PutNumber("right x: ", right_x);
  frc::SmartDashboard::PutNumber("left wheel rotations: ", l_wheel_rots);
  frc::SmartDashboard::PutNumber("right wheel rotations: ", r_wheel_rots);
  frc::SmartDashboard::PutNumber("left wheel distance (ft): ", l_wheel_dist);
  frc::SmartDashboard::PutNumber("right wheel distance (ft): ", r_wheel_dist);
}

void Robot::AutonomousInit() {
  max_speed = 0.5;
  setpoint = 5.0;     // feet
  double wheel2GearR = (5.7 * M_PI) / (40 / 14); // 5.7 inch diameter wheel, 28 teeth on driver gear, 40 teeth on driven gear
  m_leftLeadMotor->GetEncoder().SetPositionConversionFactor(wheel2GearR);
  m_rightLeadMotor->GetEncoder().SetPositionConversionFactor(wheel2GearR);
}
void Robot::AutonomousPeriodic() {
  l_wheel_rots = m_leftLeadMotor->GetEncoder().GetPosition();
  r_wheel_rots = m_rightLeadMotor->GetEncoder().GetPosition();

  l_wheel_dist = (l_wheel_rots * l_wheel_circum) / 12; 
  r_wheel_dist = (r_wheel_rots * r_wheel_circum) / 12;

  if (l_wheel_dist >= setpoint || r_wheel_dist >= setpoint) {     
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
