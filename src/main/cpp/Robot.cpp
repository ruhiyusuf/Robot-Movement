// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {
  this->m_leftLeadMotor = new rev::CANSparkMax(this->leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightLeadMotor = new rev::CANSparkMax(this->rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_leftFollowMotor = new rev::CANSparkMax(this->leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  this->m_rightFollowMotor = new rev::CANSparkMax(this->rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  this->m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  this->m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  frc::Joystick trigger{0}; // replace with USB port number on driver station

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

  if (trigger.GetTriggerPressed()) {
    m_leftLeadMotor->Set(0.5);
    m_rightLeadMotor->Set(0.5);
  }
  else {
    m_leftLeadMotor->Set(0);
    m_rightLeadMotor->Set(0);
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
