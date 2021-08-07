// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_leftLeadMotor = new rev::CANSparkMax(leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_rightLeadMotor = new rev::CANSparkMax(rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_leftFollowMotor = new rev::CANSparkMax(leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);
  m_rightFollowMotor = new rev::CANSparkMax(rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless);

  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);

  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

  controller = new frc::XboxController{0}; // replace with USB port number on driver station
}
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(controller->GetY(left_analog)));
  frc::SmartDashboard::PutNumber("right x: ", controller->GetX(right_analog));
  frc::SmartDashboard::PutNumber("left wheel distance: ", m_leftLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet);
  frc::SmartDashboard::PutNumber("right wheel distance: ", m_rightLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet);
}

void Robot::AutonomousInit() {
  kP = 0.0;
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
  setpoint = lDistance = rDistance = lSpeed = rSpeed = lError = rError = 0.0;
}
void Robot::AutonomousPeriodic() {
  if (controller->GetAButton()) {
    kP = 0.1;                 // needs tuning
    setpoint = 5.0;
    wpi::outs() << "autonomous initiated" << "\n";
  }
  else if (controller->GetBButton()) {
    kP = 0.0;
    wpi::outs() << "autonomous disabled" << "\n";
  }

  lDistance = m_leftLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet;
  rDistance = m_rightLeadMotor->GetEncoder().GetPosition() * kDriveRots2Feet;

  lError = setpoint - lDistance;
  rError = setpoint - rDistance;

  lSpeed = kP * lError;
  rSpeed = kP * rError;
  
  m_leftLeadMotor->Set(lSpeed);
  m_rightLeadMotor->Set(rSpeed);
}

void Robot::TeleopInit() {
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
}
void Robot::TeleopPeriodic() {
  left_y = controller->GetY(left_analog);
  right_x = controller->GetX(right_analog);

  if (abs(left_y) < 0.08) 
    left_y = 0;

  m_leftLeadMotor->Set(-left_y);
  m_rightLeadMotor->Set(-left_y);
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
