# Quadruple Servo Tester

![License](https://img.shields.io/badge/license-MIT-blue.svg)

## Project Overview

This project is a quadruple servo tester that allows you to control up to four servos using potentiometers connected to the KL25Z microcontroller board. Additionally, you can monitor each servo's PWM parameters on an optional LCD display. The design can be adapted for any 4 Degrees of Freedom (DOF) robots by replacing the potentiometers with other mechanical or digital control methods.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Wiring Setup](#wiring-setup)
- [Usage](#usage)
  - [Controlling Servos](#controlling-servos)
  - [Viewing PWM Parameters](#viewing-pwm-parameters)
  - [Adapting for 4 DOF Robots](#adapting-for-4-dof-robots)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction

The Quadruple Servo Tester is designed to control four servos using potentiometers. Each servo is connected to the terminals of the ADCs (Analog-to-Digital Converters) of the KL25Z board. The project also supports an optional LCD display to show each servo's PWM parameters, making it easy to monitor and adjust servo positions. This setup can be modified for use in any 4 DOF robotic system by replacing the potentiometers with other mechanical or digital control means.

## Features

- Control up to four servos using potentiometers.
- Display PWM parameters on an optional LCD display.
- Real-time adjustment of servo positions.
- Adaptable for 4 DOF robotic systems.
- User-friendly interface with clear readouts on the LCD.

## Hardware Requirements

- [KL25Z Microcontroller Board](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/kinetis-cortex-m-mcus/l-seriesultra-low-power-m0-plus/kinetis-l-series-lower-power-32-bit-mcu-based-on-arm-cortex-m0-plus-core:KL25Z)
- Four potentiometers (or other control mechanisms)
- Four servos
- Optional: LCD display (e.g., 16x2 character LCD)

## Getting Started

### Prerequisites

- [MBED Development Environment](https://os.mbed.com/) (optional)
- Understanding of MKL25Z CMSIS abbreviations
- Basic knowledge of C/C++ programming and microcontroller interfacing.

### Installation

1. **Clone this repository**:
   ```bash
   git clone https://github.com/yourusername/quadruple-servo-tester.git
