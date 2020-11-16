# Vega Ultimate Goal 2020-21
Team 7407 Vega's app for the 2020-21 FTC Game "Ultimate Goal."

## Installation
This code was written in Android Studio, the two ways to import this project are:
1. Download the zip file by clicking "Code" and then "Download Zip" on Github.
2. Clone the following link through Git: https://github.com/ThatKidTiger/Vega2020-2021-ftc_app.git

# Features
*Note:* Due to Covid-19 restrictions, Vega and its sister teams are currently still not in build season, and thus no software or hardware has been able to be tested. All features are currently theoretical.

## Localization
The plan this year is to use dead wheel odometers in a similar vein to [FTC Primitive Data 18219's OpenOdometry](https://openodometry.weebly.com/) to allow us to maintain our position on the field. Research is currently being conducted into RoadRunner and Pure Pursuit, two possible avenues for robot navigation.

## Ring Identification
Vega has previously experimented with [EasyOpenCV](https://github.com/OpenFTC/EasyOpenCV) and color thresholding to detect objects in Rover Ruckus, but our plan this year is to go for a simpler strategy, with a low-mounted camera and various "zones" of different heights, according to the space that 0, 1, and 4 rings occupy in the frame. By averaging the color in each zone, we can see which one is closest to yellow, and therefore which ring configuration exists at the start of the game.

## Launching Rings
With the built-in motor encoders have velocity PID systems, it is simple to use it to control the speed of the flywheel motor to adjust launch distance, according to the recorded position from the odometers. In combination with the IMU rotation measurements, we plan on creating a completely automatic launcher alignment and adjustment sequence.
