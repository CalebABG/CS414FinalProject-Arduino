# CS414FinalProject-Arduino
![](images/CS414FinalProjectBot.png?raw=true "RC Vehicle / Bot with LiPo batteries in front lower part of chassis, two DC voltage regulators in rear upper part of chassis with motor driver on right side, and Arduino Mega in rear lower part of chassis")

## Synopsis

This is the Arduino driver code for my Android class final project. This code handles setting up and driving the motors of the bot, as well as handling the communication between the phone and Arduino through Bluetooth.

This is most of the meat and potatoes of the project, the companion phone app can be found here: [CS414FinalProject-Android](https://github.com/CalebABG/CS414FinalProject-Android)

If you'd like to take a look at an in detail breakdown of the project and the challenges and solutions, checkout the project report: [Final Project Report](writeups/project-breakdown.md)

If you'd like to see a side-by-side demo of the app and bot together, checkout the demo on Youtube: [Final Project Demo]()

Most if not all project parts are listed below, so check them out if you're interested and want to take a go at building one!

Thanks for checking out the repo, happy coding!

---

## Parts / Supplies

A list of the parts needed to build this project can be found below. Please, please, please,
take the time to search and watch how to **PROPERLY** and **SAFELY** handle, charge and discharge `LiPo` / `Lithium Ion` batteries.
- [HowTo - LiPo Battery Charge / Care](https://www.youtube.com/watch?v=sGsJmSBKxrc)

Also, you will need some sort of chassis or support platform to mount the wheels and the other parts. Below is a link to a potential chassis, and included is a number of the parts listed below, but the parts below require no - minimal soldering, so more plug and play.

- [2WD Smart Robot Car Chassis Kit](https://www.amazon.com/YIKESHU-Smart-Chassis-Encoder-Battery/dp/B073VHQT6P)

Would also recommend using a `hot glue` gun for mounting the motors! If not hot glue then `command strips`, but you will probably end up needing to reinforce them quite often for the wheels to not come loose or out of alignment.
- [Hot Glue Gun Kit](https://www.amazon.com/Gorilla-8401509-Hot-Glue-Sticks/dp/B07K791YRP)
- [Command Strips](https://www.amazon.com/Command-Picture-Decorate-Damage-Free-PH206-14NA/dp/B073XR4X72)

### Parts

- [Arduino MEGA R3](https://www.amazon.com/ELEGOO-ATmega2560-ATMEGA16U2-Projects-Compliant/dp/B01H4ZDYCE)
- [2 x 11.1V 3S 850mAh 75C LiPo Batteries](https://www.amazon.com/dp/B07218SB7L)
  - You can get a larger capacity battery if needed (i.e > 850mAh), but make sure to then also buy a compatible `charger` and `connector` plugs
  - You can also get a higher voltage battery if needed (i.e > 11.1V), but make sure to step down the voltage for the `Arduino` to between 7-12V; as well as for the `Motor Driver` between 5-40V
    - Note: If using TT motors, driving motors with more than 25-30V, you will probably burn out your motors. Check your motors datasheet to see what the max input voltage the motors can take is.
- [LiPo Battery Balance Charger](https://www.amazon.com/dp/B099K8XFG6)
  - If you can spare the extra expense, go with this charger: [ISDT Q6 Lite Battgo Lipo Battery Charger/Discharger](https://www.amazon.com/ISDT-Battery-Charger-Discharger-Balance/dp/B078RF1SD5)
- [XT30 Plug Connectors](https://www.amazon.com/dp/B07T894CD6)
- [DC-DC Voltage Regulators](https://www.amazon.com/dp/B081N6WWJS)
- [L298N Motor Drivers](https://www.amazon.com/HiLetgo-Controller-Stepper-H-Bridge-Mega2560/dp/B07BK1QL5T)
- [TT DC Gearbox Motors](https://www.amazon.com/Gearbox-Motor-Wheel-Arduino-Smart/dp/B07P6QCJPX)
- [HC-06 Bluetooth Module](https://www.amazon.com/dp/B087R42L2J)
- [Breadboard Jumper Wires](https://www.amazon.com/dp/B08151TQHG)

### Optional Parts but **HIGHLY** recommended

- [Lipo Battery Safe Bag](https://www.amazon.com/dp/B01GCHBQJS)
- [Lipo Battery Voltage Checker](https://www.amazon.com/dp/B00XQ91ECA)
  - Be **PREPARED** these things are loud when the alarm goes off
