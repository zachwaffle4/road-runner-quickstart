# Unofficial Pinpoint Patch Instructions

To use my unofficial Pinpoint fix, do the following: 

## build.gradle

In your TeamCode-level build.gradle, add the following to your `repositories` block:

```groovy
maven {
    url = 'https://maven.zharel.me/snapshots'
}
```

In the `dependencies` block, comment the `rr-ftc` implementation out and add the following instead:

```groovy
implementation "com.acmerobotics.roadrunner:ftc_u:0.1.21-SNAPSHOT-1"
```

## GoBildaPinpointDriver

On lines 64 and 65 of `GoBildaPinpointDriver.java`, 
change the `private` to `public`. 

## PinpointLocalizer

Replace your `PinpointLocalizer` class with 
[this one](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PinpointLocalizer.java).

In line 21, replace `1 / GoBildaPinpointDriver.goBILDA_4_BAR_POD`with your `mmPerTick` value. 
This can be found by dividing the circumference of your odometry wheels **in mm** by the number of ticks per revolution.
Do not confuse this with `inPerTick` in `MecanumDrive`.

## MecanumDrive

On the topic of `inPerTick`, you must set that value 
on line 66 of `MecanumDrive` to `1.0`.
As `PinpointLocalizer` will now give values in inches during both tuning and operation,
changing this value will cause the tuners to fail.

On line 248, make sure that you construct your `PinpointLocalizer` using
`localizer = new PinpointLocalizer(hardwareMap, pose)`; there is no more `inPerTick` argument.

## TuningOpModes

Replace your `TuningOpModes` class with 
[this one](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/TuningOpModes.java)
