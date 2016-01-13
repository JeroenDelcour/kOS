// lots of kudos to Daniel Rings, his code was a major help:
// https://github.com/DanielRings/ReusableLaunchSystem/tree/master/kOS%20Script

// SETUP

clearscreen.

// set CONFIG:IPU to 300.

function PID {
    parameter x.
    parameter desired.
    parameter measured.

    set x[0] to desired.
    set x[1] to measured.
    set Kp to x[2].
    set Ki to x[3].
    set Kd to x[4].
    set integral to x[5].
    set previousMeasured to x[6].
    set outMin to x[7].
    set outMax to x[8].
    set previousError to x[10].

    set error to desired - measured.
//    if not x[9] > outMax {
    set integral to integral + (error * dt).
//    }
    if integral * Ki > outMax {
        set integral to outMax / Ki.
    }
    If (integral * Ki) < outMin {
        Set integral to outMin / Ki.
    }.

    set derivative to (error - previousError) / dt.

    set out to Kp * error + Ki * integral + Kd * derivative.

    If (out > outMax){
        Set out to outMax.
    }.
    If (out < outMin){
        Set out to outMin.
    }.

    set previousMeasured to measured.
    set previousError to error.

    set x[5] to integral.
    set x[6] to error.
    set x[10] to previousError.
    set x[9] to out.

    return x.
}

// altitude to velocity PID
Set desired to 0.
Set measured to verticalspeed.
Set Kp to 0.2.
Set Ki to 0.
Set Kd to 0.03.
set integral to 0.
Set previousMeasured to measured.
set outMin to -200.
set outMax to 200.
set previousError to 0.
set alt2velPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

// velocity to throttle PID
Set desired to 0.
Set measured to verticalspeed.
Set Kp to 0.100.
Set Ki to 0.200.
Set Kd to 0.005.
set integral to 0.
Set previousMeasured to measured.
set outMin to 0.
set outMax to 1.
set previousError to 0.
set vel2thrPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

// latitude to velocity PID
set desired to latitude.
set measured to latitude.
set Kp to 1.
set Ki to 0.
set Kd to 10.
set integral to 0.
set previousMeasured to measured.
set outMin to -0.04.
set outMax to 0.04.
set previousError to 0.
set lat2velPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

// lateral velocity to pitch PID
set desired to 0.
set measured to 0.
set Kp to 500.
set Ki to 0.
set Kd to 100.
set integral to 0.
set previousMeasured to measured.
set outMin to -10.
set outMax to 10.
set previousError to 0.
set vel2pitchPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

// longitude to velocity PID
set desired to longitude.
set measured to longitude.
set Kp to 1.
set Ki to 0.
set Kd to 10.
set integral to 0.
set previousMeasured to measured.
set outMin to -0.04.
set outMax to 0.04.
set previousError to 0.
set long2velPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

// longitudinal velocity to yaw PID
set desired to 0.
set measured to 0.
set Kp to 500.
set Ki to 0.
set Kd to 100.
set integral to 0.
set previousMeasured to measured.
set outMin to -10.
set outMax to 10.
set previousError to 0.
set vel2yawPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

when true then {
    set monitoredPID to alt2velPID.
    print "ERROR:      " + (monitoredPID[0] - monitoredPID[1]) + "" at (5,4).
    print "DESIRED:    " + monitoredPID[0] + "      " at (5,5).
    print "MEASURED:   " + monitoredPID[1] + "      " at (5,6).
    print "Kp:         " + monitoredPID[2] + "      " at (5,7).
    print "Ki:         " + monitoredPID[3] + "      " at (5,8).
    print "Kd:         " + monitoredPID[4] + "      " at (5,9).
    print "INTEGRAL:   " + monitoredPID[5] + "      " at (5,10).
    print "PREV ERROR: " + monitoredPID[6] + "      " at (5,11).
    print "OUT MIN:    " + monitoredPID[7] + "      " at (5,12).
    print "OUT MAX:    " + monitoredPID[8] + "      " at (5,13).
    print "OUT:        " + monitoredPID[9] + "      " at (5,14).
    preserve.
}

set padheight to alt:radar.
lock altitude to alt:radar - padheight.
set padlatitude to latitude.
set padlongitude to longitude.
set dt to 0.1.
set targetAltitude to 0.
sas off.
rcs on.

// END SETUP

set steer to UP + r(0,0,180).
lock steering to steer.
lock throttle to 0.
Stage.
Print "Blastoff!".

// hover for a while.
set timer to missiontime.
set previousLat to latitude.
set previousLong to longitude.
until missiontime - timer > 30 {
    set alt2velPID to PID(alt2velPID, 3000, altitude).
    set vel2thrPID to PID(vel2thrPID, alt2velPID[9], verticalSpeed).
    lock throttle to vel2thrPID[9].

    set velLat to (latitude - previousLat)/dt.
    set previousLat to latitude.
    set lat2velPID to PID(lat2velPID, padlatitude, latitude).
    set vel2pitchPID to PID(vel2pitchPID, -lat2velPID[9], velLat).

    set velLong to (longitude - previousLong)/dt.
    set previousLong to longitude.
    set long2velPID to PID(long2velPID, padlongitude, longitude).
    set vel2yawPID to PID(vel2yawPID, -long2velPID[9], velLong).

    set steer to UP + r(vel2pitchPID[9], vel2yawPID[9], 180).

    wait dt.
}

// now land with a suicide burn.
set deltaV to sqrt(2 * 9.81 * alt:radar + verticalspeed^2).
set burnAltitude to ((mass * 1000) * deltaV^2) / (2 * 1000 * maxthrust).
print burnAltitude.
lock throttle to 0.

when altitude < burnAltitude then {
    lock throttle to 1.

    when verticalspeed > 0 then {
        lock throttle to vel2thrPID[9].

        when altitude < 0.5 then {
            lock throttle to 0.
            set runmode to 0.
        }
    }
}

lock landing_speed to min(-altitude/3, -1).
until altitude < 0.2 {
    set velLat to (latitude - previousLat)/dt.
    set previousLat to latitude.
    set lat2velPID to PID(lat2velPID, padlatitude, latitude).
    set vel2pitchPID to PID(vel2pitchPID, -lat2velPID[9], velLat).
    set velLong to (longitude - previousLong)/dt.
    set previousLong to longitude.
    set long2velPID to PID(long2velPID, padlongitude, longitude).
    set vel2yawPID to PID(vel2yawPID, -long2velPID[9], velLong).
    set steer to UP + r(vel2pitchPID[9], vel2yawPID[9], 180).

    set deltaV to sqrt(2 * 9.81 * altitude + verticalspeed^2).
    set burnAltitude to ((mass * 1000) * deltaV^2) / (2 * 1000 * maxthrust).
    if altitude > 50 {
        if altitude < burnAltitude {
            lock throttle to 1.
            print "SUICIDE BURNING" at (5,17).
        } else {
            lock throttle to 0.
        }
    } else {
        set vel2thrPID to PID(vel2thrPID, landing_speed, verticalSpeed).
        lock throttle to vel2thrPID[9].
    }

    print "BURN ALTITUDE:        " + burnAltitude + "      " at (5,16).

    wait dt.
}

lock throttle to 0.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.