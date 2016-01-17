// kudos to Daniel Rings, his code was a major help:
// https://github.com/DanielRings/ReusableLaunchSystem/tree/master/kOS%20Script

clearscreen.

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

/////////////////////////////////

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

/////////////////////////////////

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
set Kp to 10000.
set Ki to 0.
set Kd to 50.
set integral to 0.
set previousMeasured to measured.
set outMin to -20.
set outMax to 20.
set previousError to 0.
set vel2pitchPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

/////////////////////////////////

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
set Kp to 10000.
set Ki to 0.
set Kd to 50.
set integral to 0.
set previousMeasured to measured.
set outMin to -20.
set outMax to 20.
set previousError to 0.
set vel2yawPID to list(desired, measured, Kp, Ki, Kd, integral, previousMeasured, outMin, outMax, 0, previousError).

/////////////////////////////////

when true then {
    set monitoredPID to vel2thrPID.
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

//////////////////////////////////////////////////////////////////////////////////////////////

lock altitude to alt:radar.
set padlatitude to -0.0971712693572044.
set padlongitude to -74.5576858520508.
set dt to 0.1.
sas off.
rcs on.

// end of setup
// start suicide burn

set safetyMargin to 100.

set steer to UP + r(0,0,180).
lock steering to steer.
lock throttle to 0.
set previousLat to latitude.
set previousLong to longitude.
lock latVel to (latitude - previousLat)/dt.
lock longVel to (longitude - previousLong)/dt.

UNTIL FALSE {
    // suicide burn
    if altitude > 200 {

        // point retrograde
        set steer to (-1) * SHIP:VELOCITY:SURFACE.

        // calculate when to start the burn
        if verticalspeed < 0 {

        }
        set gravity to body:mu / (ship:altitude + body:radius)^2.
        set verticalDeltaV to sqrt(2 * gravity * (altitude + safetyMargin) + verticalspeed^2).
        set horizontalDeltaV to groundspeed.
        set deltaV to max(0, verticalDeltaV + horizontalDeltaV).
        set burnAltitude to ((mass * 1000) * deltaV^2) / (2 * 1000 * maxthrust).
        print "BURN ALTITUDE: "+burnAltitude at (5,16).

        if altitude < burnAltitude and verticalspeed < 0 {
            print "PERFORMING SUICIDE BURN           " at (5,17).
            lock throttle to 1.
        } else {
            print "                                  " at (5,17).
            lock throttle to 0.
        }

    } else {
        gear on.
        // cancel out horizontal velocity
        set vel2yawPID to PID(vel2yawPID, 0, longVel).
        set vel2pitchPID to PID(vel2pitchPID, 0, latVel).
        set steer to UP + r(-vel2pitchPID[9], -vel2yawPID[9], 180).

        // hover until horizontal velocity is almost gone, then decend slowly
        if groundspeed > 1 {
            print "CANCELLING OUT HORIZONTAL VELOCITY" at (5,17).
            set alt2velPID to PID(alt2velPID, 50, altitude).
            set targetVerticalSpeed to alt2velPID[9].
        } else {
            print "LANDING                           " at (5,17).
            set targetVerticalSpeed to min(-1, max(-altitude/3, -10)).
            //set targetVerticalSpeed to -1.
        }
        set vel2thrPID to PID(vel2thrPID, targetVerticalSpeed, verticalSpeed).
        lock throttle to vel2thrPID[9].
    }

    if status = "LANDED" {
        clearscreen.
        print "Touchdown!".
        lock throttle to 0.
        sas on.
        break.
    }

    set previousLat to latitude.
    set previousLong to longitude.

    wait dt.
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.