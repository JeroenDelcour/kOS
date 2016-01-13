set targetOrbit to 100000.

clearscreen.
SAS off.
RCS off.
gear off.

lock steering to heading(90,90).
lock throttle to 1.
wait 0.0001.
print "Launching!".
stage.

when stage:liquidfuel < 0.1 then {
	stage.
	if ship:liquidfuel > 1 {
		preserve.
	}
}

until ship:apoapsis > targetOrbit {
	if ship:altitude > 1000 {
		set targetPitch to max(1, (90 * (1 - SHIP:ALTITUDE / 20000))).
		lock steering to heading(90, targetPitch).
	}
}

print "Coasting to apoapsis and performing circularization burn.".

lock throttle to 0.
lock steering to heading(90,0).
until ship:periapsis > 70000 and eta:apoapsis > 60*6 {
	if ETA:apoapsis < 10 {
		lock throttle to 1.
	} else if ETA:apoapsis < 30 {
		set warp to 0.
	} else if ship:altitude > 70000 {
		set warp to 3.
	}
}

lock throttle to 0.

print "Done, ship should now be in orbit!".

//This sets the user's throttle setting to zero to prevent the throttle
//from returning to the position it was at before the script was run.
set ship:control:pilotmainthrottle to 0.