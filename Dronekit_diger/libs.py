def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Taking off!")
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Arming motors")
        mav_lock.acquire()
        vehicle.armed = True
        mav_lock.release()
        time.sleep(0.5)

    # set mode to auto
    while vehicle.mode != 'AUTO':
        print("Setting mode AUTO")
        mav_lock.acquire()
        vehicle.mode = VehicleMode("AUTO")
        mav_lock.release()
        time.sleep(0.5)

    global takeoff_complete
    # wait for the vehicle to finish takeoff:
    takeoff_complete = False
    def wait_for_takeoff(conn, name, m):
        global takeoff_complete
        print("Got message: %s" % (str(m),))
        if m.text.find("Takeoff complete") != -1:
            print("Found it")
            takeoff_complete = True

    vehicle.add_message_listener('STATUSTEXT', wait_for_takeoff)
    print("Waiting for takeoff-complete")
    while not takeoff_complete:
        time.sleep(0.1)

    print("Takeoff is complete")
