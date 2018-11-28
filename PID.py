def calcOBSPID(sp, inp, oE, gains, dt):
    ### Set the Kp, Ki, Kd values ###
    Kp = gains[0]
    Ki = gains[1]
    Kd = gains[2]

    ### Calculate the error ###
    err = sp - inp

    ### Calculate the P, I, and D values ###
    P = Kp * err
    I = Ki * (err + oE) * dt
    D = Kd * (err - oE) / dt

    ### Sum P, I, and D values ###
    PID = P + I + D

    ### Return PID ###
    return PID


def calcLinePID(sp, inp, oE, gains, dt):
    ### Set the Kp, Ki, Kd values ###
    Kp = gains[0]
    Ki = gains[1]
    Kd = gains[2]

    ### Calculate the error ###
    err = sp - inp

    ### Calculate the P, I, and D values ###
    P = Kp * err
    I = Ki * (err + oE) * dt
    D = Kd * (err - oE) / dt

    ### Sum P, I, and D values ###
    PID = P + I + D

    ### Return PID ###
    return PID

