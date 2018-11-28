def calcOBSPID(sp, inp, oE, gains, dt):
    Kp = gains[0]
    Ki = gains[1]
    Kd = gains[2]

    err = sp - inp

    P = Kp * err
    I = Ki * (err + oE) * dt
    D = Kd * (err - oE) / dt

    PID = P + I + D

    return PID


def calcLinePID(sp, inp, oE, gains, dt):
    Kp = gains[0]
    Ki = gains[1]
    Kd = gains[2]

    err = sp - inp

    P = Kp * err
    I = Ki * (err + oE) * dt
    D = Kd * (err - oE) / dt

    PID = P + I + D

    return PID

