import RPi.GPIO as GPIO
import time
import numpy as np

servo1 = 38
servo2 = 40
GPIO_TRIGGER = 16
GPIO_ECHO = 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
ser1 = GPIO.PWM(servo1, 50)
ser2 = GPIO.PWM(servo2, 50)
ser1.start(7.5)
ser2.start(7.5)

#computational parameter
gamma = 0.75
alpha = 0.1

#servo1 motor parameter
num_theta1_states = 6
theta1_initial_angle = 70.0
theta1_max = 90.0
theta1_min = 70.0
delta_theta1 = (theta1_max - theta1_min)/(float(num_theta1_states)-1.0)
s1 = int((theta1_initial_angle - theta1_min)/delta_theta1)


#servo2 motor parameter
num_theta2_states = 6
theta2_initial_angle = 90.0
theta2_max = 160.0
theta2_min = 90.0
delta_theta2 = (theta2_max - theta2_min)/(float(num_theta2_states)-1.0)
s2 = int((theta2_initial_angle - theta2_min)/delta_theta2)


#initialize Q to zeros
num_states = num_theta1_states * num_theta2_states     # 36 states
num_actions = 4                                        # 4 actions
Q = np.matrix(np.zeros([num_states, num_actions]))
#print(Q)

#initialize the state number
#row indexing of the Q matrix (s1 and s2 are are the states of the servo1 and servo2 respectively)
#row indexing starts from the 0 index
s = int(s1*num_theta2_states + s2)
sprime = s
#print(s1, s2, s)

#this function will setup the servos
def setup():
    angle = 90.0
    print("setup ran once")
    d = duty(angle)

    ser1.ChangeDutyCycle(d)
    time.sleep(1)
    ser2.ChangeDutyCycle(d)
    time.sleep(1)

#This function will get the posssible actions
def get_action(EPSILON, s1, s2):
    valmax = -10000000.0
    allowed_actions = np.array([-1, -1, -1, -1])

    if((s1 + 1) != num_theta1_states):
        allowed_actions[0] = 1
        val = Q[s,0]
        if (val > valmax):
            valmax = val
            index = 0

    if(s1 != 0):
        allowed_actions[1] = 1
        val = Q[s, 1]
        if(val > valmax):
            valmax = val
            index = 1

    if((s2 + 1) != num_theta2_states):
        allowed_actions[2] = 1
        val = Q[s, 2]
        if(val > valmax):
            valmax = val
            index = 2

    if(s2 != 0):
        allowed_actions[3] = 1
        val = Q[s, 3]
        if(val > valmax):
            valmax = val
            index = 3

    state_actions = np.where(allowed_actions >= 0)[0]
    if (np.random.uniform() < (1 - EPSILON)):
        action = index

    else:
        action = np.random.choice(state_actions)

    return action
#a = get_action()
#print(a)

#for a given action and state(s) find the next state.
# and also keep the track of s1 and s2 joint states
def setsprime (action, s1, s2):
    #print(action, s1, s2)
    if action == 0:                      # joint1++
        sprime = s + num_theta2_states
        s1 = s1 + 1
    elif action == 1:
        sprime = s - num_theta2_states
        s1 = s1 - 1
    elif action == 2:
        sprime = s+1
        s2 = s2 + 1
    else:
        sprime = s - 1
        s2 = s2 - 1
    #print(sprime)
    #print(sprime, s1, s2)
    return sprime, s1, s2
#b = setsprime(a, s1, s2)

#This function will calculate the duty cycle value for a given angle
def duty(final_angle):
    dty = (1.0/20.0 * (final_angle)) + 3
    return dty

#update the position of the servo motors ser1 and ser2
#This is the physical state transition command
def set_physical_state(action, ser1_angle, ser2_angle):
    
    if (action == 0):
        current_angle = ser1_angle
        final_angle = current_angle + delta_theta1
        d = duty(final_angle)
        ser1.ChangeDutyCycle(d)
        time.sleep(0.2)
        ser1_angle = final_angle

    elif (action == 1):
        current_angle = ser1_angle
        final_angle = current_angle - delta_theta1
        d = duty(final_angle)
        ser1.ChangeDutyCycle(d)
        time.sleep(0.2)
        ser1_angle = final_angle

    elif (action == 2):
        current_angle = ser2_angle
        final_angle = current_angle + delta_theta2
        d = duty(final_angle)
        ser2.ChangeDutyCycle(d)
        time.sleep(0.2)
        ser2_angle = final_angle

    else:
        current_angle = ser2_angle
        final_angle = current_angle - delta_theta2
        d = duty(final_angle)
        ser2.ChangeDutyCycle(d)
        time.sleep(0.2)
        ser2_angle = final_angle

    return ser1_angle, ser2_angle

#This function will get the distance moved by the robot
def get_delta_distance_rolled(previous_distance):
    
    ser1.stop()
    ser2.stop()
    time.sleep(0.2)
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300 ) / 2
    distance = int(distance)
    distance = distance * 10
    
    
    
    delta_distance = distance - previous_distance
    if (abs(delta_distance) < 20 or abs(delta_distance) > 600):
        delta_distance = 0
    
    #print(delta_distance)
    return delta_distance, distance

#This function will get max a' of Q(s', a')
def get_look_ahead(sprime):
    valmax = -100000.0
    if((s1 + 1) != num_theta1_states):
        val = Q[sprime, 0]
        if(val > valmax):
            valmax = val

    if(s1 != 0):
        val = Q[sprime, 1]
        if(val > valmax):
            valmax = val

    if((s2 + 1) != num_theta2_states):
        val = Q[sprime, 2]
        if(val > valmax):
            valmax = val

    if(s2 != 0):
        val = Q[sprime, 3]
        if(val > valmax):
            valmax = val

    return valmax

#All the Q matrix value will be initialized to value 10
def initializeQ():
    Q = np.matrix(np.full(([num_states, num_actions]), 10.0))
    return Q
    #print(Q)
#c = initializeQ()

#this function get servos to 90 in the end and cleanup thepins
def get_90degree():
    d = duty(90.0)
    ser1.ChangeDutyCycle(d)
    time.sleep(0.2)
    d = duty(90.0)
    ser2.ChangeDutyCycle(d)
    time.sleep(0.2)
    ser1.stop()
    ser2.stop()
    GPIO.cleanup()
    

#This is the main brain of the system (Q learing algorithm)
def rl_brain():
    s = int(s1*num_theta2_states + s2)
    Q = np.matrix(np.zeros([num_states, num_actions]))
    roll_delay = 0.2    #time to roll forward 0.3 seconds
    exploration_minutes = 1 #desired time to explore
    exploration_const = (exploration_minutes *60)/(roll_delay)
    previous_distance = 0.0
    t = 0
    j = 0
    
    st1 = s1
    st2 = s2
    
    ser1_angle = 90.0
    ser2_angle = 90.0
    
    for i in range (50):
        t = t + 1
        EPSILON = np.exp(-t/exploration_const)
            
        a = get_action(EPSILON, st1, st2)
        #print(a, st1, st2)
        state = setsprime(a, st1, st2)
        sprime = state[0]
        st1 = state[1]
        st2 = state[2]
        #print(sprime, st1, st2)

        #print(a, ser1_angle, ser2_angle)
        angle = set_physical_state(a, ser1_angle, ser2_angle )
        ser1_angle = angle[0]
        ser2_angle = angle[1]
        #print(ser1_angle, ser2_angle)

        time.sleep(roll_delay)      # Time taken for a robot to roll completely
        
        r = get_delta_distance_rolled(previous_distance)
        previous_distance = r[1]
        reward = r[0]
        
        if (j < 1):
            reward = 0
            j = j + 1
                     
        print(reward, previous_distance)
        look_ahead_value = get_look_ahead(sprime)

        sample = reward + gamma * look_ahead_value
        Q[s, a] = Q[s, a] + alpha*(sample - Q[s, a])
        s = sprime

        if(t == 2):
            Q = initializeQ()


        #if(t == exploration_const):
        #    print(Q)
        
    #print(Q)
    get_90degree()

       
        



if __name__ == "__main__":
    setup()
    rl_brain()
