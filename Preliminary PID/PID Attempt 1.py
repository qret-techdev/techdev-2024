#import libraries
from simple_pid import PID
import time
import matplotlib.pyplot as plt

pid = PID(0.9, 0.05, 0.01, setpoint=100)

v=1
print(v)

counter = 0

x = [counter]
y = [v]

while(abs(v-pid.setpoint)>0.05):
    v=v+pid(v)
    print(v)
    time.sleep(0.1)

    counter = counter + 1
    x.append(counter)
    y.append(v)

pid.setpoint = 500

while(abs(v-pid.setpoint)>0.05):
    v=v+pid(v)
    print(v)
    time.sleep(0.1)

    counter = counter + 1
    x.append(counter)
    y.append(v)

pid.setpoint = 25

while(abs(v-pid.setpoint)>0.05):
    v=v+pid(v)
    print(v)
    time.sleep(0.1)

    counter = counter + 1
    x.append(counter)
    y.append(v)

plt.plot(x,y)
plt.show()