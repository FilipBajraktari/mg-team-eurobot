import odrive as od
import time

a = od.find_any()
diffmax = 0
diffmin = 9999
diffavg = 0

for i in range(10000):
    f1  = time.time()
    a.axis0.controller.input_vel=0.5
    a.axis1.controller.input_vel=0.5
    f2 = time.time()
    diff = f2-f1
    diffmax = max(diff,diffmax)
    diffmin = min(diff,diffmin)
    diffavg += diff
    print(diff)
print(f'max: {diffmax}')
print(f'min: {diffmin}')
print(f'avg: {diffavg/10000}')

a.axis0.controller.input_vel=0
a.axis1.controller.input_vel=0
