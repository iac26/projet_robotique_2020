from Thymio import Thymio
import time

th = Thymio.serial(port="COM7", refreshing_rate=0.1)
time.sleep(10) # To make sure the Thymio has had time to connect
print("Thymio is connected :)")


th.set_var("motor.left.target", 100)
th.set_var("motor.right.target", 100)

for i in range(100):
    time.sleep(0.1)
    value_speed=[th['motor.left.speed'],th['motor.right.speed']]
    print(value_speed)



th.set_var("motor.left.target", 0)
th.set_var("motor.right.target", 0)



