# from threading import Thread
# import time

# done = False

# def boss():
#     adder = 0
#     while not done:
#         adder += 10
#         time.sleep(1)
#         print(adder)

# def worker():
#     counter = 0
#     while not done:
#         counter+=1
#         time.sleep(1)
#         print(counter)

# Thread(target=worker, daemon = True).start()
# Thread(target=boss, daemon = False).start()


# input("Entr to break")

# done = True

'''------------------------------------------------------------------'''

# import threading
# import time

# def worker():
#     print("Worker thread started")
#     time.sleep(2)
#     print("Worker thread finished")

# t = threading.Thread(target=worker)

# t.start()

# t.join()  #waits for the current proces to complete an then goes to the next step

# print("Main program finished")

'''------------------------------------------------------------------'''

import threading
import time

def task(name):
    print("Task", name, "starting")
    time.sleep(2)
    print("Task", name, "finished")

threads = []
time_var = time.time()   #starts the timer clock


for i in range(5):
    t = threading.Thread(target=task, args=(i,)) #here args= is used to pass the argument to the threadf that is in need
    threads.append(t)
    t.start()

for t in threads:
    t.join()

print(f"Done in {time.time()-time_var:.4f}")

print("All tasks completed")
