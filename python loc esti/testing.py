import numpy as np
a = np.zeros((2,1))

b = np.zeros((2,10))

class Myclass:
    def __init__(self,num):
        self.num = num
    def add(self):
        self.num = self.num+1
        
number = Myclass(1)

for i in range(10):
    #print(i)
    number.add()
    print(number.num)
    
print(number.num)

F = np.array([[1,0],[0,1]])

z = np.array([[24],[12]])
print(np.dot(F,z))

c = np.array([[1,2,3],[1,2,3]])
print(c)
d = np.eye(2)
print(d)


"""could expand data's space as lower cases"""
Measurement = np.zeros((2,1))
new_col = np.array([[0], [0]]);
#Measurement = np.append(Measurement,new_col,axis=1)

for i in range(10):
    Measurement = np.append(Measurement,new_col,axis=1)
    print(Measurement)

print(2**2)

print(c[1,2])

print(200*12-880*2)

print("",1,"\n",2,"\n",2)