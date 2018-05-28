from autograd import elementwise_grad as egrad
from autograd import grad
from autograd import jacobian
import matplotlib.pyplot as plt
import autograd.numpy as np

np.random.seed(69)
w = np.array([5.])
x = np.linspace(0,10,50)
y = w*x**2+np.random.randn(50)*10

def f(x):
    return w*x**2

def g(w):
    return w*x**2

def L(w):
    return np.linalg.norm(g(w)-y)
w = 0.;
Llist = []
for i in range(1000):
    Llist.append(L(w))
    Lg = egrad(L)(w)
    print(L(w), w)
    w = w - 0.001*Lg
plt.plot(x,y,x,w*x**2)
#print(jacobian(g)(w))