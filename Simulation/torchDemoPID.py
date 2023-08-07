import torch
import math
import time

'''
PID:
'''
en, esum, ed = 1.0, 1.0, 1.0
setPoint = 0.0

def getTrueOutput(rad):
    # output = math.cos(rad) * math.sin(rad) + math.cos(rad) + math.sin(rad)
    output = math.cos(rad) * math.sin(rad)
    return output

def update(t):
    global en, esum, ed, setPoint
    output = getTrueOutput(t/360.0)
    en1 = en
    en = setPoint - output
    esum += en
    ed = en - en1
    setPoint = output
    print('update:', en, esum, ed)

    return setPoint

def printWeight(net):
    print('**Weight gradient net[0] \n',net[0].weight.grad)
    print('Bias gradient net[0] :\n',net[0].bias.grad)
    print('**Weight net[0] \n',net[0].weight)
    print('Bias net[0] :\n',net[0].bias)

# 2 hidden layers: 4 and 6 neurons, and 1 output neurons
net = torch.nn.Sequential(
    torch.nn.Linear(3, 1),
    # torch.nn.ReLU(),
    # torch.nn.Linear(1, 3),
    # torch.nn.ReLU(),
    torch.nn.Linear(1, 1),
    # torch.nn.ReLU(),
)

# with torch.no_grad():
#     net[0].weight[0,1] = 0.0
#     net[0].weight[0,2] = 0.0

#     net[0].weight[1,0] = 0.0
#     net[0].weight[1,2] = 0.0

#     net[0].weight[2,0] = 0.0
#     net[0].weight[2,1] = 0.0

# printWeight(net)

num_generation = 20
optimizer = torch.optim.Adam(net.parameters(), lr=0.1)
loss = []

for generation in range(num_generation):
    t = time.time()
    # printWeight(net)
    print(f'---------generation: {generation}-----------')
    input = torch.tensor([en, esum, ed])
    # print('input=:\n', input)

    output = net.forward(input)
    print('output=net.forward(input):\n',output)

    target = update(generation)
    print('Target:\n',target)

    l = abs(target - output)
    print('*Loss:\n', l)
    loss.append(l.detach().numpy())
    optimizer.zero_grad() # clear x.grad for every parameter x in the optimizer.
    l.backward()
    # print('network structure: \n', net)

    optimizer.step()

    print(f'total time: {time.time() - t}')
    printWeight(net)

import matplotlib.pyplot
matplotlib.pyplot.plot(loss)
matplotlib.pyplot.xlabel("Iteration")
matplotlib.pyplot.ylabel("loss")
matplotlib.pyplot.show()