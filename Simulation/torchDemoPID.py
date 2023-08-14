import torch
import math
import time
from torch.utils.tensorboard import SummaryWriter

'''
    torch: 1.10.2/ 2.0.1
    python:3.6.10/ 3.8.10
    matplotlib: 2.2.3/ 3.7.2
'''
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
    # print('update:', en, esum, ed)

    return setPoint

def printWeight(net):
    print('**Weight gradient net[0] \n',net[0].weight.grad)
    print('Bias gradient net[0] :\n',net[0].bias.grad)
    print('**Weight net[0] \n',net[0].weight)
    print('Bias net[0] :\n',net[0].bias)

# 2 hidden layers: 4 and 6 neurons, and 1 output neurons
net = torch.nn.Sequential(
    torch.nn.Linear(3, 1), # PID
    # torch.nn.ReLU(),

    #followed by system model
    torch.nn.Linear(1, 3),
    torch.nn.ReLU(),
    torch.nn.Linear(3, 3),
    torch.nn.ReLU(),
    torch.nn.Linear(3, 1),
)

# with torch.no_grad():
#     net[0].weight[0,1] = 0.0
#     net[0].weight[0,2] = 0.0

#     net[0].weight[1,0] = 0.0
#     net[0].weight[1,2] = 0.0

#     net[0].weight[2,0] = 0.0
#     net[0].weight[2,1] = 0.0

# printWeight(net)

num_generation = 500
optimizer = torch.optim.Adam(net.parameters(), lr=0.1)
loss = []
writer = SummaryWriter()

def log_grad(model, logger, step):
    for tag, value in model.named_parameters():
        if value.grad is not None:
            logger.add_histogram(tag + '/grad', value.grad.cpu(), step)

def log_value(model, logger, step):
    for tag, value in model.named_parameters():
        l = value.tolist()
        if tag == 'weight':
            # print(f'tag: {tag}. value: {value}. value0: {l[0][0]}')
            logger.add_scalar(tag + '/value0', l[0][0], step)
            logger.add_scalar(tag + '/value1', l[0][1], step)
            logger.add_scalar(tag + '/value2', l[0][2], step)
        elif tag == 'bias':
            logger.add_scalar(tag + '/bias', l[0], step)


for generation in range(num_generation):
    t = time.time()
    # printWeight(net)
    # print(f'---------generation: {generation}-----------')
    input = torch.tensor([en, esum, ed])
    # print('input=:\n', input)

    output = net.forward(input)
    # print('output=net.forward(input):\n',output)

    target = update(generation)
    # print('Target:\n',target)

    l = abs(target - output)
    # print('*Loss:\n', l)
    loss.append(l.detach().numpy())
    optimizer.zero_grad() # clear x.grad for every parameter x in the optimizer.
    l.backward()
    # print('network structure: \n', net)

    optimizer.step()

    # print(f'total time: {time.time() - t}')

    # printWeight(net)
    writer.add_scalar('Loss/train', l, generation)
    log_value(net[0], writer, generation)
    writer.add_scalar('Time', time.time() - t, generation)


# import matplotlib.pyplot
# matplotlib.pyplot.plot(loss)
# matplotlib.pyplot.xlabel("Iteration")
# matplotlib.pyplot.ylabel("loss")
# matplotlib.pyplot.show()

writer.flush()
writer.close()