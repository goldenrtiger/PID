import torch
import torch.nn.functional as F

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

    return setPoint

# 2 hidden layers: 4 and 6 neurons, and 1 output neurons
# net = torch.nn.Sequential(
#     torch.nn.Linear(3, 1), # PID
#     torch.nn.Sigmoid(),

#     #followed by system model
#     torch.nn.Linear(1, 3),
#     torch.nn.ReLU(),
#     torch.nn.Linear(3, 3),
#     torch.nn.ReLU(),
#     torch.nn.Linear(3, 1),
# )


class myModel(torch.nn.Module):
    def __init__(self):
        super(myModel, self).__init__()
        self.fc0 = torch.nn.Linear(3,1)
        self.fc1 = torch.nn.Linear(1,3)
        self.fc2 = torch.nn.Linear(3,3)
        self.fc3 = torch.nn.Linear(3,1)
    
    def forward(self, x):
        # x = F.sigmoid(self.fc0(x))
        x = self.fc0(x)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

activation = {}
def get_activation(name):
    def hook(model, input, output):
        activation[name] = output.detach()
    return hook

net = myModel()
net.fc0.register_forward_hook(get_activation('fc0'))

num_generation = 5000
optimizer = torch.optim.Adam(net.parameters(), lr=0.1)
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
    input = torch.tensor([en, esum, ed])

    output = net.forward(input)
    output0 = activation['fc0']

    target = torch.tensor(update(generation))

    fLoss = torch.nn.MSELoss()
    l = fLoss(target, output)
    # l = target - output
    optimizer.zero_grad() # clear x.grad for every parameter x in the optimizer.
    l.backward()

    optimizer.step()

    writer.add_scalar('Loss/train', l, generation)
    log_value(net.fc0, writer, generation)
    writer.add_scalar('Time', time.time() - t, generation)

writer.flush()
writer.close()