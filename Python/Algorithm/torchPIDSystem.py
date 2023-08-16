import torch
import torch.nn.functional as F
import time
from torch.utils.tensorboard import SummaryWriter

class myModel(torch.nn.Module):
    def __init__(self):
        super(myModel, self).__init__()
        self.fc0 = torch.nn.Linear(3,1)
        self.fc1 = torch.nn.Linear(1,3)
        self.fc2 = torch.nn.Linear(3,3)
        self.fc3 = torch.nn.Linear(3,1)
    
    def forward(self, x):
        x = F.sigmoid(self.fc0(x))
        # x = self.fc0(x)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)

activation = {}
def get_activation(name):
    def hook(model, input, output):
        activation[name] = output.detach()
    return hook

class Net:
    def __init__(self):
        # self.net = torch.nn.Sequential(
        #     torch.nn.Linear(3,1), #PID
        #     #followed by system model
        #     torch.nn.Sigmoid(),
        #     torch.nn.Linear(1, 3),
        #     torch.nn.ReLU(),
        #     torch.nn.Linear(3, 3),
        #     torch.nn.ReLU(),
        #     torch.nn.Linear(3, 1),
        # )
        self.net = myModel()
        self.net.fc0.register_forward_hook(get_activation('fc0'))
        self.optimizer = torch.optim.Adam(self.net.parameters(), lr=0.05)
        self.en, self.esum, self.ed = 0.0, 0.0, 0.0
        # self.loss = []
        self.writer = SummaryWriter()
        self.step = 0

    def __del__(self):
        self.writer.flush()
        self.writer.close()
        print('Destructor called.')

    def __PIDVaribles(self):
        en1 = self.en
        self.en = self.setpoint - self.realOutput
        self.esum += self.en
        self.ed = self.en - en1
            
    def __log_value(self, model, step):
        for tag, value in model.named_parameters():
            l = value.tolist()
            if tag == 'weight':
                # print(f'tag: {tag}. value: {value}. value0: {l[0][0]}')
                self.writer.add_scalar(tag + '/value0', l[0][0], step)
                self.writer.add_scalar(tag + '/value1', l[0][1], step)
                self.writer.add_scalar(tag + '/value2', l[0][2], step)
            elif tag == 'bias':
                self.writer.add_scalar(tag + '/bias', l[0], step)

    def __log(self, model, step, **kwargs):
        '''
            keywords: loss=x, deltaT=x, PIDOut=x
        '''
        self.__log_value(model, step)
        for key, value in kwargs.items():
            self.writer.add_scalar(key, value, step)

    def train(self, setpoint:float, realOutput:float)->float:     
        t = time.time()
        self.step += 1 

        self.setpoint = setpoint
        self.realOutput = realOutput

        x = torch.tensor([self.en, self.esum, self.ed])
        # calculate
        y = self.net.forward(x)
        y0 = F.sigmoid(activation['fc0'])
        self.__PIDVaribles() # update x

        fLoss = torch.nn.MSELoss()
        factor = 1
        input = torch.tensor([self.realOutput * 10, y * 10, 0], requires_grad=True) 
        target = torch.tensor([self.setpoint * 10, self.realOutput * 10, 0]) 
        l = fLoss(input, target)
        self.optimizer.zero_grad()
        # compute gradient
        l.backward()
        # optimize
        self.optimizer.step()
        loss_ = self.realOutput - self.setpoint
        self.__log(model=self.net.fc0, step=self.step, loss=loss_, deltaT=time.time() - t, PIDOut=y0)

        return y0.detach().numpy()[0] # as PID output 

