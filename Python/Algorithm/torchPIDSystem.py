import torch
import time
from torch.utils.tensorboard import SummaryWriter

class Net:
    def __init__(self):
        self.net = torch.nn.Sequential(
            torch.nn.Linear(3,1), #PID
            #followed by system model
            torch.nn.Linear(1, 3),
            torch.nn.ReLU(),
            torch.nn.Linear(3, 3),
            torch.nn.ReLU(),
            torch.nn.Linear(3, 1),
        )
        self.optimizer = torch.optim.Adam(self.net.parameters(), lr=0.1)
        self.en, self.esum, self.ed = 0.0, 0.0, 0.0
        self.loss = []
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
        y0 = self.net[0].forward(x)
        # print(f'weights of self.net[0]: {self.net[0].weight}')
        # print(f'bias of self.net[0]: {self.net[0].bias}')
        self.__PIDVaribles() # update x

        l = abs(y - self.realOutput)
        self.loss.append(l.detach().numpy())
        self.optimizer.zero_grad()
        # compute gradient
        l.backward()
        # optimize
        self.optimizer.step()

        self.__log(model=self.net[0], step=self.step, loss=l, deltaT=time.time() - t, PIDOut=y0)

        return y0.detach().numpy()[0] # as PID output 

