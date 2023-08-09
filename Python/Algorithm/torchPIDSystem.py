import torch

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

    def __PIDVaribles(self):
        en1 = self.en
        self.en = self.setpoint - self.realOutput
        self.esum += self.en
        self.ed = self.en - en1

    def train(self, setpoint:float, realOutput:float)->float:     
        self.setpoint = setpoint
        self.realOutput = realOutput

        x = torch.tensor([self.en, self.esum, self.ed])
        # calculate
        y = self.net.forward(x)
        self.__PIDVaribles() # update x

        l = abs(y - self.realOutput)
        self.loss.append(l.detach().numpy())
        self.optimizer.zero_grad()
        # compute gradient
        l.backward()
        # optimize
        self.optimizer.step()

        return y.detach().numpy()[0] # as PID output 

