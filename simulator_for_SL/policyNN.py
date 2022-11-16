from Model import *

class Policy():
    def __init__(self):
        self.policy=Model(50,2,512,512,64)
        PATH='Checkpoint.pth'
        self.policy.load_state_dict(torch.load(PATH))
        self.policy.eval()

    def act(self,lidarDepths,disAndAngle):
        input1=torch.tensor([lidarDepths]).float()
        input2=torch.tensor([disAndAngle]).float()
        return self.policy.forward(input1,input2)
