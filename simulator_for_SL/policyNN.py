from Model import *
from config import *
from train import MyTrainer


class Policy():
    def __init__(self):
        self.policy=Model(50,2,512,512,64)
        self.storeLearntWeightsFromData()
        self.loadWeights()

    # Uses data in apf_data.csv to optimize weights of current NN model
    # The new learnt weights are written in file Checkpoint.pth
    def storeLearntWeightsFromData(self,filename=APF_DATA_FILENAME):
        trainer=MyTrainer(filename,self.policy)
        trainer.trainDataset()
    
    # Uses data in apf_data.csv to optimize weights of current NN model
    # The new learnt weights are written in file Checkpoint.pth
    def loadWeights(self,filename='Checkpoint.pth'):
        self.policy.load_state_dict(torch.load(filename))
        self.policy.eval()

    def act(self,lidarDepths,disAndAngle):
        input1=torch.tensor([lidarDepths]).float()
        input2=torch.tensor([disAndAngle]).float()
        return self.policy.forward(input1,input2)
