from model import *
from config import *
from train import MyTrainer


class Policy():
    def __init__(self):
        self.model=Model(50,2,512,512,64)

    # Uses data in apf_data.csv to optimize weights of current NN model
    # The new learnt weights are written in file Checkpoint.pth
    def storeLearntWeightsFromData(self,dataFilename="apf_data.csv",checkpointFilename="checkpoint.pth"):
        trainer=MyTrainer(dataFilename,self.model)
        trainer.trainDataset(checkpointFilename)
    
    # Uses data in apf_data.csv to optimize weights of current NN model
    # The new learnt weights are written in file Checkpoint.pth
    def loadWeights(self,checkpointFilename='checkpoint.pth'):
        self.model.load_state_dict(torch.load(checkpointFilename))
        self.model.eval()

    def act(self,lidarDepths,disAndAngle):
        input1=torch.tensor([lidarDepths]).float()
        input2=torch.tensor([disAndAngle]).float()
        return self.model.forward(input1,input2)
