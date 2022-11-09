# class Policy():
#     def __init__(self):

#     def train(self,"filename"):

#     def forward(self, x):
#         x = F.relu(self.fc1(x))
#         x = self.fc2(x)
#         return F.softmax(x, dim=1)
    
#     def act(self, state):
#         state = torch.from_numpy(state).float().unsqueeze(0).to('cpu')
#         probs = self.forward(state).cpu()
#         m = Categorical(probs)
#         action = torch.argmax(m.logits)
#         return action.item(), m.log_prob(torch.tensor(action))