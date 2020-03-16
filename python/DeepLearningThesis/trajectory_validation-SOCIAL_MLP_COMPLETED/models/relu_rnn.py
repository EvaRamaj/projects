import torch
import torch.nn as nn
import numpy as np

from torch.autograd import Variable

class RNNModel(nn.Module):

    def __init__(self, args, infer=False):
        super(RNNModel, self).__init__()

        self.args = args
        self.infer = infer
        self.use_cuda = args.use_cuda

        if infer:
            # Test time
            self.seq_length = 1
        else:
            # Training time
            self.input_seq_length = args.seq_length

        self.rnn_size = args.rnn_size
        # self.embedding_size = args.embedding_size
        self.input_size = args.input_size
        self.output_size = args.output_size
        self.maxNumPeds = args.maxNumPeds
        self.seq_length = args.seq_length
        self.gru = args.gru

        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0)
        self.input_embedding_layer = nn.Linear(2, 64)
        self.rnn = nn.RNN(64, 256, nonlinearity='relu', batch_first=True)
        self.cell = nn.RNNCell(2, 256, nonlinearity='relu')
        self.output_layer = nn.Linear(256, 2)

    def forward(self, *args):
        input_data = args[0]
        hidden_states = args[1]

        embedding = self.dropout(self.relu(self.input_embedding_layer(input_data)))
        out_rnn, hn = self.rnn(embedding, hidden_states)
        next_position = self.output_layer(hn[0])

        hn = hn[0]
        pred_positions = [next_position.clone().requires_grad_(True)]

        for i in range(4):
            hn = self.cell(next_position, hn)
            temp = self.output_layer(hn)
            pred_positions.append(temp.clone().requires_grad_(True))
            next_position = temp

        return torch.stack(pred_positions).view(input_data.size()[0], 5, 2)
