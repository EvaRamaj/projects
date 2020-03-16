import torch
import torch.nn as nn
import numpy as np

from torch.autograd import Variable

class TestModel(nn.Module):

    def __init__(self, args, infer=False):
        super(TestModel, self).__init__()

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

        self.input_embedding_layer = nn.Linear(2, 2)
        self.lstm = nn.LSTM(64, 128, batch_first=True)
        self.cell = nn.LSTMCell(2, 128)
        self.output_layer = nn.Linear(128, 2)

    def forward(self, *args):
        input_data = args[0]
        hidden_states = args[1]
        cell_states = args[2]
        pred_positions = []

        for i in range(39):
            embedding = self.input_embedding_layer(input_data)
            pred_positions.append(embedding.clone().detach().requires_grad_(True))

        return embedding
