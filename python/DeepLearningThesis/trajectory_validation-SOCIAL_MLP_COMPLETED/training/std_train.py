import torch
import torch.nn as nn
import numpy as np
from torch.autograd import Variable
from torch.utils.tensorboard import SummaryWriter
import matplotlib.pyplot as plt

import copy
import argparse
import os
import time
import pickle
import subprocess

from data_loaders.std_data_loader import DataLoader_Stanford

import argparse
from data_loaders.ngsim_data_loader import DataLoader_NGSIM
from data_loaders.eth_data_loader import DataLoader_ETH
from models.vlstm import VLSTMModel
from models.relu_rnn import RNNModel
from models.no_feed import RNN_MLP_Model
from models.mlp_lstm import LSTM_MLP_Model
from models.test_model import TestModel
from data_loaders.eth_data_loader import DataLoader_ETH
from statistics import Statistics
from util import write_pickle


def main():
    parser = argparse.ArgumentParser()

    # ----------------------------------- model parameters ----------------------------------------------

    # RNN size parameter of the Encoders and Decoder (dimensions of the output/hidden state)
    parser.add_argument('--rnn_size', type=int, default=48,
                        help='size of RNN hidden state')
    # Type of recurrent unit parameter
    parser.add_argument('--rnn_type', type=str, default='gru',
                        help='gru, or lstm')
    # Length of the input RNN sequence length parameter
    parser.add_argument('--input_seq_length', type=int, default=10,
                        help='Model input sequence length')
    # Length of the target RNN sequence length parameter
    parser.add_argument('--target_seq_length', type=int, default=5,
                        help='Model target sequence length')
    # Maximum number of agents to be considered
    parser.add_argument('--max_agents', type=int, default=205,
                        help='Maximum number of agents')
    # Number of agents we want to reduce the data to, to speed up training
    parser.add_argument('--reduce_agents', type=int, default=0,
                        help='Number of agents we want to reduce the data to - set to 0 if not used')
    # Number of predictions k
    parser.add_argument('--num_predictions', type=int, default=5,
                        help='Number of trajectory predictions k')
    # Pooling frequency: perform social pooling every n steps
    parser.add_argument('--pooling_frequency', type=int, default=5,
                        help='Pool every n steps. Set to 1 to pool every step')
    # Size of the latent space vector z
    parser.add_argument('--z_dim', type=int, default=48,
                        help='Size of the latent space vector z')
    # Number of refinement iterations
    parser.add_argument('--num_refinements', type=int, default=1,
                        help='Number of refinement iterations of decoder2')

    # ---------------------------------- training parameters ----------------------------------------------

    # Size of each batch parameter
    parser.add_argument('--batch_size', type=int, default=12,
                        help='minibatch size')
    # Number of epoch parameter
    parser.add_argument('--num_epochs', type=int, default=30,
                        help='number of epochs')
    # Learning rate parameter
    parser.add_argument('--learning_rate', type=float, default=0.0001,
                        help='learning rate')
    # Gradient value at which it should be clipped
    parser.add_argument('--grad_clip', type=float, default=1.,
                        help='clip gradients at this value')
    # Decay rate for the learning rate parameter
    parser.add_argument('--decay_rate', type=float, default=0.95,
                        help='decay rate')
    parser.add_argument('--use_cuda', action="store_true", default=True,
                        help='Use GPU or not')
    # Lambda regularization parameter (L2)
    parser.add_argument('--lambda_param', type=float, default=0.0005,
                        help='L2 regularization parameter')
    parser.add_argument('--seq_length', type=int, default=20,
                       help='RNN sequence length')
    parser.add_argument('--input_size', type=int, default=2)
    parser.add_argument('--output_size', type=int, default=5)
    parser.add_argument('--maxNumPeds', type=int, default=27,
                        help='Maximum Number of Pedestrians')
    parser.add_argument('--gru', action="store_true", default=False,
                        help='True : GRU cell, False: LSTM cell')
    parser.add_argument('--restore_model', type=bool, default=False,
                        help='restore the saved model')
    parser.add_argument('--restore_file', type=str, default='model_0.tar',
                        help='file where the model is saved')

    args = parser.parse_args()
    train(args)

def get_printer(msg):
    """This function returns a printer function, that prints information about a  tensor's
    gradient. Used by register_hook in the backward pass.
    """
    def printer(tensor):
        if tensor.nelement() == 1:
            print(f"{msg} {tensor}")
        else:
            print(f"{msg} tensor: {tensor}"
                  f" max: {tensor.max()} min: {tensor.min()}"
                  f" mean: {tensor.mean()}")
    return printer


def register_hook(tensor, msg):
    """Utility function to call retain_grad and Pytorch's register_hook
    in a single line
    """
    print("current tensor", tensor)
    tensor.retain_grad()
    tensor.register_hook(get_printer(msg))
    print("tensor after hook", tensor)


def train(args):
    dataloader = DataLoader_ETH(args, forcePreProcess=True, infer=False)

    writer = SummaryWriter(log_dir='../runs/stanford/MLP_LSTM')

    # model creation
    # net = VLSTMModel(args)
    # net = RNNModel(args)
    # net = RNN_MLP_Model(args)
    net = LSTM_MLP_Model(args)
    # writer.add_graph(net, dataloader[0], verbose=True)

    if args.use_cuda:
        net = net.cuda()
    learning_rate = args.learning_rate


    # optimizer = torch.optim.RMSprop(net.parameters(), lr=args.learning_rate)
    # optimizer = torch.optim.Adagrad(net.parameters(), lr=learning_rate, weight_decay=args.lambda_param)
    optimizer = torch.optim.Adam(net.parameters(), lr=learning_rate, weight_decay=args.lambda_param)
    start_epoch = 0

    if args.restore_model:
        checkpoint = torch.load('../saved_models/' + args.restore_file)
        net.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        start_epoch = checkpoint['epoch']
        loss_epoch = checkpoint['loss']

    best_val_loss = 100
    best_val_data_loss = 100

    smallest_err_val = 100000
    smallest_err_val_data = 100000

    best_epoch_val = 0
    best_epoch_val_data = 0

    best_err_epoch_val = 0
    best_err_epoch_val_data = 0

    all_epoch_results = []
    running_loss = 0
    valid_running_loss = 0
    counter = 0

    batch_counter = 0
    valid_batch_counter = 0
    plot_x_batches = []
    plot_y_batches = []
    plot_num_agents_batches = []

    for epoch in range(start_epoch, 10000):
        net.train()
        loss_epoch = 0

        # For each batch
        for batch in range(dataloader.num_batches):
            batch_counter += 1
            # x_batch, y_batch source target respectively all the positions for each agent
            x_batch, y_batch, d, image, image_size, num_agents = dataloader.next_batch(random_update=False)

            if epoch == 0 and batch == 0:
                plot_x_batch, plot_y_batch, plot_num_agents = copy.deepcopy(x_batch), copy.deepcopy(y_batch), \
                                                              copy.deepcopy(num_agents)
                plot_x_batches.append(plot_x_batch)
                plot_y_batches.append(plot_y_batch)
                plot_num_agents_batches.append(plot_num_agents)

            loss_batch = 0
            # Zero out gradients
            net.zero_grad()
            optimizer.zero_grad()

            # x_batch[sequence] all agents' positions in this scene
            for sequence in range(dataloader.batch_size):
                # x_seq, targets, d_seq, numPedsList_seq, PedsList_seq, num_agents_seq = x_batch[sequence],
                # y_batch[sequence],\
                #                                                             d[sequence], image[sequence],\
                #                                                       image_size[sequence], num_agents[sequence]
                x_seq, targets, num_agents_seq = x_batch[sequence], y_batch[sequence], num_agents[sequence]
                x_seq = torch.from_numpy(x_seq[:num_agents_seq, :, 1:3]).float()
                targets = torch.from_numpy(targets[:num_agents_seq, :, 1:3]).float()

                if args.use_cuda:
                    x_seq = x_seq.cuda().detach().requires_grad_()
                    targets = targets.cuda()
                # give initial state for lstm
                hidden_states = Variable(torch.zeros(1, num_agents_seq, 256), requires_grad=True)

                if args.use_cuda:
                    hidden_states = hidden_states.cuda()


                cell_states = Variable(torch.zeros(1, num_agents_seq, 256), requires_grad=True)
                if args.use_cuda:
                    cell_states = cell_states.cuda()


                # Forward prop
                outputs = net(x_seq, hidden_states, cell_states)

                # Compute loss
                loss = nn.MSELoss()
                mse = loss(outputs, targets)
                loss_batch += mse
                # print(net.input_embedding_layer.weight.grad)

                # register_hook(outputs, " input grad")
                # Compute gradients


                # plot history trajectory, target and predicted for each agent
            loss_batch = loss_batch / dataloader.batch_size
            loss_batch.backward()
                # print(net.input_embedding_layer.weight.grad)
                # Clip gradients
                # torch.nn.utils.clip_grad_norm_(net.parameters(), args.grad_clip)

                # Update parameters
            optimizer.step()
            loss_epoch += loss_batch

        torch.save({
            'epoch': epoch,
            'model_state_dict': net.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'loss': loss_epoch,
        }, '../saved_models/model_' + str(epoch) + '.tar')
        net.eval()
        if epoch % 5 == 0:
            batch_num = min(len(plot_x_batches), 20)
            for i in range(batch_num):
                plot_x_batch, plot_y_batch, plot_num_agents = plot_x_batches[i], plot_y_batches[i], \
                                                              plot_num_agents_batches[i]
                for seq in range(dataloader.batch_size):
                    plot_x_seq, plot_targets, plot_num_agents_seq = plot_x_batch[seq], plot_y_batch[seq], \
                                                                    plot_num_agents[seq]
                    plot_x_seq = torch.from_numpy(plot_x_seq[:plot_num_agents_seq, :, 1:3]).float()
                    plot_targets = torch.from_numpy(plot_targets[:plot_num_agents_seq, :, 1:3]).float()

                    if args.use_cuda:
                        plot_x_seq = plot_x_seq.cuda().detach().requires_grad_()
                        plot_targets = plot_targets.cuda()
                    # give initial state for lstm
                    hidden_states = Variable(torch.zeros(1, plot_num_agents_seq, 256), requires_grad=True)

                    if args.use_cuda:
                        hidden_states = hidden_states.cuda()


                    cell_states = Variable(torch.zeros(1, plot_num_agents_seq, 256), requires_grad=True)
                    if args.use_cuda:
                        cell_states = cell_states.cuda()

                    # Forward prop
                    plot_outputs = net(plot_x_seq, hidden_states, cell_states)

                    init_trajectory = torch.cat([plot_x_seq, plot_targets], 1).cpu().detach().numpy()
                    pred_trajectory = plot_outputs.cpu().detach().numpy()
                    for agent, _ in enumerate(init_trajectory):
                        tag = str(epoch) + ',' + str(i) + ',' + str(seq) + ',' + str(agent)
                        sorted_trajectory = init_trajectory[agent]#[np.argsort(init_trajectory[agent][:, 0])]
                        sorted_pred_trajectory = pred_trajectory[agent]#[np.argsort(pred_trajectory[agent][:, 0])]
                        if init_trajectory.shape[1] > 30:
                            counter += 1
                            print("ton exw megalo", counter)
                        figure = plt.figure()
                        plt.axis("equal")
                        plt.plot(plot_x_seq[agent][:, 0].cpu().detach().numpy(), plot_x_seq[agent][:, 1].cpu().detach().numpy(), 'b',
                                 plot_targets[agent][:, 0].cpu().detach().numpy(), plot_targets[agent][:, 1].cpu().detach().numpy(), 'g',
                                 sorted_pred_trajectory[:, 0], sorted_pred_trajectory[:, 1], 'r'
                                 )
                        writer.add_figure(tag, figure, global_step=None, close=True, walltime=None)
                        plt.close()

        loss_epoch /= dataloader.num_batches
        writer.add_scalar('Loss/train/exp6', loss_epoch.item(), epoch)

        if dataloader.valid_num_batches > 0:
            print('****************Validation epoch beginning******************')
            loss_epoch = 0
            # Validation
            for batch in range(dataloader.valid_num_batches):
                valid_batch_counter += 1
                # Get batch data
                x_batch, y_batch, d, image, image_size, num_agents = dataloader.next_valid_batch(random_update=False)

                # Loss for this batch
                loss_batch = 0
                err_batch = 0

                # For each sequence
                for sequence in range(dataloader.batch_size):
                    x_seq, targets, num_agents_seq = x_batch[sequence], y_batch[sequence], num_agents[sequence]

                    x_seq = torch.from_numpy(x_seq[:num_agents_seq, :, 1:3]).float().requires_grad_(True)
                    targets = torch.from_numpy(targets[:num_agents_seq, :, 1:3]).float()

                    if args.use_cuda:
                        x_seq = x_seq.cuda().detach().requires_grad_()
                        targets = targets.cuda()
                    # give initial state for lstm
                    hidden_states = Variable(torch.zeros(1, num_agents_seq, 256), requires_grad=True)

                    if args.use_cuda:
                        hidden_states = hidden_states.cuda()

                    cell_states = Variable(torch.zeros(1, num_agents_seq, 256), requires_grad=True)
                    if args.use_cuda:
                        cell_states = cell_states.cuda()

                    # Forward prop
                    outputs = net(x_seq, hidden_states, cell_states)

                    # Compute loss
                    loss = nn.MSELoss()
                    mse = loss(outputs, targets)
                    loss_batch += mse
                    # print(net.input_embedding_layer.weight.grad)
                    # Zero out gradients
                    # net.zero_grad()
                    optimizer.zero_grad()

                loss_batch = loss_batch / dataloader.batch_size
                loss_epoch += loss_batch
                # Update best validation loss until now
                if loss_epoch < best_val_loss:
                    best_val_loss = loss_epoch
                    best_epoch_val = epoch

            loss_epoch /= dataloader.valid_num_batches
            writer.add_scalar('Loss/valid/exp6', loss_epoch.item(), epoch)

    print('(epoch {}), valid_loss = {:.3f}'.format(epoch, loss_epoch))
    print('Best epoch', best_epoch_val, 'Best validation loss', best_val_loss, 'Best error epoch', best_err_epoch_val,
          'Best error', smallest_err_val)
    writer.close()


if __name__ == '__main__':
    main()



