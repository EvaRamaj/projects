"""
Train script for the DESIRE model

Author: Christian Weber
Date: 12th January 2018

A lot of the Code was used and modified from the Social LSTM model

Author: Anirudh Vemula
Date: 13th June 2017
GitHub: https://github.com/vvanirudh/social-lstm-tf
"""

import argparse
from data_loaders.ngsim_data_loader import DataLoader_NGSIM
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
    parser.add_argument('--input_seq_length', type=int, default=20,
                        help='Model input sequence length')
    # Length of the target RNN sequence length parameter
    parser.add_argument('--target_seq_length', type=int, default=40,
                        help='Model target sequence length')
    # Maximum number of agents to be considered
    parser.add_argument('--max_agents', type=int, default=1000,
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
    parser.add_argument('--batch_size', type=int, default=16,
                        help='minibatch size')
    # Number of epoch parameter
    parser.add_argument('--num_epochs', type=int, default=30,
                        help='number of epochs')
    # Learning rate parameter
    parser.add_argument('--learning_rate', type=float, default=0.004,
                        help='learning rate')
    # Gradient value at which it should be clipped
    parser.add_argument('--grad_clip', type=float, default=1.,
                        help='clip gradients at this value')
    # Decay rate for the learning rate parameter
    parser.add_argument('--decay_rate', type=float, default=0.95,
                        help='decay rate')
    # Restore model from a previous checkpoint
    parser.add_argument('--restore_model', type=bool, default=False,
                        help='Restore the model from a saved checkpoint')
    # Define the file to restore from
    parser.add_argument('--restore_file', type=str, default="model.ckpt-4",
                        help='File which is supposed to be restored')

    args = parser.parse_args()
    train(args)


def train(args):

    # Create a data_loader object
    # data_loader = DataLoader_NGSIM(args, forcePreProcess=True, infer=False)
    statistics = Statistics('./data/preprocessed/NGSIM/train_trajectories_0.cpkl')
    # statistics.video_from_frame_array('./data/preprocessed/NGSIM/videos/', 'peachtree')
    # statistics.create_array()
    # statistics.sanitize_array()
    # statistics.trans_agent_location('./data/preprocessed/NGSIM/train_array_global_local.cpkl')
    statistics.load_array('./data/preprocessed/NGSIM/train_trajectories_sanitized_array_1.cpkl')
    statistics.transform_to_wgs84('us101')
    # statistics.noise()
    # statistics.offset()
    # statistics.r_squared()
    # statistics.curvature_with_space_sampling()
    # statistics.curvature_with_time_sampling()


if __name__ == '__main__':
    main()
