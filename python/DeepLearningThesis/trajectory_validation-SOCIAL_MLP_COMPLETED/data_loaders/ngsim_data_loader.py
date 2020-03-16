'''
The data loader script for the Stanford Drone Dataset handles the pre processing of the data,
as well as processing the input and target data in batches and sequences.

Author : Christian Weber
Date : April 2018

A lot of the Code was used and modified from the Social LSTM implementation by:
Author: Anirudh Vemula
Date: 17th October 2017
GitHub: https://github.com/vvanirudh/social-lstm-tf
'''

import os
import pickle
import numpy as np
import cv2 as cv
import random
import pandas as pd

feet_to_meters = 1 / 3.2808
ms_to_sec = 0.001


# The data loader class that loads data from the datasets, considering each frame as a datapoint
# and a sequence of consecutive frames as the sequence.
class DataLoader_NGSIM:

    def __init__(self, args, forcePreProcess=False, infer=False):
        """
        Initialiser function for the SocialDataLoader class
        params:
        args: Arguments for the model
        forcePreProcess : Flag to forcefully preprocess the data again from the .txt files
        infer: Flag to process the data in inference mode
        """

        self.infer = infer

        # Instead of every frame only every nth frame is considered
        self.skip = 3

        # List containing the training data directories where the raw data resides
        self.input_dir = './data/NGSIM/initial.csv'

        if self.infer:
            # In inference mode
            self.batch_size = 1
        else:
            # In training mode
            self.batch_size = args.batch_size

        # Data directory where the pre-processed pickle file resides
        self.data_dir = './data/preprocessed/NGSIM/'

        # Maximum number of agents in a single frame (Number obtained by checking the datasets)
        self.max_agents = args.max_agents
        # Reduce training data to reduce_agents number of agents to speed up training
        self.reduce_agents = args.reduce_agents

        # Store the arguments
        self.input_seq_length = args.input_seq_length
        self.target_seq_length = args.target_seq_length

        # Validation ratio
        self.val_fraction = 0.2

        # Define the path in which the processed data would be stored
        train_file = os.path.join(self.data_dir, "train_trajectories.cpkl")
        test_file = os.path.join(self.data_dir, "test_trajectories.cpkl")

        # If the train file doesn't exist or forcePreProcess is true
        if not (os.path.exists(train_file)) or forcePreProcess:
            print("Creating pre-processed training data from raw data")
            # Preprocess the data from the txt files of the datasets
            # Note that this data is processed in frames
            self.frame_preprocess(self.input_dir, train_file)

        # If the test file doesn't exist or forcePreProcess is true
        if not (os.path.exists(test_file)) or forcePreProcess:
            print("Creating pre-processed testing data from raw data")
            # Pre-process the data from the txt files of the datasets
            # Note that this data is processed in frames
            # self.frame_preprocess(self.input_dir, test_file)

        # Load the processed data from the pickle file
        if self.infer:
            self.load_preprocessed(test_file)
        else:
            self.load_preprocessed(train_file)

        # Reset all the data pointers of the dataloader object
        self.reset_batch_pointer(valid=False)
        self.reset_batch_pointer(valid=True)

    # noinspection PyTypeChecker
    def frame_preprocess(self, data_dir, data_file):
        """
        Function that will pre-process the annotations.txt files of each dataset
        params:
        data_dir : List of directories where raw data resides
        data_file : The file into which all the pre-processed data needs to be stored
        """

        # all_frame_data is a list of numpy arrays corresponding to each dataset
        # Each numpy array corresponds to a frame and is of size (num_agents, 3) each row containing (agentID, x, y)
        all_frame_data = []
        # Validation frame data
        valid_frame_data = []
        # frameList_data is a list of lists corresponding to each dataset
        # Each list contains the frameIds of all the frames in the dataset
        frame_list_data = []
        # num_agents data would be a list of lists corresponding to each dataset
        # Each list contains the number of agents in each frame in the dataset
        num_agents_data = []
        # Image data and image size contain the image data and it's width, height of the current dataset
        image_data = []
        image_size = []
        # Index of the current dataset
        dataset_index = 0
        chunksize = 10 ** 6
        datasets = {}
        dataset_keys = []
        counter = 0
        for chunk in pd.read_csv(data_dir, chunksize=chunksize):
            print(counter)
            counter += 1
            numpy_chunk = chunk.to_numpy()
            print(numpy_chunk.shape)
            keys = list(np.unique(numpy_chunk[:, -1]))
            data_for_new_keys = [(numpy_chunk[np.where(numpy_chunk[:, -1] == key)], key) for key in keys if key not
                                 in dataset_keys]
            data_for_old_keys = [(numpy_chunk[np.where(numpy_chunk[:, -1] == key)], key) for key in keys if key
                                 in dataset_keys]
            dataset_keys += keys
            dataset_keys = list(set(dataset_keys))
            for data, key in data_for_new_keys:
                datasets[key] = [data]
            for data, key in data_for_old_keys:
                datasets[key] = np.vstack([dataset for dataset in datasets[key]] + [data]).astype(str)
        for key in datasets.keys():
            if type(datasets[key]) == list:
                datasets[key] = np.vstack([dataset for dataset in datasets[key]]).astype(str)
            # datasets[key] = np.unique(datasets[key], axis=0)
        # For each dataset
        for dataset in list(datasets.values()):
            print("Starting", dataset)
            self.skip = self.skip
            frame_list = np.unique(dataset[:, 3]).astype(np.float).tolist()
            # Number of frames
            num_frames = len(np.asarray(frame_list))
            if self.infer:
                valid_num_frames = 0
            else:
                valid_num_frames = int(num_frames * self.val_fraction)

            # Add the list of frameIDs to the frameList_data
            frame_list_data.append(frame_list)
            # Initialize the list of num_agents for the current dataset
            num_agents_data.append([])
            # Initialize the numpy array for the current dataset
            all_frame_data.append(np.zeros((num_frames - valid_num_frames + 1, self.max_agents, 19)))
            # Initialize the numpy array for the current dataset
            valid_frame_data.append(np.zeros((valid_num_frames, self.max_agents, 19)))
            # Add the image to the image_data

            curr_frame = 0

            for ind, frame in enumerate(frame_list):
                if ind % 1000 == 0:
                    print("Index", ind, "out of: ", len(frame_list), " for dataset", dataset_index)
                if ind % self.skip != 0:
                    # Skip every n frames
                    continue

                # Extract all data of the current frame
                data_in_frame = dataset[dataset[:, 3].astype(np.float) == frame, :]
                # Extract agents list from the data in current frame
                agents_list = data_in_frame[:, 0].astype(np.float).tolist()

                # Add number of agents in the current frame to the stored data
                num_agents_data[dataset_index].append(len(agents_list))

                # Initialize the row of the numpy array
                agents_with_pos = []

                # For each agent in the current frame
                for agent in agents_list:
                    # Extract their x and y positions defining the bounding box
                    total_frames = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 2][0].astype(np.float)
                    current_time = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 3][0].astype(
                        np.float) * ms_to_sec
                    current_local_x = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 4][0].astype(
                        np.float) * feet_to_meters
                    current_local_y = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 5][0].astype(
                        np.float) * feet_to_meters
                    current_global_x = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 6][0].astype(
                        np.float)
                    current_global_y = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 7][0].astype(
                        np.float)
                    v_length = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 8][0].astype(
                        np.float) * feet_to_meters
                    v_width = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 9][0].astype(
                        np.float) * feet_to_meters
                    v_class = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 10][0].astype(
                        np.float)
                    v_vel = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 11][0].astype(
                        np.float) * feet_to_meters
                    v_acc = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 12][0].astype(
                        np.float) * feet_to_meters
                    lane_id = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 13][0].astype(
                        np.float)
                    origin = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 14][0]
                    destination = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 15][0]
                    intersection_id = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 16][0]
                    section_id = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 17][0]
                    direction = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 18][0]
                    movement = data_in_frame[data_in_frame[:, 0].astype(float) == agent, 19][0]

                    # Normalize coordinates depending on the image size
                    # Add their pedID, x, y to the row of the numpy array
                    agents_with_pos.append([agent, total_frames, current_local_x, current_local_y, current_global_x,
                                            current_global_y, v_length, v_width, v_class, v_vel, v_acc, lane_id, origin,
                                            destination, intersection_id, section_id, direction, movement,
                                            current_time])

                if (curr_frame >= valid_num_frames) or self.infer:
                    # At inference time, no validation data
                    # Add the details of all the agents in the current frame to all_frame_data
                    all_frame_data[dataset_index][curr_frame - valid_num_frames, 0:len(agents_list), :] = \
                        np.array(agents_with_pos)
                else:
                    valid_frame_data[dataset_index][curr_frame, 0:len(agents_list), :] = np.array(agents_with_pos)

                # Increment of the frame index
                curr_frame += 1

            data_file = './data/preprocessed/NGSIM/train_trajectories_' + repr(dataset_index) + '.cpkl'
            f = open(data_file, "wb")
            pickle.dump((all_frame_data[dataset_index], frame_list_data[dataset_index],
                         num_agents_data[dataset_index], valid_frame_data[dataset_index], image_data, image_size), f,
                        protocol=2)
            f.close()

            # Increase the dataset index
            dataset_index += 1

    def load_preprocessed(self, data_file):
        """
        Function to load the pre-processed data into the DataLoader object
        params:
        data_file : the path to the pickled data file
        """
        # Load data from the pickled file
        f = open(data_file, 'rb')
        self.raw_data = pickle.load(f)
        f.close()

        # Get all the data from the pickle file
        self.data = self.raw_data[0]
        self.frame_list = self.raw_data[1]
        self.num_agents_list = self.raw_data[2]
        self.valid_data = self.raw_data[3]
        self.image_data = self.raw_data[4]
        self.image_size = self.raw_data[5]

        counter = 0
        valid_counter = 0
        max_agents = 0

        # For each dataset
        for dataset in range(len(self.data)):
            # get the frame data for the current dataset
            all_frame_data = self.data[dataset]
            valid_frame_data = self.valid_data[dataset]
            max_num_agents = np.amax(self.num_agents_list[dataset], axis=0)

            print(
                "Dataset {}: training data   = {} , maximum number of agents = {}".format(dataset, len(all_frame_data),
                                                                                          max_num_agents))
            print("Dataset {}: validation data = {}".format(dataset, len(valid_frame_data)))

            # Increment the counter with the number of sequences in the current dataset
            counter += int(len(all_frame_data) / (self.input_seq_length + self.target_seq_length))
            valid_counter += int(len(valid_frame_data) / (self.input_seq_length + self.target_seq_length))

            # maximum number of agents in one dataset
            if max_num_agents > max_agents:
                max_agents = max_num_agents

        # Calculate the number of batches
        self.num_batches = int(counter / self.batch_size)
        self.valid_num_batches = int(valid_counter / self.batch_size)
        # On an average, we need twice the number of batches to cover the data
        # due to randomization introduced
        print('Total number of training batches:', self.num_batches * 2)
        print('Total number of validation batches:', self.valid_num_batches)
        print('Maximum number of agents = ', max_agents)

    def next_batch(self, random_update=True):
        '''
        Function to get the next batch of points
        '''
        # Source data
        x_batch = []
        # Target data
        y_batch = []
        # Dataset data
        d = []
        # Image data
        image = []
        image_size = []
        # Iteration index
        i = 0

        while i < self.batch_size:

            # Extract the frame data of the current dataset
            frame_data = self.data[self.dataset_pointer]
            # Extract the image data of the current dataset
            self.image = self.image_data[self.dataset_pointer]
            self.image_dims = self.image_size[self.dataset_pointer]
            # Get the frame pointer for the current dataset
            idx = self.frame_pointer

            # While there is still seq_length number of frames left in the current dataset
            if idx + self.input_seq_length + self.target_seq_length < len(frame_data):

                # All the data in the source sequence
                seq_source_frame_data = frame_data[idx:idx + self.input_seq_length]
                # All the data in the target sequence
                seq_target_frame_data = frame_data[
                                        idx + self.input_seq_length:idx + self.input_seq_length + self.target_seq_length]

                # Number of unique agents in both the source and the target sequence
                agent_id_source = np.unique(seq_source_frame_data[0, :, 0])
                agent_id_target = np.unique(seq_target_frame_data[self.target_seq_length - 1, :, 0])
                # Number of agents which are present in both sequences
                agent_id_both = np.intersect1d(agent_id_source, agent_id_target)

                # only IDs != 0
                agent_id_both = agent_id_both[agent_id_both > 0]
                num_unique_agents = len(agent_id_both)

                if num_unique_agents == 0:
                    # Advance the frame pointer to a random point
                    if random_update:
                        self.frame_pointer += random.randint(1, self.input_seq_length + self.target_seq_length + 10)
                    else:
                        if self.infer:
                            self.frame_pointer += 1
                        else:
                            self.frame_pointer += self.input_seq_length + self.target_seq_length + 10
                    continue

                # initialize a numpy array for the source data only containing valid IDs
                source_data = np.zeros((self.input_seq_length, self.max_agents, 3))
                # initialize a numpy array for the target data only containing valid IDs
                target_data = np.zeros((self.target_seq_length, self.max_agents, 3))

                for frame in range(self.input_seq_length):

                    curr_frame_data = seq_source_frame_data[frame, :]

                    for agent in range(num_unique_agents):
                        agent_id = agent_id_both[agent]
                        agent_data = curr_frame_data[curr_frame_data[:, 0] == agent_id, :]

                        if agent_data.shape[0] != 0:
                            source_data[frame, agent, :] = agent_data

                for frame in range(self.target_seq_length):

                    curr_frame_data = seq_target_frame_data[frame, :]

                    for agent in range(num_unique_agents):
                        agent_id = agent_id_both[agent]
                        agent_data = curr_frame_data[curr_frame_data[:, 0] == agent_id, :]

                        if agent_data.shape[0] != 0:
                            target_data[frame, agent, :] = agent_data

                source_data = np.transpose(source_data, [1, 0, 2])
                target_data = np.transpose(target_data, [1, 0, 2])

                if self.reduce_agents > 0:
                    # Reduce number of Agents in the Sequence to reduce_peds
                    source_data_reduced = source_data[0:self.reduce_agents, :, :]
                    target_data_reduced = target_data[0:self.reduce_agents, :, :]

                    x_batch.append(source_data_reduced)
                    y_batch.append(target_data_reduced)
                else:
                    x_batch.append(source_data)
                    y_batch.append(target_data)

                # Advance the frame pointer to a random point
                if random_update:
                    self.frame_pointer += random.randint(1, self.input_seq_length + self.target_seq_length + 10)
                else:
                    if self.infer:
                        self.frame_pointer += 1
                    else:
                        self.frame_pointer += self.input_seq_length + self.target_seq_length

                d.append(self.dataset_pointer)
                image.append(self.image)
                image_size.append(self.image_dims)
                i += 1

            else:
                # Not enough frames left
                # Increment the dataset pointer and set the frame_pointer to zero
                self.tick_batch_pointer(valid=False)

        return x_batch, y_batch, d, image, image_size

    def next_valid_batch(self, random_update=True):
        '''
        Function to get the next batch of points
        '''
        # Source data
        x_batch = []
        # Target data
        y_batch = []
        # Dataset data
        d = []
        # Image data
        image = []
        image_size = []
        # Iteration index
        i = 0

        while i < self.batch_size:

            # Extract the frame data of the current dataset
            frame_data = self.valid_data[self.valid_dataset_pointer]
            # Extract the image data of the current dataset
            self.image = self.image_data[self.valid_dataset_pointer]
            self.image_size = self.image_size[self.valid_dataset_pointer]
            # Get the frame pointer for the current dataset
            idx = self.valid_frame_pointer

            # While there is still seq_length number of frames left in the current dataset
            if idx + self.input_seq_length + self.target_seq_length < len(frame_data):

                # All the data in the source sequence
                seq_source_frame_data = frame_data[idx:idx + self.input_seq_length]
                # All the data in the target sequence
                seq_target_frame_data = frame_data[
                                        idx + self.input_seq_length:idx + self.input_seq_length + self.target_seq_length]

                # Number of unique agents in both the source and the target sequence
                agent_id_source = np.unique(seq_source_frame_data[0, :, 0])
                agent_id_target = np.unique(seq_target_frame_data[self.target_seq_length - 1, :, 0])
                # Number of agents present in all frames
                agent_id_test = set(seq_source_frame_data[:, 0, 0])
                for s in seq_source_frame_data[1:, 0, 0]:
                    agent_id_test.intersection_update(s)
                print(agent_id_test)
                # Number of agents which are present in both sequences
                agent_id_both = np.intersect1d(agent_id_source, agent_id_target)

                # only IDs != 0
                agent_id_both = agent_id_both[agent_id_both > 0]
                num_unique_agents = len(agent_id_both)

                if num_unique_agents == 0:
                    # Advance the frame pointer to a random point
                    if random_update:
                        self.valid_frame_pointer += random.randint(1,
                                                                   self.input_seq_length + self.target_seq_length + 10)
                    else:
                        self.valid_frame_pointer += self.input_seq_length + self.target_seq_length + 10
                    continue

                # initialize a numpy array for the source data only containing valid IDs
                source_data = np.zeros((self.input_seq_length, self.max_agents, 3))
                # initialize a numpy array for the target data only containing valid IDs
                target_data = np.zeros((self.target_seq_length, self.max_agents, 3))

                for frame in range(self.input_seq_length):

                    curr_frame_data = seq_source_frame_data[frame, :]

                    for agent in range(num_unique_agents):
                        agent_id = agent_id_both[agent]
                        agent_data = curr_frame_data[curr_frame_data[:, 0] == agent_id, :]

                        if agent_data.shape[0] != 0:
                            source_data[frame, agent, :] = agent_data

                for frame in range(self.target_seq_length):

                    curr_frame_data = seq_target_frame_data[frame, :]

                    for agent in range(num_unique_agents):
                        agent_id = agent_id_both[agent]
                        agent_data = curr_frame_data[curr_frame_data[:, 0] == agent_id, :]

                        if agent_data.shape[0] != 0:
                            target_data[frame, agent, :] = agent_data

                source_data = np.transpose(source_data, [1, 0, 2])
                target_data = np.transpose(target_data, [1, 0, 2])

                if self.reduce_agents > 0:
                    # Reduce number of Agents in the Sequence to reduce_peds
                    source_data_reduced = source_data[0:self.reduce_agents, :, :]
                    target_data_reduced = target_data[0:self.reduce_agents, :, :]

                    x_batch.append(source_data_reduced)
                    y_batch.append(target_data_reduced)
                else:
                    x_batch.append(source_data)
                    y_batch.append(target_data)

                # Advance the frame pointer to a random point
                if random_update:
                    self.valid_frame_pointer += random.randint(1, self.input_seq_length + self.target_seq_length)
                else:
                    self.valid_frame_pointer += self.input_seq_length + self.target_seq_length

                d.append(self.dataset_pointer)
                image.append(self.image)
                image_size.append(self.image_dims)
                i += 1

            else:
                # Not enough frames left
                # Increment the dataset pointer and set the frame_pointer to zero
                self.tick_batch_pointer(valid=True)

        return x_batch, y_batch, d, image, image_size

    def tick_batch_pointer(self, valid=False):
        '''
        Advance the dataset pointer
        '''
        if not valid:
            # Go to the next dataset
            self.dataset_pointer += 1
            # Set the frame pointer to zero for the current dataset
            self.frame_pointer = 0
            # If all datasets are done, then go to the first one again
            if self.dataset_pointer >= len(self.data):
                self.dataset_pointer = 0
        else:
            # Go to the next dataset
            self.valid_dataset_pointer += 1
            # Set the frame pointer to zero for the current dataset
            self.valid_frame_pointer = 0
            # If all datasets are done, then go to the first one again
            if self.valid_dataset_pointer >= len(self.valid_data):
                self.valid_dataset_pointer = 0

    def reset_batch_pointer(self, valid=False):
        '''
        Reset all pointers
        '''
        if not valid:
            # Go to the first frame of the first dataset
            self.dataset_pointer = 0
            self.frame_pointer = 0
        else:
            self.valid_dataset_pointer = 0
            self.valid_frame_pointer = 0
