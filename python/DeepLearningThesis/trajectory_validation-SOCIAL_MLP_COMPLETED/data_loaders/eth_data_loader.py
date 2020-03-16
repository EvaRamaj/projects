import os
import pickle
import numpy as np
import random
from statistics import Statistics



# The data loader class that loads data from the datasets, considering each frame as a datapoint
# and a sequence of consecutive frames as the sequence.
class DataLoader_ETH():

    def __init__(self, args, forcePreProcess=False, infer=False):
        '''
        Initialiser function for the SocialDataLoader class
        params:
        args: Arguments for the model
        forcePreProcess : Flag to forcefully preprocess the data again from the .txt files
        infer: Flag to process the data in inference mode
        '''
        # Instead of every frame only every nth frame is considered
        # self.skip = 3
        self.infer = infer
        # List containing the training data directories where the raw data resides
        # self.train_dirs = ['./data/eth/biwi', './data/eth/crowds/arxiepiskopi', './data/eth/crowds/crowds_zara02',
        #                   './data/eth/crowds/crowds_zara03', './data/eth/crowds/students001',
        #                   './data/eth/crowds/students003']
        self.train_dirs = [
                           '../data/eth/stanford/bookstore_0.txt',
                           '../data/eth/stanford/bookstore_1.txt',
                           '../data/eth/stanford/bookstore_2.txt', '../data/eth/stanford/bookstore_3.txt',
                           '../data/eth/stanford/coupa_3.txt', '../data/eth/stanford/deathCircle_0.txt',
                           '../data/eth/stanford/deathCircle_1.txt', '../data/eth/stanford/deathCircle_2.txt',
                           '../data/eth/stanford/deathCircle_3.txt', '../data/eth/stanford/gates_0.txt',
                           '../data/eth/stanford/gates_1.txt', '../data/eth/stanford/gates_3.txt',
                           '../data/eth/stanford/gates_4.txt', '../data/eth/stanford/gates_5.txt',
                           '../data/eth/stanford/gates_6.txt', '../data/eth/stanford/gates_7.txt',
                           '../data/eth/stanford/gates_8.txt', '../data/eth/stanford/hyang_4.txt',
                           '../data/eth/stanford/hyang_5.txt', '../data/eth/stanford/hyang_6.txt',
                           '../data/eth/stanford/hyang_7.txt', '../data/eth/stanford/hyang_9.txt',
                           '../data/eth/stanford/nexus_0.txt', '../data/eth/stanford/nexus_1.txt',
                           '../data/eth/stanford/nexus_2.txt', '../data/eth/stanford/nexus_3.txt',
                           '../data/eth/stanford/nexus_4.txt', '../data/eth/stanford/nexus_7.txt',
                           '../data/eth/stanford/nexus_8.txt', '../data/eth/stanford/nexus_9.txt'
                          ]
        # Instead of every frame only every nth frame is considered
        # self.skip = 3

        if self.infer:
            # In inference mode
            self.batch_size = 1
        else:
            # In training mode
            self.batch_size = args.batch_size

        # Data directory where the pre-processed pickle file resides
        self.data_dir = '../data/'

        # Maximum number of agents in a single frame (Number obtained by checking the datasets)
        self.max_agents = args.max_agents
        # Reduce training data to reduce_agents number of agents to speed up training
        self.reduce_agents = args.reduce_agents

        # Store the arguments
        self.input_seq_length = args.input_seq_length
        self.target_seq_length = args.target_seq_length

        # Validation ratio
        self.val_fraction = 0.2

        train_file = os.path.join(self.data_dir, "train_trajectories.cpkl")
        # Just for now! I have to change this
        test_file = os.path.join(self.data_dir, "test_trajectories.cpkl")

        # If the train file doesn't exist or forcePreProcess is true
        if not (os.path.exists(train_file)) or forcePreProcess:
            print("Creating pre-processed training data from raw data")
            # Preprocess the data from the txt files of the datasets
            # Note that this data is processed in frames
            self.frame_preprocess(self.train_dirs, train_file)

        # Load the processed data from the pickle file
        if self.infer:
            self.load_preprocessed(test_file)
        else:
            self.load_preprocessed(train_file)

        # Reset all the data pointers of the dataloader object
        self.reset_batch_pointer(valid=False)
        self.reset_batch_pointer(valid=True)

    def frame_preprocess(self, data_dirs, data_file):
        '''
        Function that will pre-process the annotations.txt files of each dataset
        params:
        data_dirs : List of directories where raw data resides
        data_file : The file into which all the pre-processed data needs to be stored
        '''

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
        # image_data = []
        # image_size = []
        # Index of the current dataset
        dataset_index = 0

        # For each dataset
        for directory in data_dirs:
            # file_path = os.path.join(directory, 'annotations.txt')
            # Load the data from the txt file
            data = np.genfromtxt(directory, delimiter=' ')
            # print("I read the data", data)

            # Frame IDs of the frames in the current dataset
            frame_list = np.unique(data[:, 0]).tolist()

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
            all_frame_data.append(np.zeros((num_frames - valid_num_frames + 1, self.max_agents, 3)))
            # all_frame_data.append(np.zeros((num_frames, 415, 3)))
            valid_frame_data.append(np.zeros((valid_num_frames, self.max_agents, 3)))

            # just now for testing I have to change the agents number

            curr_frame = 0

            for frame in frame_list:
                # Extract all data of the current frame
                data_in_frame = data[data[:, 0] == frame, :]
                # print("this is the data in frame", data_in_frame)
                # Extract agents list from the data in current frame
                agents_list = data_in_frame[:, 1].tolist()
                # print("this is the agents list", agents_list)

                # Add number of agents in the current frame to the stored data
                num_agents_data[dataset_index].append(len(agents_list))

                # Initialize the row of the numpy array
                agents_with_pos = []

                # For each agent in the current frame
                for agent in agents_list:
                    # Extract their x and y positions defining the bounding box
                    current_x = data_in_frame[data_in_frame[:, 1] == agent, 2][0]
                    current_y = data_in_frame[data_in_frame[:, 1] == agent, 3][0]
                    # Add their pedID, x, y to the row of the numpy array
                    agents_with_pos.append([agent, current_x, current_y])

                if (curr_frame >= valid_num_frames) or self.infer:
                    # At inference time, no validation data
                    # Add the details of all the agents in the current frame to all_frame_data
                    all_frame_data[dataset_index][curr_frame - valid_num_frames, 0:len(agents_list), :] = np.array(agents_with_pos)
                else:
                    valid_frame_data[dataset_index][curr_frame, 0:len(agents_list), :] = np.array(agents_with_pos)

                # Increment of the frame index
                curr_frame += 1

            # Increase the dataset index
            dataset_index += 1

        print("this is the all_frame_data", all_frame_data)
        # Save the tuple (all_frame_data, frame_list_data, num_agents_data) in the pickle file
        f = open(data_file, "wb")
        pickle.dump((all_frame_data, valid_frame_data, num_agents_data), f,
                    protocol=2)
        f.close()

    def load_preprocessed(self, data_file):
        '''
        Function to load the pre-processed data into the DataLoader object
        params:
        data_file : the path to the pickled data file
        '''
        # Load data from the pickled file
        f = open(data_file, 'rb')
        self.raw_data = pickle.load(f)
        f.close()
        # Get all the data from the pickle file
        self.data = self.raw_data[0]
        self.valid_data = self.raw_data[1]
        # self.frame_list = self.raw_data[2]
        self.num_agents_list = self.raw_data[2]
        # print(self.num_agents_list)
        # self.valid_data = self.raw_data[3] # current dataset
        # self.image_data = self.raw_data[4]
        # self.image_size = self.raw_data[5]

        counter = 0
        valid_counter = 0
        max_agents = 0
        # For each frame
        for frame in range(len(self.data)):
            # get the frame data for the current dataset
            # all_frame_data = self.data # I have only one dataset, lets iterate through frames
            #valid_frame_data = self.valid_data[dataset]
            max_num_agents = np.amax(self.num_agents_list[frame], axis=0)

            print("look here", len(self.data[frame][1]))
            print("Frame {}: training data   = {} , maximum number of agents = {}".format(frame, len(self.data),
                                                                                            max_num_agents))
            #print("Dataset {}: validation data = {}".format(dataset, len(valid_frame_data)))

            # Increment the counter with the number of sequences in the current dataset
            counter += int(len(self.data[frame]) / (self.input_seq_length + self.target_seq_length))
            valid_counter += int(len(self.valid_data[frame]) / (self.input_seq_length + self.target_seq_length))

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
        num_agents = []
        # Iteration index
        i = 0

        while i < self.batch_size:

            # Extract the frame data of the current dataset
            frame_data = self.data[self.dataset_pointer]
            # Extract the image data of the current dataset
            # self.image = self.image_data[self.dataset_pointer]
            # self.image_dims = self.image_size[self.dataset_pointer]
            # Get the frame pointer for the current dataset
            idx = self.frame_pointer
            # While there is still seq_length number of frames left in the current dataset
            if idx + self.input_seq_length + self.target_seq_length < len(frame_data):

                # All the data in the source sequence
                seq_source_frame_data = frame_data[idx:idx + self.input_seq_length]
                # All the data in the target sequence
                seq_target_frame_data = frame_data[idx + self.input_seq_length:idx + self.input_seq_length + self.target_seq_length]

                # Number of unique agents in both the source and the target sequence
                agent_id_source = np.unique(seq_source_frame_data[0, :, 0])
                agent_id_target = np.unique(seq_target_frame_data[self.target_seq_length-1, :, 0])
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

                num_agents.append(num_unique_agents)

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

                trajectories = np.concatenate([source_data, target_data], axis=1)
                for agent in range(num_unique_agents):
                    trajectories[agent, :, 1:3] = np.asarray(Statistics.trans_global_local(trajectories[agent, :, 1:3]))
                    source_data[agent] = trajectories[agent, :self.input_seq_length, :]
                    target_data[agent] = trajectories[agent, self.input_seq_length:(self.input_seq_length +
                                                                                    self.target_seq_length), :]

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
                # image.append(self.image)
                # image_size.append(self.image_dims)
                i += 1

            else:
                # Not enough frames left
                # Increment the dataset pointer and set the frame_pointer to zero
                self.tick_batch_pointer(valid=False)

        return x_batch, y_batch, d, image, image_size, num_agents

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
        num_agents = []
        # Iteration index
        i = 0

        while i < self.batch_size:

            # Extract the frame data of the current dataset
            frame_data = self.valid_data[self.valid_dataset_pointer]
            # Extract the image data of the current dataset
            # self.image = self.image_data[self.valid_dataset_pointer]
            # self.image_size = self.image_size[self.valid_dataset_pointer]
            # Get the frame pointer for the current dataset
            idx = self.valid_frame_pointer

            # While there is still seq_length number of frames left in the current dataset
            if idx + self.input_seq_length + self.target_seq_length < len(frame_data):

                # All the data in the source sequence
                seq_source_frame_data = frame_data[idx:idx + self.input_seq_length]
                # All the data in the target sequence
                seq_target_frame_data = frame_data[idx + self.input_seq_length:idx + self.input_seq_length + self.target_seq_length]

                # Number of unique agents in both the source and the target sequence
                agent_id_source = np.unique(seq_source_frame_data[0, :, 0])
                agent_id_target = np.unique(seq_target_frame_data[self.target_seq_length-1, :, 0])
                # Number of agents present in all frames
                # agent_id_test = set(seq_source_frame_data[:, 0, 0])
                # for s in seq_source_frame_data[1:, 0, 0]:
                #     agent_id_test.intersection_update(s)
                # print(agent_id_test)
                # Number of agents which are present in both sequences
                agent_id_both = np.intersect1d(agent_id_source, agent_id_target)

                # only IDs != 0
                agent_id_both = agent_id_both[agent_id_both > 0]
                num_unique_agents = len(agent_id_both)

                if num_unique_agents == 0:
                    # Advance the frame pointer to a random point
                    if random_update:
                        self.valid_frame_pointer += random.randint(1, self.input_seq_length + self.target_seq_length + 10)
                    else:
                        self.valid_frame_pointer += self.input_seq_length + self.target_seq_length + 10
                    continue

                num_agents.append(num_unique_agents)
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

                trajectories = np.concatenate([source_data, target_data], axis=1)
                for agent in range(num_unique_agents):
                    trajectories[agent, :, 1:3] = np.asarray(Statistics.trans_global_local(trajectories[agent, :, 1:3]))
                    source_data[agent] = trajectories[agent, :self.input_seq_length, :]
                    target_data[agent] = trajectories[agent, self.input_seq_length:(self.input_seq_length +
                                                                                    self.target_seq_length), :]

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
                # image.append(self.image)
                # image_size.append(self.image_dims)
                i += 1

            else:
                # Not enough frames left
                # Increment the dataset pointer and set the frame_pointer to zero
                self.tick_batch_pointer(valid=True)

        return x_batch, y_batch, d, image, image_size, num_agents

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

