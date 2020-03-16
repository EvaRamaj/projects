import os
import pickle
import numpy as np



# The data loader class that loads data from the datasets, considering each frame as a datapoint
# and a sequence of consecutive frames as the sequence.
class DataLoader_ETH():

    def __init__(self, forcePreProcess=False):
        '''
        Initialiser function for the SocialDataLoader class
        params:
        args: Arguments for the model
        forcePreProcess : Flag to forcefully preprocess the data again from the .txt files
        infer: Flag to process the data in inference mode
        '''
        # Instead of every frame only every nth frame is considered
        # self.skip = 3

        # List containing the training data directories where the raw data resides
        self.train_dir = './data/arxiepiskopi'
        # Data directory where the pre-processed pickle file resides
        self.data_dir = './data'
        train_file = os.path.join(self.data_dir, "train_trajectories_arxiepiskopi.cpkl")

        # If the train file doesn't exist or forcePreProcess is true
        if not (os.path.exists(train_file)) or forcePreProcess:
            print("Creating pre-processed training data from raw data")
            # Preprocess the data from the txt files of the datasets
            # Note that this data is processed in frames
            self.frame_preprocess(self.train_dir, train_file)

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
        # Index of the current dataset
        dataset_index = 0

        file_path = os.path.join(data_dirs, 'arxiepiskopi1.txt')
        # Load the data from the txt file
        data = np.genfromtxt(file_path, delimiter=' ')
        # print("I read the data", data)

        # Frame IDs of the frames in the current dataset
        frame_list = np.unique(data[:, 0]).tolist()
        # Number of frames
        num_frames = len(np.asarray(frame_list))
        # print("this is the frame list",frame_list)
        # print("this is the num_frames", num_frames)
        # Initialize the numpy array for the current dataset
        all_frame_data.append(np.zeros((num_frames, 24, 3)))  # just now for testing I have to change the agents number

        curr_frame = 0

        for frame in frame_list:
            # Extract all data of the current frame
            data_in_frame = data[data[:, 0] == frame, :]
            # print("this is the data in frame", data_in_frame)
            # Extract agents list from the data in current frame
            agents_list = data_in_frame[:, 1].tolist()
            # print("this is the agents list", agents_list)

            # Add number of agents in the current frame to the stored data
            # [dataset_index].append(len(agents_list))

            # Initialize the row of the numpy array
            agents_with_pos = []

            # For each agent in the current frame
            for agent in agents_list:
                # Extract their x and y positions defining the bounding box
                current_x = data_in_frame[data_in_frame[:, 1] == agent, 2][0]
                current_y = data_in_frame[data_in_frame[:, 1] == agent, 3][0]
                # Add their pedID, x, y to the row of the numpy array
                agents_with_pos.append([agent, current_x, current_y])

            all_frame_data[dataset_index][curr_frame, 0:len(agents_list), :] = np.array(agents_with_pos)

            # Increment of the frame index
            curr_frame += 1
        print("this is the all_frame_data", all_frame_data)
        # Save the tuple (all_frame_data, frame_list_data, num_agents_data) in the pickle file
        f = open(data_file, "wb")
        pickle.dump((all_frame_data, frame_list), f,
                    protocol=2)
        f.close()

