import pickle


def load_pickle(pickle_path):
    try:
        with open(pickle_path, 'rb') as pickle_file:
            return pickle.load(pickle_file)
    except OSError:
        raise


def write_pickle(output_path, data):
    try:
        with open(output_path, 'wb') as pickle_file:
            pickle.dump(data, pickle_file, protocol=2)
    except OSError:
        raise
