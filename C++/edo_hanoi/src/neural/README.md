# e.DO Hanoi Challenge - Neural Networks

## Download training data and neural network

The data and the pre-trained model are aready part of the repository. Alternatively, you can download the imgs.zip file from the [google drive folder](https://drive.google.com/open?id=1s4GazkEnlCFoYU8w_XlNFdhM6rkA52s5) and put the images into `edo_hanoi/src/neural/imgs`. 

Also, if they aren't present already (which they should be if on the latest commit), copy the `train.csv` and `test.csv` files into `edo_hanoi/src/neural`. These files contain the list of images coupled with the ground truth. You can exchange lines between the two files to choose what data to use for training and what to use for testing. All available data is listed in `groundtruth.csv`.

The neural detector expects the model file to be `edo_hanoi/src/neural/cnn.model`. For this to work, you need to start the detector in `edo_hanoi/src`. When starting it somewhere else, please adjust the model path at the start of the file `neural_detector.py`.

## Training the network

To train it yourself, go into the `edo_hanoi/src/neural` folder and set the configuration parameters and the network model in `network.py`. Further hyperparameters for the training can be set in `train.py`.

To start training, simply run `python2 train.py`. By default, the training script will save the model in `cnn.model`, and will also create checkpoints along the way.

## Testing and using the network

To test the network, first make sure that the path to the model file is set correctly in `neural_detector.py`.

A simple detector test can be start by running `python2 detector_test.py --no-sim`.

Other than that, the neural detector can be used/called to detect the hanoi state in a given image.
