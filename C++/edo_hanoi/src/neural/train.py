import csv
import numpy as np
import keras
from keras.utils import plot_model
import os.path
import cv2

import network

def load_model(model_path):
    load_model = False
    if os.path.isfile(model_path):
        ans = raw_input("Existing model %s found. Do you want to load it? [Y/n]: " % model_path)
        load_model = ans != "n"

    if load_model:
        print("Loading model %s" % model_path)
        model = keras.models.load_model(model_path)
    else:
        print("Creating new model.")
        model = network.create_model()
    plot_model(model, to_file='model.png')
    return model


def save_model(model_path, model):
    overwrite_model = False
    while os.path.isfile(model_path):
        ans = raw_input("Model %s already exists. Do you want to overwrite it? [y/N]: " % model_path)
        overwrite_model = ans == "y"

        if overwrite_model:
            print("Overwriting model.")
            model.save(model_path)
            return 
        else:
            model_path = raw_input("Enter alternative model path: ")
    print("Saved model %s" % model_path)
    model.save(model_path)


if __name__ == "__main__":
    batch_size = 16
    model_path = "cnn.model"

    model = load_model(model_path)
    model.summary()

    opt = keras.optimizers.Adam(lr=0.001)
    model.compile(
        loss="categorical_crossentropy",          
        optimizer=opt,
        metrics=["accuracy"])


    x_train,y_train = network.load_data("train.csv")
    x_test,y_test = network.load_data("test.csv")
    
    acc_history = []
    for i in range(10):
        model.fit(x_train, y_train, epochs=5, batch_size=batch_size, shuffle=True)

        res = model.evaluate(x_test, y_test, batch_size=batch_size)
        accs = np.array(res[-8:])
        acc = np.average(accs)
        print("Evaluation on test set:")
        print(accs)
        print(acc)
        acc_history.append(acc)
        
        model.save("%s_%s_.chpkt" % (model_path,i))
        
    print(acc_history)

    save_model(model_path, model)

