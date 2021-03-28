from os import system
import numpy as np
import tensorflow as tf
import glob
import re
import cv2
import matplotlib.pyplot as plt
import random

from tensorflow.keras.models import Sequential
from tensorflow.keras import layers

IMG_WIDTH = 128
IMG_HEIGHT = 128

BATCH_SIZE = 8
EPOCHS = 20

def preprocess_img(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = np.delete(img, slice(1, 90), 0)
    img = np.delete(img, slice(300, img.shape[0]), 0)
    img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_LANCZOS4)
    img = np.reshape(img, (img.shape[0], img.shape[1], 1))
    img = img / 255
    return img


def cnn_model():

    model = tf.keras.Sequential()

    model.add(layers.Input(shape=(IMG_HEIGHT, IMG_WIDTH, 1)))
    model.add(layers.Conv2D(6, (5, 5)))
    model.add(layers.Activation('relu'))
    model.add(layers.MaxPooling2D(pool_size=(4, 4)))

    model.add(layers.Conv2D(16, (5, 5)))
    model.add(layers.Activation('relu'))
    model.add(layers.MaxPooling2D(pool_size=(2, 2)))

    model.add(layers.Flatten())

    model.add(layers.Dense(120))
    model.add(layers.Activation('relu'))

    model.add(layers.Dense(84))
    model.add(layers.Activation('relu'))

    model.add(layers.Dense(3))
    model.add(layers.Activation('softmax'))

    model.compile(optimizer='adam', loss='categorical_crossentropy',
                  metrics=['accuracy'])

    return model

def load_dataset():
    X = []
    Y = []
    for filename in (glob.glob("./dataset/*")):
        regex = r"([A-Z_]+)_\d+\.jpg"
        category = re.findall(regex, filename, re.MULTILINE)[0]
        img = cv2.imread(filename)
        X.append(img)
        if (category == "NO_ACTION"):
            Y.append(np.array([1, 0, 0]))
        elif (category == "STEER_LEFT"):
            Y.append(np.array([0, 1, 0]))
        elif (category == "STEER_RIGHT"):
            Y.append(np.array([0, 0, 1]))
    X = [preprocess_img(x) for x in X]
    return np.array(X), np.array(Y)

def balance_ds(X, Y):
    left_indices = [i for i, x in enumerate(Y) if np.array_equal(x, [0, 1, 0])]
    right_indices = [i for i, x in enumerate(Y) if np.array_equal(x, [0, 0, 1])]
    no_action_indices = [i for i, x in enumerate(Y) if np.array_equal(x, [1, 0, 0])]
    final_samples_per_class = min(len(left_indices), len(right_indices))
    final_samples_per_class = min(final_samples_per_class, len(no_action_indices))
    new_X = []
    new_Y = []
    for i in range(0, round(final_samples_per_class / 3)):
        new_X.append(X[left_indices[i]])
        new_Y.append(Y[left_indices[i]])
        new_X.append(X[right_indices[i]])
        new_Y.append(Y[right_indices[i]])
        new_X.append(X[no_action_indices[i]])
        new_Y.append(Y[no_action_indices[i]])

    return np.array(new_X), np.array(new_Y)

def shuffle_ds(X, Y):
    new_X = []
    new_Y = []
    indices = [i for i, x in enumerate(X)]
    random.shuffle(indices)
    new_X = [X[i] for i in indices]
    new_Y = [Y[i] for i in indices]
    return np.array(new_X), np.array(new_Y)

def show_img(img, label):
    plt.imshow(img, cmap='gray')
    if (label[0] == 1):
        plt.title("NO ACTION")
    elif (label[1] == 1):
        plt.title("STEER_LEFT")
    elif (label[2] == 1):
        plt.title("STEER_RIGHT")
    plt.show()

def generator():
    X = []
    Y = []
    i = 0
    for filename in (glob.glob("./dataset/*")):
        regex = r"([A-Z_]+)_\d+\.jpg"
        category = re.findall(regex, filename, re.MULTILINE)[0]
        img = cv2.imread(filename)
        X.append(img)
        if (category == "NO_ACTION"):
            Y.append(np.array([1, 0, 0]))
        elif (category == "STEER_LEFT"):
            Y.append(np.array([0, 1, 0]))
        elif (category == "STEER_RIGHT"):
            Y.append(np.array([0, 0, 1]))
        i += 1
        if i == BATCH_SIZE:
            X = [preprocess_img(x) for x in X]
            X, Y = shuffle_ds(X, Y)
            X, Y = balance_ds(X, Y)
            if (len(Y) != 0):
                yield np.array(X), np.array(Y)
            X = []
            Y = []
            i = 0

def balance_ds_directory(directory):
    no_action_filenames = []
    steer_left_filenames = []
    steer_right_filenames = []
    for filename in (glob.glob("./dataset/*")):
        regex = r"([A-Z_]+)_\d+\.jpg"
        category = re.findall(regex, filename, re.MULTILINE)[0]
        if (category == "NO_ACTION"):
            no_action_filenames.append(filename)
        elif (category == "STEER_LEFT"):
            steer_left_filenames.append(filename)
        elif (category == "STEER_RIGHT"):
            steer_right_filenames.append(filename)
    samples_per_cat = min(len(no_action_filenames), len(steer_left_filenames))
    samples_per_cat = min(len(steer_right_filenames), samples_per_cat)
    random.shuffle(no_action_filenames)
    random.shuffle(steer_left_filenames)
    random.shuffle(steer_right_filenames)
    for i in range(0, len(no_action_filenames) - samples_per_cat):
        system("rm " + no_action_filenames[i])
    for i in range(0, len(steer_left_filenames) - samples_per_cat):
        system("rm " + steer_left_filenames[i])
    for i in range(0, len(steer_right_filenames) - samples_per_cat):
        system("rm " + steer_right_filenames[i])



# balance_ds_directory("dataset")
X, Y = load_dataset()
# X, Y = shuffle_ds(X, Y)
# X, Y = balance_ds(X, Y)

# for i in range(0, len(X)):
#     show_img(X[i], Y[i])

model = cnn_model()
# print(model.summary())

# gen = generator()

model.fit(X, Y, batch_size = BATCH_SIZE, epochs = EPOCHS)
model.save('tf_model_driving.h5', include_optimizer=False)
