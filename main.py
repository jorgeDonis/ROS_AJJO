from typing import ChainMap
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
IMG_HEIGHT = 64

BATCH_SIZE = 8
EPOCHS = 30

def preprocess_img(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = np.delete(img, slice(1, 100), 0)
    img = np.delete(img, slice(290, img.shape[0]), 0)
    img = cv2.resize(img, (IMG_WIDTH, IMG_HEIGHT), interpolation=cv2.INTER_LANCZOS4)
    img = np.reshape(img, (img.shape[0], img.shape[1], 1))
    img = img / 255
    # img = img.astype(np.float32) / 255.0
    return img

def get_model():
    inputs = tf.keras.layers.Input(shape=(IMG_HEIGHT, IMG_WIDTH, 1), name='opencv_original_imgs')
    x = tf.keras.layers.Conv2D(9, (4, 4), padding='same') (inputs)
    x = tf.keras.layers.MaxPool2D(pool_size=(3, 3)) (x)
    x = tf.keras.layers.Conv2D(3, (3, 3)) (x)
    x = tf.keras.layers.Conv2D(3, (3, 3)) (x)
    x = tf.keras.layers.Conv2D(5, (2, 2)) (x)
    x = tf.keras.layers.MaxPool2D(pool_size=(2, 2)) (x)
    x = tf.keras.layers.Conv2D(5, (2, 2), padding='same') (x)
    x = tf.keras.layers.MaxPool2D(pool_size=(2, 2)) (x)
    x = tf.keras.layers.Flatten() (x)
    x = tf.keras.layers.Dense(64) (x)
    x = tf.keras.layers.Dense(32) (x)
    outputs = tf.keras.layers.Dense(3, activation='relu') (x)
    model = tf.keras.Model(inputs, outputs)
    model.compile(optimizer='adam', loss='categorical_crossentropy',
                  metrics=['accuracy'])

    return model


def cnn_model():

    model = tf.keras.Sequential()

    model.add(layers.Conv2D(6, (5, 5)))
    model.add(layers.Activation('relu'))
    model.add(layers.MaxPooling2D(pool_size=(2, 2)))

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

X, Y = load_dataset()
X, Y = shuffle_ds(X, Y)
X, Y = balance_ds(X, Y)

# for i in range(0, len(X)):
#     show_img(X[i], Y[i])

# model = get_model()
model = cnn_model()
model.fit(X, Y, batch_size = BATCH_SIZE, epochs = EPOCHS)
model.save('tf_model_driving.h5', include_optimizer=False)
