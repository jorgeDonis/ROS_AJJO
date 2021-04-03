from os import system
import os
import numpy as np
import tensorflow as tf
import glob
import re
import cv2
import matplotlib.pyplot as plt
import random
import math

from sklearn.utils.class_weight import compute_class_weight
from tensorflow.keras.models import Sequential
from tensorflow.keras import layers, Model

IMG_WIDTH = 222
IMG_HEIGHT = 222

TEST_PCTG = 0.2

BATCH_SIZE = 64
EPOCHS = 40

REGEX = r"([A-Z]+)_([A-Z]+)_(.*)_(.*)_(.*)_(.*)\.jpg"

CATEGORIES = ['NO_ACTION', 'STEER_LEFT', 'STEER_RIGHT']

def process_path(path):
    parts = tf.strings.split(path, os.path.sep)[-1]
    parts = tf.strings.split(parts, "_")
    category = tf.strings.join([parts[0], "_", parts[1]])
    one_hot = category == CATEGORIES
    
    img = tf.io.read_file(path)
    img = tf.image.decode_jpeg(img, channels=1)
    img = tf.image.resize(img, [IMG_HEIGHT, IMG_WIDTH], method='bilinear')
    img = tf.math.divide(img, 255)

    odom_x = tf.strings.to_number(parts[2])
    odom_y = tf.strings.to_number(parts[3])
    odom_x = tf.math.divide(odom_x, 20)
    odom_y = tf.math.divide(odom_y, 20)
    odom_yaw = tf.strings.to_number(parts[4])
    odom_y = tf.math.divide(odom_yaw, math.pi)

    input = {
        'image' : img,
        'odometry' : tf.stack([
            odom_x, odom_y, odom_yaw
        ])
    }

    return input, one_hot

def configure_for_performance(ds):
  ds = ds.cache()
  ds = ds.shuffle(buffer_size=1000)
  ds = ds.batch(BATCH_SIZE)
  ds = ds.prefetch(buffer_size=1000)
  return ds

def load_datsets():
    image_count = len(glob.glob("./dataset/*.jpg"))
    list_ds = tf.data.Dataset.list_files("./dataset/*.jpg", shuffle=False)
    list_ds = list_ds.shuffle(image_count, reshuffle_each_iteration=False)
    val_size = int(image_count * TEST_PCTG)
    train_ds = list_ds.skip(val_size)
    val_ds = list_ds.take(val_size)

    train_ds = train_ds.map(process_path, num_parallel_calls=1000)
    val_ds = val_ds.map(process_path, num_parallel_calls=1000)
    train_ds = configure_for_performance(train_ds)
    val_ds = configure_for_performance(val_ds)

    return train_ds, val_ds

def cnn_model():

    input_A = layers.Input(shape=(IMG_HEIGHT, IMG_WIDTH, 1), name='image')
    input_B = layers.Input(shape=(3,), name='odometry')

    # Conv2D inputA
    Conv2d1 = layers.Conv2D(6, (5, 5), activation="tanh")(input_A)
    max_pool_2d1 = layers.MaxPooling2D(pool_size=(2, 2))(Conv2d1)

    Conv2d2 = layers.Conv2D(6, (5, 5), activation="tanh")(max_pool_2d1)
    max_pool_2d2 = layers.MaxPooling2D(pool_size=(2, 2))(Conv2d2)

    Conv2d3 = layers.Conv2D(6, (5, 5), activation="tanh")(max_pool_2d2)
    max_pool_2d3 = layers.MaxPooling2D(pool_size=(2, 2))(Conv2d3)

    flatten = layers.Flatten()(max_pool_2d3)
    norm = layers.BatchNormalization()(flatten)

    hidden1 = layers.Dense(120, activation="relu")(norm)
    hidden2 = layers.Dense(84, activation='relu')(hidden1)
    hidden3 = layers.Dense(16, activation='relu')(hidden2)
    hidden4 = layers.Dense(16, activation='relu')(hidden3)

    denseB_1 = layers.Dense(16, activation='relu')(input_B)
    denseB_2 = layers.Dense(16, activation='relu')(denseB_1)

    concat_A = layers.Concatenate()([denseB_2, hidden4])

    hidden5 = layers.Dense(32, activation='relu') (concat_A)
    hidden6 = layers.Dense(32, activation='relu')(hidden5)
    hidden7 = layers.Dense(16, activation='relu')(hidden6)

    output = layers.Dense(3, activation='softmax')(hidden7)

    model = Model(inputs=[input_A, input_B], outputs=[output])

    model.compile(optimizer='adamax', loss='categorical_crossentropy',
                  metrics=['accuracy'])

    return model


def balance_ds_directory():
    all_filenames = glob.glob("./dataset/*.jpg")
    category_samples = [0, 0, 0]
    category_paths = [[], [], []]
    for path in all_filenames:
        filename = os.path.basename(path)
        r = re.findall(REGEX, filename)
        category = r[0][0] + "_" + r[0][1]
        one_hot = [1 if x == category else 0 for x in CATEGORIES]
        category_index = np.argmax(one_hot)
        category_samples[category_index] += 1
        category_paths[category_index].append(path)
    samples_per_category = min(category_samples)
    for category_names in category_paths:
        imgs_to_be_deleted = len(category_names) - samples_per_category
        random.shuffle(category_names)
        for i in range(0, imgs_to_be_deleted):
            system("rm " + category_names[i])

# balance_ds_directory()
train_ds, test_ds = load_datsets()

model = cnn_model()

callback = tf.keras.callbacks.EarlyStopping(monitor='accuracy', patience=3)
model.fit(
    train_ds, 
    validation_data=test_ds,
    epochs=EPOCHS,
    callbacks=[callback],
)
system("rm tf_model_driving.h5")
model.save('tf_model_driving.h5', include_optimizer=False)
