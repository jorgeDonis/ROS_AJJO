#!/usr/bin/env python
# coding: utf-8

# # Diferenciación entre robot y pared

# In[68]:

# ### Import packages

# In[69]:


import matplotlib.pyplot as plt
import numpy as np
import collections
from keras.callbacks import EarlyStopping
import cv2, glob

import sklearn
from sklearn.model_selection import train_test_split
from sklearn import metrics
from sklearn.model_selection import StratifiedKFold

import tensorflow as tf
import pathlib

from tensorflow import keras
from tensorflow.keras.models import Sequential
from tensorflow.keras import layers


# ### Define global constants

# In[70]:


batch_size = 32
epochs = 60

activation_function = "tanh"
optimizer_function = "adamax"

# Scaling input image to theses dimensions
img_rows, img_cols = 128, 64


# In[71]:


def balance_ds(X, Y):
    robot_indices = [i for i, x in enumerate(Y) if np.array_equal(x, [0, 1])]
    wall_indices = [i for i, x in enumerate(Y) if np.array_equal(x, [1, 0])]
    final_samples_per_class = min(len(robot_indices), len(wall_indices))
    new_X = []
    new_Y = []
    for i in range(0, round(final_samples_per_class / 2)):
        new_X.append(X[robot_indices[i]])
        new_Y.append(Y[robot_indices[i]])
        new_X.append(X[wall_indices[i]])
        new_Y.append(Y[wall_indices[i]])
    return np.array(new_X), np.array(new_Y)


# In[72]:


def readImg(nameImg):
    img = cv2.imread(nameImg)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = np.delete(img, slice(0, 100), 0)
    img = np.delete(img, slice(290, img.shape[0]), 0)
    print(img.shape)
    img = cv2.resize(img, (img_rows, img_cols), interpolation=cv2.INTER_LANCZOS4)
    print(img.shape)
    img = np.reshape(img, (img.shape[0], img.shape[1], 1))
    img = img / 255
    return img


# In[73]:


def load_data():
    robotNP = list()
    paredNP = list()

    for nameImg in glob.glob('clases/Pared/*'):
        paredNP.append(readImg(nameImg))
    for nameImg in glob.glob('clases/Robot/*'):
        robotNP.append(readImg(nameImg))

    X = list()
    y = list()

    for item in paredNP:
        X.append(item)
        y.append(0)

    for item in robotNP:
        X.append(item)
        y.append(1)

    return np.array(X), np.array(y)


# ### Plot images

# In[74]:


def plot_symbols(img,name):
  cv2.imshow(name, img)
  cv2.waitKey(0)
  cv2.destroyAllWindows()  


# In[75]:



def cnn_model():
    
    model = Sequential()


    model.add(layers.Conv2D(6, (5, 5)))
    model.add(layers.Activation(activation_function))
    model.add(layers.MaxPooling2D(pool_size=(2, 2)))
    
    model.add(layers.Conv2D(16, (5, 5)))
    model.add(layers.Activation(activation_function))
    model.add(layers.MaxPooling2D(pool_size=(2, 2)))
        
    model.add(layers.Flatten())
    
    model.add(layers.Dense(120))
    model.add(layers.Activation(activation_function))
    
    model.add(layers.Dense(84))
    model.add(layers.Activation(activation_function))
    
    model.add(layers.Dense(2))
    model.add(layers.Activation('softmax'))

    return model


# ### Load data

# In[76]:


X, y = load_data()


# ### Split examples in training/test sets

# In[77]:


# (shuffle)
def zafel(X, y):
    index = np.arange(y.size)
    np.random.shuffle(index)
    X_old = np.copy(X)
    y_old = np.copy(y) 

    for i in range(y.size):
        X[i] = X_old[index[i]] 
        y[i] = y_old[index[i]] 
    return X, y


# In[78]:


X, y = zafel(X,y)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.25, random_state=42)

# Convert integers to one-hot vector
y_train_nn = keras.utils.to_categorical(y_train, 2)
y_test_nn = keras.utils.to_categorical(y_test, 2)
y_nn = keras.utils.to_categorical(y, 2)
X, y_nn = balance_ds(X, y_nn)

print("X_train {} X_test {}".format(X_train.shape, X_test.shape))
print('y_train {} y_test {}'.format(y_train.shape ,y_test.shape))


# ## Model and optimizers (choose one)

# In[79]:


### Simple


# In[80]:



model = cnn_model()

model.compile(loss='categorical_crossentropy',optimizer=optimizer_function, metrics=['accuracy'])
early_stopping = EarlyStopping(monitor='val_loss', patience=10)
model.fit(X_train, y_train_nn, batch_size=batch_size, epochs=epochs, validation_split=0.3, verbose=2, callbacks=[early_stopping])


scores = model.evaluate(X_test, y_test_nn, verbose=0)
print(' {} of {}; {} of {}%'.format(model.metrics_names[0],scores[0],model.metrics_names[1],scores[1]*100))
acc = scores[1] * 100
loss = scores[0]

#print(model.summary())


# In[48]:


y_scores = model.predict(X_test) # Confidence prediction per class
y_pred = y_scores.argmax(axis=1) # Select classes with most confidence prediction


print(y_pred.shape)
print("Acc {}".format(acc))
print("Loss {}".format(loss))

loss, acc = model.evaluate(X_test, y_test_nn, batch_size=batch_size)
print('loss: {} acc: {}'.format(loss,acc))


# In[49]:


print('Predictions', collections.Counter(y_pred),'\n')

print('Confusion matrix')
print(metrics.confusion_matrix(y_test,y_pred),'\n')

target_names = ['PARED', 'ROBOT']

print(metrics.classification_report(y_test, y_pred, target_names=target_names))


# In[50]:


#print(y_pred)
#for i in range(y_pred.size):\n",
    #plot_symbols(X_test[i],"windiw{}".format(i)) #mostrar en X capa y no en la 1a
#plot_symbols_it(it,15)


# In[ ]:


model.fit(X, y_nn, batch_size = batch_size, epochs = epochs)
model.save('tf_model_driving.h5', include_optimizer=False)
        

