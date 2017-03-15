#import all required
import time
import json
from keras.models import Model
from keras.layers import Dense, GlobalAveragePooling2D, Dropout
from keras import backend as K
from keras.applications.vgg16 import VGG16
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input
from keras.optimizers import Adam
import tensorflow as tf
from sklearn.cross_validation import train_test_split
from sklearn.utils import shuffle
from keras.models import Model
import pandas as pd
from scipy.misc import imread
import numpy as np
from skimage.transform import resize
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

#read the file
input_file=pd.read_csv('.\\data\\data\\driving_log.csv')
X_train=[]
Y_train=[]

#read center images and normalize them and add to input dataset
for i in range(8035):
    img=imread('.\\data\\data\\'+input_file.ix[i][0])
    norm_image=img
    cv2.normalize(img,norm_image, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    X_train.append(norm_image)
    Y_train.append(input_file.ix[i][3])
	
#read left and right images and normalize them and add to input dataset
for i in range(8035):
    imgl=imread('.\\data\\data\\'+input_file.ix[i][1])
    norm_imagel =imgl
    cv2.normalize(imgl, norm_imagel, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    
    imgr=imread('.\\data\\data\\'+input_file.ix[i][2])
    norm_imager = imgr
    cv2.normalize(imgr, norm_imager, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
    
    X_train.append(norm_imagel)
    Y_train.append(input_file.ix[i][3] + 0.25)
    X_train.append(norm_imager)
    Y_train.append(input_file.ix[i][3] - 0.25)
	
#flip images
for i in range(len(X_train)):
    image_loop=X_train[i]
    image_flipped=cv2.flip(X_train[i],1)
    X_train.append(image_flipped)
    Y_train.append(0 - Y_train[i])

#crop & resize
length_initial=len(X_train)
image_shape=X_train[0].shape
a=round(image_shape[0]*30/100)
b=image_shape[0]
c=0
d=image_shape[1]
for i in range(length_initial):
    image_loop=X_train[i]
    X_train[i]=image_loop[a:b,c:d]
    X_train[i]=cv2.resize(X_train[i],(64,64))
	
# create the base pre-trained model
base_model = VGG16(weights='imagenet', include_top=False)

#set all layers of base model as trainable
for layer in base_model.layers[0:]:
   layer.trainable = True
   
#a global spatial average pooling layer
x = base_model.output
x = GlobalAveragePooling2D()(x)

#Dropout Layer
x= (Dropout(0.5))(x)

# Add two fully-connected layer
x = Dense(1024, activation='elu')(x)
x = Dense(512, activation='elu')(x)

#logistic layer
predictions = Dense(1, activation='linear')(x)

# Model to be trained
model = Model(input=base_model.input, output=predictions)

#compile Model
model.compile(optimizer=Adam(lr=0.00001),loss='mae')

#split into train and validation set
X_train, X_val, Y_train, Y_val = train_test_split(X_train, Y_train, random_state=0, test_size=0.05)

#Convert to numpy array
X_train=np.asarray(X_train)
Y_train=np.asarray(Y_train)
X_val=np.asarray(X_val)
Y_val=np.asarray(Y_val)

#Train Model
model.fit(X_train, Y_train, batch_size=32, nb_epoch=7, verbose=1, validation_data=(X_val, Y_val))

#freeze weights till layer 14 and train from 15th layer
for layer in model.layers[:15]:
   layer.trainable = False
for layer in model.layers[15:]:
   layer.trainable = True
   
#Retrain Model
model.fit(X_train, Y_train, batch_size=32, nb_epoch=7, verbose=1, validation_data=(X_val, Y_val))   


#Save Model
json_string = model.to_json()
with open('model.json','w') as f:
    json.dump(json_string,f,ensure_ascii=False)
model.save_weights('model.h5')