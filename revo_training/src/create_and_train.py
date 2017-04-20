from keras.applications.vgg16 import VGG16
from keras.models import Sequential, Model, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D, Input, BatchNormalization, add, concatenate, PReLU, Add, Conv2DTranspose
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping
from keras import backend as K
import cv2
import string, random, time, sys
import numpy as np
import tensorflow as tf

import revo_lf_models
import revo_lf_generators

# HYPERPARAMETERS
#img_height = 648
#img_width = 1280
img_height = 224
img_width = 224
img_size = (img_height, img_width)
mask_size = (56,56)
input_shape = (img_height, img_width, 3)
batch_size = 32
epochs = 500
steps_per_epoch = int(2459/batch_size) + 1
validation_steps = int(647/batch_size) + 1
seed = 1
model_name= sys.argv[1]

def zip3(*iterables):
    # zip('ABCD', 'xy') --> Ax By
    sentinel = object()
    iterators = [iter(it) for it in iterables]
    while iterators:
        result = []
        for it in iterators:
            elem = next(it, sentinel)
            if elem is sentinel:
                return
            result.append(elem)
        yield tuple(result)

if sys.argv[2] == '-n':
    model = buildModelD1()
elif sys.argv[2] == '-l':
    model = load_model(model_name)

model.compile(loss='binary_crossentropy', optimizer='adadelta', metrics=['accuracy'])

checkpoint = ModelCheckpoint(
        model_name,
        monitor='val_loss',
        verbose=0,
        save_best_only=True)

tb = TensorBoard(
        log_dir='./logs',
        histogram_freq=0,
        write_graph=True,
        write_images=True)

early = EarlyStopping(patience=batch_size, verbose=1)

j = 0
for x,y in val_generator:
	start = time.clock()
	predictions = model.predict_on_batch(x)
	end = time.clock()
	print("Seconds per image: " + str((end - start) / batch_size))
	j += 1
	if j > 5:
		break

model.fit_generator(
	train_generator,
	steps_per_epoch=steps_per_epoch,
	epochs=epochs,
	callbacks=[checkpoint, tb, early],
	validation_data=val_generator,
	validation_steps=validation_steps)

def getID(size=6, chars=string.ascii_lowercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))

#model = load_model(model_name)
j = 0
for x,y in val_generator:
	j += 1
	predictions = model.predict_on_batch(x)
	print(str(j) + '/' + str(validation_steps))
	for i in range(len(predictions)):
		ID = getID()
		cv2.imwrite('results/' + ID + '_img.png', x[i]*255)
		cv2.imwrite('results/' + ID + '_mask.png', y[i]*255)
		cv2.imwrite('results/' + ID + '_pred.png', predictions[i]*255)
		print(predictions[i].max())
	if j >= validation_steps:
		break
