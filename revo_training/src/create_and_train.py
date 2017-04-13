from keras.applications.vgg16 import VGG16
from keras.models import Sequential, Model, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D, Input, BatchNormalization, add, concatenate, PReLU, Add, Conv2DTranspose
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping
from keras import backend as K
import cv2
import string, random, time
import numpy as np
import tensorflow as tf

# HYPERPARAMETERS
#img_height = 648
#img_width = 1280
img_height = 224
img_width = 224
img_size = (img_height, img_width)
mask_size = (112,112)
input_shape = (img_height, img_width, 3)
batch_size = 4
epochs = 500
steps_per_epoch = int(944/batch_size) + 1
validation_steps = int(251/batch_size) + 1
seed = 1

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

def buildModel():
    model = Sequential()
    model.add(Conv2D(64, (11,11), padding='same', activation='relu', input_shape=input_shape))
    model.add(MaxPooling2D())
    model.add(Conv2D(128, (5,5), padding='same', activation='relu'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Conv2D(128, (3,3), padding='same', activation='relu', dilation_rate=2))
    model.add(Dropout(0.5))
    model.add(Conv2D(512, (7,7), padding='same', activation='relu', dilation_rate=4))
    model.add(Dropout(0.25))
    model.add(Conv2D(1024, (1,1), padding='same', activation='relu'))
    model.add(Conv2D(1, (1,1), padding='same', activation='sigmoid'))
    return model

def buildSimpleModel():
    img = Input(shape=input_shape)
    conv1_1 = Conv2D(64, (3,3), padding='same')(img)
    conv1_2 = Conv2D(64, (3,3), padding='same', activation='relu')(conv1_1)

    conv2_1 = Conv2D(128, (3,3), padding='same', dilation_rate=2)(conv1_2)
    conv2_2 = Conv2D(128, (3,3), padding='same', dilation_rate=2, activation='relu')(conv2_1)

    conv3_1 = Conv2D(256, (3,3), padding='same', dilation_rate=4)(conv2_2)
    conv3_2 = Conv2D(256, (3,3), padding='same', dilation_rate=4, activation='relu')(conv3_1)

    msk = Conv2D(1, (1,1), padding='same', activation='relu')(conv3_2)

    return Model(inputs=img, outputs=msk)

model = buildSimpleModel()
#model = load_model('best.h5')
model.compile(loss='binary_crossentropy', optimizer='adadelta', metrics=['accuracy'])

data_gen_args = dict(rotation_range=30.,
                     width_shift_range=0.2,
                     height_shift_range=0.2,
                     zoom_range=0.2,
		     fill_mode='constant',
                     horizontal_flip=True,
                     rescale=1./255)

image_datagen = ImageDataGenerator(**data_gen_args)
mask_datagen = ImageDataGenerator(**data_gen_args)

image_generator = image_datagen.flow_from_directory(
        'data/segmentation_lines/training/images/',
        target_size=img_size,
        batch_size=batch_size,
        class_mode=None,
        seed=seed)
mask_generator = mask_datagen.flow_from_directory(
        'data/segmentation_lines/training/masks/',
        target_size=mask_size,
        color_mode='grayscale',
        batch_size=batch_size,
        class_mode=None,
        seed=seed)

train_generator = zip3(image_generator, mask_generator)

val_image_datagen = ImageDataGenerator(rescale=1./255)
val_mask_datagen = ImageDataGenerator(rescale=1./255)

val_image_generator = val_image_datagen.flow_from_directory(
	'data/segmentation_lines/validation/images/',
	target_size=img_size,
	batch_size=batch_size,
	class_mode=None,
	seed=seed)
val_mask_generator = val_mask_datagen.flow_from_directory(
	'data/segmentation_lines/validation/masks/',
	target_size=mask_size,
	color_mode='grayscale',
	batch_size=batch_size,
	class_mode=None,
	seed=seed)

val_generator = zip3(val_image_generator, val_mask_generator)

checkpoint = ModelCheckpoint(
        'best.h5',
        monitor='val_loss',
        verbose=0,
        save_best_only=True)

tb = TensorBoard(
        log_dir='./logs',
        histogram_freq=0,
        write_graph=True,
        write_images=True)

early = EarlyStopping(patience=3, verbose=1)

for x,y in val_generator:
	start = time.clock()
	predictions = model.predict_on_batch(x)
	end = time.clock()
	print("Seconds per image: " + str((end - start) / batch_size))
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
