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

# HYPERPARAMETERS
#img_height = 648
#img_width = 1280
img_height = 224
img_width = 224
img_size = (img_height, img_width)
mask_size = (112,112)
input_shape = (img_height, img_width, 3)
batch_size = 32
epochs = 500
steps_per_epoch = int(2092/batch_size) + 1
validation_steps = int(553/batch_size) + 1
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

# 0.05 sec/img, converges to ~0.98 val_acc
def buildModelA():
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

# 0.006 sec/img, converges to ~0.9754 val_acc
def buildModelB1():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(Dropout(0.25)(conv2_1))
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(Dropout(0.25)(conv2_2))
    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv2_3)

    return Model(inputs=img, outputs=pred)

# 0.0065 sec/img
def buildModelB2():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv3_3)

    return Model(inputs=img, outputs=pred)

# 0.0128 sec/img, converges to 0.945 val_acc
def buildModelB3():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same')(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same')(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same')(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same')(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same')(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same')(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.013 sec/img, converges to ~0.977 val_acc
def buildModelB4():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same')(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same')(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same')(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same')(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.016 sec/img, converges to ~0.983 val_acc
def buildModelB5():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=8, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv5_3)
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=16, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(conv6_3)

    return Model(inputs=img, outputs=pred)

# 0.0163 sec/img, converges to ~0.985 val_acc
def buildModelB6():
    img = Input(shape=input_shape)
    inception_1 = Conv2D(8, (1,1), padding='same')(img)
    inception_3 = Conv2D(8, (3,3), padding='same')(img)
    inception_5 = Conv2D(8, (5,5), padding='same')(img)

    conv1 = Conv2D(32, (3,3), padding='same', activation='relu')(concatenate([inception_1, inception_3, inception_5]))
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    conv5_1 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv4_3)
    conv5_2 = Conv2D(64, (3,3), padding='same', dilation_rate=8)(conv5_1)
    conv5_3 = Conv2D(64, (3,3), padding='same', dilation_rate=8, activation='relu')(conv5_2)

    conv6_1 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(Dropout(0.25)(conv5_3))
    conv6_2 = Conv2D(64, (3,3), padding='same', dilation_rate=16)(conv6_1)
    conv6_3 = Conv2D(64, (3,3), padding='same', dilation_rate=16, activation='relu')(conv6_2)

    pred = Conv2D(1, (5,5), padding='same', activation='sigmoid')(Dropout(0.5)(conv6_3))

    return Model(inputs=img, outputs=pred)

#
def buildModelC1():
    img = Input(shape=input_shape)

    conv1 = Conv2D(8, (1,1), padding='same')(img)
    pool1 = MaxPooling2D()(conv1)

    conv2_1 = Conv2D(64, (3,3), padding='same')(pool1)
    conv2_2 = Conv2D(64, (3,3), padding='same')(conv2_1)
    conv2_3 = Conv2D(64, (3,3), padding='same', activation='relu')(conv2_2)

    conv3_1 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv2_3)
    conv3_2 = Conv2D(64, (3,3), padding='same', dilation_rate=2)(conv3_1)
    conv3_3 = Conv2D(64, (3,3), padding='same', dilation_rate=2, activation='relu')(conv3_2)

    conv4_1 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv3_3)
    conv4_2 = Conv2D(64, (3,3), padding='same', dilation_rate=4)(conv4_1)
    conv4_3 = Conv2D(64, (3,3), padding='same', dilation_rate=4, activation='relu')(conv4_2)

    pred = Conv2D(1, (5,5), padding='same', dilation_rate=4, activation='sigmoid')(Dropout(0.5)(conv6_3))

    return Model(inputs=img, outputs=pred)


if sys.argv[2] == '-n':
    model = buildModelB6()
elif sys.argv[2] == '-l':
    model = load_model(model_name)

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
        'data/segmentation/training/images/',
        target_size=img_size,
        batch_size=batch_size,
        class_mode=None,
        seed=seed)
mask_generator = mask_datagen.flow_from_directory(
        'data/segmentation/training/masks/',
        target_size=mask_size,
        color_mode='grayscale',
        batch_size=batch_size,
        class_mode=None,
        seed=seed)

train_generator = zip3(image_generator, mask_generator)

val_image_datagen = ImageDataGenerator(rescale=1./255)
val_mask_datagen = ImageDataGenerator(rescale=1./255)

val_image_generator = val_image_datagen.flow_from_directory(
	'data/segmentation/validation/images/',
	target_size=img_size,
	batch_size=batch_size,
	class_mode=None,
	seed=seed)
val_mask_generator = val_mask_datagen.flow_from_directory(
	'data/segmentation/validation/masks/',
	target_size=mask_size,
	color_mode='grayscale',
	batch_size=batch_size,
	class_mode=None,
	seed=seed)

val_generator = zip3(val_image_generator, val_mask_generator)

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
	if j > 0:
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
