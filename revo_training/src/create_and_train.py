from keras.applications.vgg16 import VGG16
from keras.models import Sequential, Model, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D, Input, BatchNormalization, add, concatenate, PReLU, Add, Conv2DTranspose
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping
from keras import backend as K
import cv2
import string, random
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
steps_per_epoch = int(1406/batch_size) + 1
validation_steps = int(361/batch_size) + 1
seed = 1
internal_scale = 0.5

vgg = VGG16(include_top=False, weights='imagenet', input_shape=input_shape)
plot_model(vgg, 'vgg.png', show_shapes=True)
vgg.save_weights('vgg16_imagenet_weights.h5')

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

def addBottleneck(model, output, downsampling=False, upsampling=False, asymmetric=False, dilation_rate=(1,1)):
    if downsampling:
        shape = (batch_size, int(int(model.shape[1]) / 2), int(int(model.shape[2]) / 2), int(output - int(model.shape[3])))
        x = tf.convert_to_tensor(np.zeros(shape))
        main = Conv2D(output, (shape[1],shape[2]), padding='same')(MaxPooling2D()(model))
        projection = PReLU()(BatchNormalization()(Conv2D(int(output * internal_scale), (2,2), strides=(2,2), padding='same')(model)))
    elif upsampling:
        main = Conv2D(output, (int(model.shape[1]), int(model.shape[2])), padding='same')(UpSampling2D()(model))
        projection = PReLU()(BatchNormalization()(Conv2D(int(output * internal_scale), (1,1), padding='same')(model)))
    else:
        main = model
        projection = PReLU()(BatchNormalization()(Conv2D(int(output * internal_scale), (1,1), padding='same')(model)))

    if asymmetric:
        conv = PReLU()(BatchNormalization()(Conv2D(int(output * internal_scale), (5,1), padding='same')(Conv2D(int(output * internal_scale), (1,5), padding='same')(projection))))
    else:
        conv = PReLU()(BatchNormalization()(Conv2D(int(output * internal_scale), (3,3), padding='same', dilation_rate=dilation_rate)(projection)))

    if upsampling:
        #expansion = PReLU()(BatchNormalization()(Conv2DTranspose(output, (1,1), strides=(2,2), padding='same')(conv)))
        expansion = UpSampling2D()(PReLU()(BatchNormalization()(Conv2D(output, (1,1), padding='same')(conv))))
    else:
        expansion = PReLU()(BatchNormalization()(Conv2D(output, (1,1), padding='same')(conv)))
    conv_branch = Dropout(0.01)(expansion)

    out = PReLU()(add([conv_branch, main]))

    return out

def buildModel():
    i = Input(shape=input_shape)
    c = Conv2D(13, (3,3), strides=(2,2), padding='same')(i)
    p = MaxPooling2D()(i)
    initial_block_output = concatenate([c,p])

    bottleneck_1_0 = addBottleneck(initial_block_output, 64, downsampling=True)
    bottleneck_1_1 = addBottleneck(bottleneck_1_0, 64)
    bottleneck_1_2 = addBottleneck(bottleneck_1_1, 64)
    bottleneck_1_3 = addBottleneck(bottleneck_1_2, 64)
    bottleneck_1_4 = addBottleneck(bottleneck_1_3, 64)

    bottleneck_2_0 = addBottleneck(bottleneck_1_4, 128, downsampling=True)
    bottleneck_2_1 = addBottleneck(bottleneck_2_0, 128)
    bottleneck_2_2 = addBottleneck(bottleneck_2_1, 128, dilation_rate=(2,2))
    bottleneck_2_3 = addBottleneck(bottleneck_2_2, 128, asymmetric=True)
    bottleneck_2_4 = addBottleneck(bottleneck_2_3, 128, dilation_rate=(4,4))
    bottleneck_2_5 = addBottleneck(bottleneck_2_4, 128)
    bottleneck_2_6 = addBottleneck(bottleneck_2_5, 128, dilation_rate=(8,8))
    bottleneck_2_7 = addBottleneck(bottleneck_2_6, 128, asymmetric=True)
    bottleneck_2_8 = addBottleneck(bottleneck_2_7, 128, dilation_rate=(16,16))

    bottleneck_3_1 = addBottleneck(bottleneck_2_8, 128)
    bottleneck_3_2 = addBottleneck(bottleneck_3_1, 128, dilation_rate=(2,2))
    bottleneck_3_3 = addBottleneck(bottleneck_3_2, 128, asymmetric=True)
    bottleneck_3_4 = addBottleneck(bottleneck_3_3, 128, dilation_rate=(4,4))
    bottleneck_3_5 = addBottleneck(bottleneck_3_4, 128)
    bottleneck_3_6 = addBottleneck(bottleneck_3_5, 128, dilation_rate=(8,8))
    bottleneck_3_7 = addBottleneck(bottleneck_3_6, 128, asymmetric=True)
    bottleneck_3_8 = addBottleneck(bottleneck_3_7, 128, dilation_rate=(16,16))

    bottleneck_4_0 = addBottleneck(bottleneck_3_8, 64, upsampling=True)
    bottleneck_4_1 = addBottleneck(bottleneck_4_0, 64)
    bottleneck_4_2 = addBottleneck(bottleneck_4_1, 64)

    bottleneck_5_0 = addBottleneck(bottleneck_4_2, 32, upsampling=True)
    bottleneck_5_1 = addBottleneck(bottleneck_5_0, 32)

    fullconv = Conv2D(1, (1,1))(bottleneck_5_1)

    return Model(inputs=i, outputs=fullconv)

model = buildModel()
#plot_model(model, 'enet.png', show_shapes=True)
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

#model = buildModel()
#model = load_model('best.h5')
#model.load_weights('best_weights.h5')
#model.save('lane_finder.h5')

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
