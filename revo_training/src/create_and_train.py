from keras.applications.vgg16 import VGG16
from keras.models import Sequential, load_model
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint, EarlyStopping
import cv2
import string, random

# HYPERPARAMETERS
img_height = 224
img_width = 224
img_size = (img_height, img_width)
mask_size = (28,28)
input_shape = (img_height, img_width, 3)
batch_size = 8
epochs = 500
steps_per_epoch = int(1631/batch_size) + 1
validation_steps = int(431/batch_size) + 1
seed = 1

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


def buildModel():
    model = Sequential()

    model.add(Conv2D(64, (3,3), padding='same', activation='relu', name='block1_conv1', input_shape=(224,224,3)))
    model.add(Conv2D(64, (3,3), padding='same', activation='relu', name='block1_conv2'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(128, (3,3), padding='same', activation='relu', name='block2_conv1'))
    model.add(Conv2D(128, (3,3), padding='same', activation='relu', name='block2_conv2'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(256, (3,3), padding='same', activation='relu', name='block3_conv1'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', name='block3_conv2'))
    model.add(Conv2D(256, (3,3), padding='same', activation='relu', name='block3_conv3'))
    model.add(MaxPooling2D((2,2)))

    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block4_conv1'))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block4_conv2'))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block4_conv3'))

    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block5_conv1', dilation_rate=2))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block5_conv2', dilation_rate=2))
    model.add(Conv2D(512, (3,3), padding='same', activation='relu', name='block5_conv3', dilation_rate=2))

    model.add(Conv2D(4096, (7,7), padding='same', activation='relu', name='conv6', dilation_rate=4))
    model.add(Dropout(0.5))
    model.add(Conv2D(4096, (1,1), padding='same', activation='relu', name='conv7'))
    model.add(Dropout(0.5))
    model.add(Conv2D(1, (1,1), padding='same', activation='sigmoid', name='pred'))

    #model.add(UpSampling2D(size=(8,8)))

    model.load_weights('vgg16_imagenet_weights.h5', by_name=True)
    for layer in model.layers:
        if layer.name == 'block5_conv1':
            break
        else:
            layer.trainable = False

    model.compile(optimizer='adadelta', loss='mean_squared_error', metrics=['accuracy'])

    plot_model(model, 'dilated.png', show_shapes=True)

    return model

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

model = buildModel()
best_model = load_model('best.h5')
best_model.save_weights('best_weights.h5')
model.load_weights('best_weights.h5', by_name=True)
'''
model.fit_generator(
        train_generator,
        steps_per_epoch=steps_per_epoch,
        epochs=epochs,
        callbacks=[checkpoint, tb, early],
	validation_data=val_generator,
	validation_steps=validation_steps)
'''
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
