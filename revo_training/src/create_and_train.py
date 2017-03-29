from keras.applications.vgg16 import VGG16
from keras.models import Sequential
from keras.utils import plot_model
from keras.layers import Conv2D, MaxPooling2D, Dropout, UpSampling2D
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint

# HYPERPARAMETERS
img_height = 224
img_width = 224
img_size = (img_height, img_width)
input_shape = (img_height, img_width, 3)
batch_size = 2
epochs = 1000
steps_per_epoch = int(1631/batch_size) + 1
steps_on_val = int(431/batch_size) + 1

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
    model.compile(optimizer='adadelta', loss='mean_squared_error', metrics=['accuracy'])

    plot_model(model, 'dilated.png', show_shapes=True)

    return model

data_gen_args = dict(rotation_range=30.,
                     width_shift_range=0.2,
                     height_shift_range=0.2,
                     zoom_range=0.2,
                     horizontal_flip=True,
                     rescale=1./255)

image_datagen = ImageDataGenerator(**data_gen_args)
mask_datagen = ImageDataGenerator(**data_gen_args)

seed = 1
image_generator = image_datagen.flow_from_directory(
        'data/segmentation/training/images/',
        target_size=img_size,
        batch_size=batch_size,
        class_mode=None,
        seed=seed)
mask_generator = mask_datagen.flow_from_directory(
        'data/segmentation/training/masks/',
        target_size=(28,28),
        color_mode='grayscale',
        batch_size=batch_size,
        class_mode=None,
        seed=seed)

train_generator = zip3(image_generator, mask_generator)

checkpoint = ModelCheckpoint(
        'best.h5',
        monitor='loss',
        verbose=0,
        save_best_only=True)

tb = TensorBoard(
        log_dir='./logs',
        histogram_freq=0,
        write_graph=True,
        write_images=True)

model = buildModel()

model.fit_generator(
        train_generator,
        steps_per_epoch=steps_per_epoch,
        epochs=epochs,
        callbacks=[checkpoint, tb])

'''
for x,y in train_generator:
    print(x.shape)
    print(y.shape)
    model.train_on_batch(x,y)
    '''
