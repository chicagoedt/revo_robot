from keras.models import load_model
from keras.preprocessing import ImageDataGenerator

import sys

img_height = 224
img_width = 224
img_size = (img_height, img_width)
input_shape = (img_height, img_width, 3)
batch_size = 256
epochs = 50

model = load_model(sys.argv[1])

datagen = ImageDataGenerator(
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        horizontal_flip=True,
        zoom_range=0.2,
        rescale=1./255)

training_generator = datagen.flow_from_directory(
        'data/training',
        target_size=img_size,
        batch_size=batch_size)

validation_generator = datagen.flow_from_directory(
        'data/validation',
        target_size=img_size,
        batch_size=batch_size)

model.fit_generator(
        training_generator,
        steps_per_epoch=8,
        epochs=epochs,
        validation_data=validation_generator,
        validation_steps=2)
