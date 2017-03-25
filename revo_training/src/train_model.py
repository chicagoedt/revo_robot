from keras.models import load_model
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import TensorBoard, ModelCheckpoint
import numpy as np
import sys

img_height = 224
img_width = 224
img_size = (img_height, img_width)
input_shape = (img_height, img_width, 3)
batch_size = 64
epochs = 50

model = load_model(sys.argv[1])
model.compile(optimizer='adadelta', loss='categorical_crossentropy', metrics=['accuracy'])

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

model.fit_generator(
        training_generator,
        steps_per_epoch=25,
        epochs=epochs,
        callbacks=[checkpoint, tb],
        validation_data=validation_generator,
        validation_steps=7)
